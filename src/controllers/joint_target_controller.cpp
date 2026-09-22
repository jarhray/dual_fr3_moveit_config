#include "dual_fr3_moveit_config/controllers/joint_target_controller.hpp"

#include <chrono>
#include <cstring>
#include <exception>
#include <set>
#include <pluginlib/class_list_macros.hpp>

namespace dual_fr3_moveit_config::controllers {
double JointTargetController::monotonic_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
JointTargetController::CallbackReturn JointTargetController::on_init() {
  try {
    auto_declare<bool>("enabled", false);
    auto_declare<bool>("calibration_verified", false);
    auto_declare<bool>("runtime_limits_verified", false);
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("robot_state_interface", "");
    for (const char* name : {"joint_lower", "joint_upper", "k_gains", "d_gains",
        "max_velocity", "max_acceleration", "max_torque", "max_torque_rate", "max_tracking_error"})
      auto_declare<std::vector<double>>(name, {});
    auto_declare<double>("command_timeout_s", 0.10);
    auto_declare<double>("state_timeout_s", 0.01);
    auto_declare<double>("max_period_s", 0.005);
    auto_declare<double>("future_tolerance_s", 0.005);
    auto_declare<double>("max_force_norm_n", 0.0);
    auto_declare<double>("max_moment_norm_nm", 0.0);
    auto_declare<double>("activation_velocity_rad_s", 0.01);
    auto_declare<double>("stopped_velocity_rad_s", 0.003);
    auto_declare<double>("settle_time_s", 0.1);
    auto_declare<double>("state_publish_rate_hz", 50.0);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
JointTargetController::CallbackReturn JointTargetController::on_configure(
    const rclcpp_lifecycle::State&) {
  try {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    robot_state_interface_ = get_node()->get_parameter("robot_state_interface").as_string();
    if (joints_.size() != 7 || std::set<std::string>(joints_.begin(), joints_.end()).size() != 7 ||
        robot_state_interface_.empty()) throw std::runtime_error("seven distinct joints and robot_state_interface required");
    const auto suffix = robot_state_interface_.rfind("/robot_state");
    if (suffix == std::string::npos || suffix + 12 != robot_state_interface_.size())
      throw std::runtime_error("robot_state_interface must end with /robot_state");
    const auto arm_name = robot_state_interface_.substr(0, suffix);
    for (size_t i = 0; i < 7; ++i)
      if (joints_[i] != arm_name + "_joint" + std::to_string(i + 1))
        throw std::runtime_error("joints must match robot_state arm and native joint1..joint7 order");
    auto array = [this](const char* name, Joints& dest) {
      const auto values = get_node()->get_parameter(name).as_double_array();
      if (values.size() != 7) throw std::runtime_error(std::string(name) + " must have seven values");
      std::copy(values.begin(), values.end(), dest.begin());
    };
    array("joint_lower", config_.lower); array("joint_upper", config_.upper);
    array("k_gains", config_.kp); array("d_gains", config_.kd);
    array("max_velocity", config_.max_velocity); array("max_acceleration", config_.max_acceleration);
    array("max_torque", config_.max_torque); array("max_torque_rate", config_.max_torque_rate);
    array("max_tracking_error", config_.max_tracking_error);
    auto scalar = [this](const char* name) { return get_node()->get_parameter(name).as_double(); };
    config_.command_timeout = scalar("command_timeout_s");
    config_.state_timeout = scalar("state_timeout_s");
    config_.max_period = scalar("max_period_s");
    config_.future_tolerance = scalar("future_tolerance_s");
    config_.max_force = scalar("max_force_norm_n");
    config_.max_moment = scalar("max_moment_norm_nm");
    config_.activation_velocity = scalar("activation_velocity_rad_s");
    config_.stopped_velocity = scalar("stopped_velocity_rad_s");
    config_.settle_time = scalar("settle_time_s");
    const double rate = scalar("state_publish_rate_hz");
    if (!config_.valid() || !std::isfinite(rate) || rate <= 0.0 || rate > 1000.0)
      throw std::runtime_error("invalid limits; configure verified finite positive values");
    publish_period_ = 1.0 / rate;
    guard_.configure(config_);
    state_pub_ = get_node()->create_publisher<msg::JointTargetState>("~/state", rclcpp::QoS(1).reliable());
    state_rt_pub_ = std::make_unique<realtime_tools::RealtimePublisher<msg::JointTargetState>>(state_pub_);
    state_rt_pub_->msg_.state.reserve(32);
    state_rt_pub_->msg_.reason.reserve(64);
    target_sub_ = get_node()->create_subscription<msg::JointTarget>(
      "~/target", rclcpp::QoS(1).reliable(),
      [this](const msg::JointTarget::SharedPtr msg) { target_callback(*msg); });
    stop_service_ = get_node()->create_service<std_srvs::srv::Trigger>("~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = active_.load();
        response->message = response->success ? "stop requested; await STOPPED; controller retains effort ownership" : "controller inactive";
        if (response->success) {
          Reason expected = Reason::None;
          requested_stop_.compare_exchange_strong(expected, Reason::Cancelled);
        }
      });
    parameter_guard_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters) {
          const auto& name = parameter.get_name();
          const bool gate = name == "enabled" || name == "calibration_verified" ||
            name == "runtime_limits_verified";
          if (active_.load() || !gate || parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "Configured limits are immutable; deactivate/cleanup before changing them. Active gates are immutable; use stop.";
            break;
          }
        }
        return result;
      });
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Configuration failed: %s", error.what());
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration JointTargetController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joints_) result.names.push_back(joint + "/effort");
  return result;
}
controller_interface::InterfaceConfiguration JointTargetController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  result.names.push_back(robot_state_interface_);
  return result;
}
Feedback JointTargetController::feedback() const {
  // Same pointer-valued double interface exported by franka_hardware and used by
  // FrankaRobotState semantic component. memcpy avoids type-punning UB.
  const double address = state_interfaces_[robot_state_index_].get_value();
  static_assert(sizeof(address) == sizeof(franka::RobotState*));
  franka::RobotState* state = nullptr;
  std::memcpy(&state, &address, sizeof(state));
  Feedback result;
  if (state == nullptr) {
    result.robot_time = std::numeric_limits<double>::quiet_NaN();
    return result;
  }
  result.q = state->q;
  result.dq = state->dq;
  result.tau = state->tau_J_d;
  result.wrench = state->O_F_ext_hat_K;
  result.robot_time = state->time.toSec();
  return result;
}
JointTargetController::CallbackReturn JointTargetController::on_activate(const rclcpp_lifecycle::State&) {
  active_ = false;
  for (const char* gate : {"enabled", "calibration_verified", "runtime_limits_verified"}) {
    if (!get_node()->get_parameter(gate).as_bool()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Activation refused: %s is false", gate);
      return CallbackReturn::FAILURE;
    }
  }
  bool state_found = false;
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
    if (state_interfaces_[i].get_name() == robot_state_interface_) { robot_state_index_ = i; state_found = true; }
  if (!state_found || command_interfaces_.size() != 7) return CallbackReturn::FAILURE;
  for (size_t i = 0; i < 7; ++i) {
    bool found = false;
    for (size_t j = 0; j < command_interfaces_.size(); ++j)
      if (command_interfaces_[j].get_name() == joints_[i] + "/effort") { command_index_[i] = j; found = true; }
    if (!found) return CallbackReturn::FAILURE;
  }
  const auto fb = feedback();
  const double now = monotonic_now();
  static std::atomic<uint64_t> session_counter{0};
  const auto next_session = static_cast<uint64_t>(now * 1e9) + (++session_counter);
  if (!guard_.activate(next_session, now, get_node()->now().seconds(), fb)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Activation refused: feedback, initial load/torque/velocity or limits invalid");
    return CallbackReturn::FAILURE;
  }
  for (size_t i = 0; i < 7; ++i) command_interfaces_[command_index_[i]].set_value(fb.tau[i]);
  requested_stop_ = Reason::None;
  last_generation_ = generation_.load();
  session_ = next_session;
  last_publish_ = 0.0;
  active_ = true;
  return CallbackReturn::SUCCESS;
}
JointTargetController::CallbackReturn JointTargetController::on_deactivate(const rclcpp_lifecycle::State&) {
  active_ = false;
  session_ = 0;
  // Retain final effort value; the controller manager/hardware performs the switch.
  // The caller must establish a verified holding successor, never replay an old JTC trajectory.
  return CallbackReturn::SUCCESS;
}
JointTargetController::CallbackReturn JointTargetController::on_cleanup(const rclcpp_lifecycle::State&) {
  active_ = false;
  session_ = 0;
  parameter_guard_.reset();
  target_sub_.reset();
  stop_service_.reset();
  state_rt_pub_.reset();
  state_pub_.reset();
  return CallbackReturn::SUCCESS;
}
void JointTargetController::target_callback(const msg::JointTarget& msg) {
  if (!active_.load()) return;
  Target target;
  target.session = msg.session;
  target.sequence = msg.sequence;
  target.stamp = static_cast<double>(msg.stamp.sec) + static_cast<double>(msg.stamp.nanosec) * 1e-9;
  target.received = monotonic_now();
  target.valid_for = msg.valid_for_s;
  target.q = msg.positions;
  target.dq = msg.velocities;
  Reason error = Reason::None;
  const double ros_now = get_node()->now().seconds();
  if (target.session != session_.load()) error = Reason::WrongSession;
  else if (!finite(target.q) || !finite(target.dq) || !std::isfinite(target.valid_for) ||
      target.valid_for <= 0.0 || target.valid_for > config_.command_timeout ||
      msg.stamp.nanosec >= 1000000000 || target.stamp <= 0.0) error = Reason::InvalidTarget;
  else if (target.stamp > ros_now + config_.future_tolerance || ros_now - target.stamp > target.valid_for)
    error = Reason::Timestamp;
  else if (target.sequence == 0 || (callback_previous_.session == target.session &&
      target.sequence <= callback_previous_.sequence)) error = Reason::Sequence;
  for (size_t i = 0; error == Reason::None && i < 7; ++i) {
    if (target.q[i] < config_.lower[i] || target.q[i] > config_.upper[i]) error = Reason::JointLimit;
    else if (std::abs(target.dq[i]) > config_.max_velocity[i]) error = Reason::Velocity;
    else if (callback_previous_.session == target.session) {
      const double dt = target.stamp - callback_previous_.stamp;
      if (dt <= 0.0 || dt > config_.command_timeout) error = Reason::Timestamp;
      else if (std::abs(target.q[i] - callback_previous_.q[i]) > config_.max_velocity[i] * dt + 1e-10)
        error = Reason::Velocity;
      else if (std::abs(target.dq[i] - callback_previous_.dq[i]) > config_.max_acceleration[i] * dt + 1e-10)
        error = Reason::Acceleration;
    }
  }
  if (error != Reason::None) {
    Reason expected = Reason::None;
    requested_stop_.compare_exchange_strong(expected, error);
    return;
  }
  callback_previous_ = target;
  target.generation = ++generation_;
  target_buffer_.writeFromNonRT(target);
}
void JointTargetController::publish_state(const rclcpp::Time& time, const Feedback& fb) {
  if (!state_rt_pub_->trylock()) return;
  auto& msg = state_rt_pub_->msg_;
  msg.stamp = time;
  msg.session = guard_.session();
  msg.last_sequence = guard_.sequence();
  msg.state.assign(phase_name(guard_.phase()));
  msg.reason.assign(reason_name(guard_.reason()));
  msg.positions = fb.q;
  msg.velocities = fb.dq;
  state_rt_pub_->unlockAndPublish();
}
controller_interface::return_type JointTargetController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (!active_.load()) return controller_interface::return_type::OK;
  const double now = monotonic_now();
  const auto fb = feedback();
  const Reason stop = requested_stop_.exchange(Reason::None);
  if (stop != Reason::None) guard_.stop(stop, fb);
  const auto* target = target_buffer_.readFromRT();
  if (target->generation > last_generation_) {
    last_generation_ = target->generation;
    guard_.accept(*target, now, time.seconds(), fb);
  }
  const auto& torque = guard_.update(now, time.seconds(), period.seconds(), fb);
  for (size_t i = 0; i < 7; ++i) command_interfaces_[command_index_[i]].set_value(torque[i]);
  if (now - last_publish_ >= publish_period_) {
    publish_state(time, fb);
    last_publish_ = now;
  }
  return controller_interface::return_type::OK;
}
}  // namespace dual_fr3_moveit_config::controllers

PLUGINLIB_EXPORT_CLASS(dual_fr3_moveit_config::controllers::JointTargetController, controller_interface::ControllerInterface)
