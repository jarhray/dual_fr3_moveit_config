#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <franka/robot_state.h>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "dual_fr3_moveit_config/controllers/guard.hpp"
#include "dual_fr3_moveit_config/msg/joint_target.hpp"
#include "dual_fr3_moveit_config/msg/joint_target_state.hpp"

namespace dual_fr3_moveit_config::controllers {
class JointTargetController : public controller_interface::ControllerInterface {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
 private:
  static double monotonic_now();
  Feedback feedback() const;
  void target_callback(const msg::JointTarget&);
  void publish_state(const rclcpp::Time&, const Feedback&);
  std::vector<std::string> joints_;
  std::string robot_state_interface_;
  size_t robot_state_index_{0};
  std::array<size_t, 7> command_index_{};
  Config config_;
  Guard guard_;
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> session_{0}, generation_{0};
  std::atomic<Reason> requested_stop_{Reason::None};
  Target callback_previous_{};  // only the default mutually-exclusive callback group accesses it
  uint64_t last_generation_{0};
  double publish_period_{0.02}, last_publish_{0.0};
  realtime_tools::RealtimeBuffer<Target> target_buffer_;
  rclcpp::Subscription<msg::JointTarget>::SharedPtr target_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Publisher<msg::JointTargetState>::SharedPtr state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<msg::JointTargetState>> state_rt_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};
}  // namespace dual_fr3_moveit_config::controllers
