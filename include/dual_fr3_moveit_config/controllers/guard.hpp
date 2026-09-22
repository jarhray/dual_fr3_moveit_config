#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace dual_fr3_moveit_config::controllers {
using Joints = std::array<double, 7>;
enum class Phase { Holding, Tracking, Stopping, Stopped };
enum class Reason {
  None, Cancelled, Timeout, InvalidTarget, WrongSession, Sequence, Timestamp,
  JointLimit, Velocity, Acceleration, TrackingError, Load, StateInvalid,
  StateStale, Period, Torque, Clock
};
inline const char* phase_name(Phase value) {
  switch (value) {
    case Phase::Holding: return "HOLDING";
    case Phase::Tracking: return "TRACKING";
    case Phase::Stopping: return "STOPPING";
    case Phase::Stopped: return "STOPPED";
  }
  return "STOPPED";
}
inline const char* reason_name(Reason value) {
  switch (value) {
    case Reason::None: return "none";
    case Reason::Cancelled: return "cancelled";
    case Reason::Timeout: return "command_timeout";
    case Reason::InvalidTarget: return "invalid_target";
    case Reason::WrongSession: return "wrong_session";
    case Reason::Sequence: return "nonmonotonic_sequence";
    case Reason::Timestamp: return "invalid_timestamp";
    case Reason::JointLimit: return "joint_limit";
    case Reason::Velocity: return "velocity_limit";
    case Reason::Acceleration: return "acceleration_limit";
    case Reason::TrackingError: return "tracking_error";
    case Reason::Load: return "external_load_limit";
    case Reason::StateInvalid: return "invalid_robot_state";
    case Reason::StateStale: return "stale_robot_state";
    case Reason::Period: return "invalid_control_period";
    case Reason::Torque: return "torque_limit";
    case Reason::Clock: return "clock_discontinuity";
  }
  return "invalid_robot_state";
}
struct Config {
  Joints lower{}, upper{}, kp{}, kd{}, max_velocity{}, max_acceleration{};
  Joints max_torque{}, max_torque_rate{}, max_tracking_error{};
  double command_timeout{0.10}, state_timeout{0.01}, max_period{0.005};
  double future_tolerance{0.005}, max_force{0.0}, max_moment{0.0};
  double activation_velocity{0.01}, stopped_velocity{0.003}, settle_time{0.1};
  bool valid() const {
    const std::array<double, 10> values{command_timeout, state_timeout, max_period,
      future_tolerance, max_force, max_moment, activation_velocity, stopped_velocity,
      settle_time, 1.0};
    for (double x : values) if (!std::isfinite(x) || x <= 0.0) return false;
    if (max_period > command_timeout || state_timeout > command_timeout) return false;
    for (size_t i = 0; i < 7; ++i) {
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i])
        return false;
      for (double x : {kp[i], kd[i], max_velocity[i], max_acceleration[i],
          max_torque[i], max_torque_rate[i], max_tracking_error[i]})
        if (!std::isfinite(x) || x <= 0.0) return false;
    }
    return true;
  }
};
struct Target {
  uint64_t session{0}, sequence{0}, generation{0};
  double stamp{0.0}, received{0.0}, valid_for{0.0};
  Joints q{}, dq{};
};
struct Feedback {
  Joints q{}, dq{}, tau{};
  std::array<double, 6> wrench{};
  double robot_time{0.0};
};
inline bool finite(const Joints& q) {
  for (double x : q) if (!std::isfinite(x)) return false;
  return true;
}

// Allocation-free deterministic core. All time inputs are seconds; now is monotonic,
// ros_now is solely for checking message stamps, robot_time is libfranka device time.
class Guard {
 public:
  explicit Guard(const Config& config = Config{}) : config_(config) {}
  void configure(const Config& config) { config_ = config; }
  bool activate(uint64_t session, double now, double ros_now, const Feedback& fb) {
    if (!config_.valid() || session == 0 || !valid_feedback(fb) ||
        !std::isfinite(now) || !std::isfinite(ros_now)) return false;
    for (size_t i = 0; i < 7; ++i)
      if (fb.q[i] < config_.lower[i] || fb.q[i] > config_.upper[i] ||
          std::abs(fb.dq[i]) > config_.activation_velocity ||
          std::abs(fb.tau[i]) > config_.max_torque[i]) return false;
    if (load_exceeded(fb)) return false;
    session_ = session;
    sequence_ = 0;
    phase_ = Phase::Holding;
    reason_ = Reason::None;
    activated_ = last_received_ = last_update_ = robot_changed_ = now;
    last_ros_ = ros_now;
    last_robot_ = fb.robot_time;
    ref_ = goal_ = fb.q;
    ref_velocity_.fill(0.0);
    goal_velocity_.fill(0.0);
    previous_ = Target{};
    last_feedback_ = fb;
    torque_ = fb.tau;
    valid_for_ = config_.command_timeout;
    settle_elapsed_ = 0.0;
    return true;
  }
  void stop(Reason reason, const Feedback& fb) {
    if (latched()) return;
    phase_ = Phase::Stopping;
    reason_ = reason;
    // Freeze at the measured position, discard target lead. The internal reference
    // returns toward this point with bounded velocity/acceleration; effort is slew limited.
    goal_ = finite(fb.q) ? fb.q : last_feedback_.q;
    goal_velocity_.fill(0.0);
  }
  Reason basic_target(const Target& target, double now, double ros_now) const {
    if (!std::isfinite(target.stamp) || !std::isfinite(target.received) ||
        !std::isfinite(target.valid_for) || !finite(target.q) || !finite(target.dq) ||
        target.valid_for <= 0.0 || target.valid_for > config_.command_timeout ||
        target.stamp <= 0.0) return Reason::InvalidTarget;
    if (target.session != session_) return Reason::WrongSession;
    if (target.sequence <= sequence_) return Reason::Sequence;
    if (target.stamp > ros_now + config_.future_tolerance ||
        ros_now - target.stamp > target.valid_for || target.received < activated_ ||
        now < target.received || now - target.received > target.valid_for)
      return Reason::Timestamp;
    for (size_t i = 0; i < 7; ++i) {
      if (target.q[i] < config_.lower[i] || target.q[i] > config_.upper[i])
        return Reason::JointLimit;
      if (std::abs(target.dq[i]) > config_.max_velocity[i]) return Reason::Velocity;
    }
    return Reason::None;
  }
  bool accept(const Target& target, double now, double ros_now, const Feedback& fb) {
    if (latched()) return false;
    if (now - last_received_ > std::min(valid_for_, config_.command_timeout)) {
      stop(Reason::Timeout, fb);
      return false;
    }
    Reason error = basic_target(target, now, ros_now);
    if (error == Reason::None) {
      const double dt = sequence_ ? target.stamp - previous_.stamp : target.stamp - last_ros_;
      if (sequence_ && (!(dt > 0.0) || dt > config_.command_timeout)) {
        error = Reason::Timestamp;
      } else {
        for (size_t i = 0; i < 7; ++i) {
          if (std::abs(target.q[i] - fb.q[i]) > config_.max_tracking_error[i])
            error = Reason::TrackingError;
          // The first target must be a stationary handshake near the measured pose.
          if (!sequence_ && (std::abs(target.q[i] - fb.q[i]) >
                config_.max_velocity[i] * config_.max_period ||
              std::abs(target.dq[i]) > config_.activation_velocity))
            error = Reason::TrackingError;
          if (sequence_ && std::abs(target.q[i] - previous_.q[i]) >
                config_.max_velocity[i] * dt + 1e-10) error = Reason::Velocity;
          if (sequence_ && std::abs(target.dq[i] - previous_.dq[i]) >
                config_.max_acceleration[i] * dt + 1e-10) error = Reason::Acceleration;
        }
      }
    }
    if (error != Reason::None) { stop(error, fb); return false; }
    goal_ = target.q;
    goal_velocity_ = target.dq;
    previous_ = target;
    sequence_ = target.sequence;
    last_received_ = target.received;
    valid_for_ = target.valid_for;
    phase_ = Phase::Tracking;
    return true;
  }
  const Joints& update(double now, double ros_now, double dt, const Feedback& feedback) {
    const bool feedback_ok = valid_feedback(feedback);
    if (!feedback_ok) stop(Reason::StateInvalid, last_feedback_);
    const Feedback& fb = feedback_ok ? feedback : last_feedback_;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > config_.max_period ||
        !std::isfinite(now) || now <= last_update_) {
      stop(Reason::Period, fb);
      // Do not integrate across a scheduling gap. Keep the last finite command.
      last_update_ = now;
      return torque_;
    }
    if (!std::isfinite(ros_now) || ros_now < last_ros_ ||
        std::abs((ros_now - last_ros_) - (now - last_update_)) > config_.command_timeout)
      stop(Reason::Clock, fb);
    last_ros_ = ros_now;
    last_update_ = now;
    bool state_fresh = feedback_ok;
    if (feedback_ok) {
      if (fb.robot_time > last_robot_) robot_changed_ = now;
      if (fb.robot_time < last_robot_ || now - robot_changed_ > config_.state_timeout) {
        state_fresh = false;
        stop(Reason::StateStale, fb);
      }
      last_robot_ = fb.robot_time;
      last_feedback_ = fb;
      if (load_exceeded(fb)) stop(Reason::Load, fb);
      for (size_t i = 0; i < 7; ++i) {
        if (fb.q[i] < config_.lower[i] || fb.q[i] > config_.upper[i])
          stop(Reason::JointLimit, fb);
        if (std::abs(fb.dq[i]) > config_.max_velocity[i]) stop(Reason::Velocity, fb);
        if (std::abs(ref_[i] - fb.q[i]) > config_.max_tracking_error[i])
          stop(Reason::TrackingError, fb);
        if (std::abs(fb.tau[i]) > config_.max_torque[i]) stop(Reason::Torque, fb);
      }
    }
    if (now - last_received_ > std::min(valid_for_, config_.command_timeout) ||
        (sequence_ && ros_now - previous_.stamp > valid_for_)) stop(Reason::Timeout, fb);
    // No trusted state means no reference integration and no assertion of a completed
    // stop. The hardware's own FCI watchdog/reflex remains necessary in this case.
    if (!state_fresh) return torque_;
    bool settled = true;
    for (size_t i = 0; i < 7; ++i) {
      const double error = goal_[i] - ref_[i];
      const double braking_speed = std::sqrt(2.0 * config_.max_acceleration[i] * std::abs(error));
      // Position feedback plus feedforward; braking bound matters for stop/hold.
      double desired = std::clamp(error * 20.0 + goal_velocity_[i],
        -config_.max_velocity[i], config_.max_velocity[i]);
      if (latched() || phase_ == Phase::Holding)
        desired = std::clamp(desired, -braking_speed, braking_speed);
      const double dv = config_.max_acceleration[i] * dt;
      ref_velocity_[i] += std::clamp(desired - ref_velocity_[i], -dv, dv);
      ref_[i] = std::clamp(ref_[i] + dt * ref_velocity_[i], config_.lower[i], config_.upper[i]);
      const double requested = std::clamp(config_.kp[i] * (ref_[i] - fb.q[i]) +
        config_.kd[i] * (ref_velocity_[i] - fb.dq[i]), -config_.max_torque[i], config_.max_torque[i]);
      // Start from the actual previous robot command, not an assumed zero torque.
      const double origin = feedback_ok ? fb.tau[i] : torque_[i];
      const double step = config_.max_torque_rate[i] * dt;
      torque_[i] = origin + std::clamp(requested - origin, -step, step);
      settled = settled && std::abs(fb.dq[i]) <= config_.stopped_velocity &&
        std::abs(ref_velocity_[i]) <= config_.stopped_velocity &&
        std::abs(ref_[i] - goal_[i]) <= 1e-4;
    }
    if (phase_ == Phase::Stopping) {
      settle_elapsed_ = settled ? settle_elapsed_ + dt : 0.0;
      if (settle_elapsed_ >= config_.settle_time) phase_ = Phase::Stopped;
    } else if (phase_ == Phase::Stopped && !settled) {
      phase_ = Phase::Stopping;
      settle_elapsed_ = 0.0;
    }
    return torque_;
  }
  bool latched() const { return phase_ == Phase::Stopping || phase_ == Phase::Stopped; }
  Phase phase() const { return phase_; }
  Reason reason() const { return reason_; }
  uint64_t session() const { return session_; }
  uint64_t sequence() const { return sequence_; }
  const Joints& reference() const { return ref_; }
  const Joints& reference_velocity() const { return ref_velocity_; }
 private:
  static bool valid_feedback(const Feedback& fb) {
    if (!finite(fb.q) || !finite(fb.dq) || !finite(fb.tau) ||
        !std::isfinite(fb.robot_time) || fb.robot_time < 0.0) return false;
    for (double x : fb.wrench) if (!std::isfinite(x)) return false;
    return true;
  }
  bool load_exceeded(const Feedback& fb) const {
    return std::hypot(fb.wrench[0], fb.wrench[1], fb.wrench[2]) > config_.max_force ||
      std::hypot(fb.wrench[3], fb.wrench[4], fb.wrench[5]) > config_.max_moment;
  }
  Config config_;
  Phase phase_{Phase::Stopped};
  Reason reason_{Reason::None};
  uint64_t session_{0}, sequence_{0};
  double activated_{0}, last_received_{0}, last_update_{0}, last_ros_{0};
  double last_robot_{0}, robot_changed_{0}, valid_for_{0}, settle_elapsed_{0};
  Target previous_{};
  Feedback last_feedback_{};
  Joints ref_{}, ref_velocity_{}, goal_{}, goal_velocity_{}, torque_{};
};
}  // namespace dual_fr3_moveit_config::controllers
