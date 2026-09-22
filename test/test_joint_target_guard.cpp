#include <gtest/gtest.h>
#include <limits>
#include "dual_fr3_moveit_config/controllers/guard.hpp"

using namespace dual_fr3_moveit_config::controllers;
namespace {
Config config() {
  Config c;
  c.lower.fill(-2.0); c.upper.fill(2.0); c.kp.fill(100.0); c.kd.fill(10.0);
  c.max_velocity.fill(0.1); c.max_acceleration.fill(0.5);
  c.max_torque.fill(20.0); c.max_torque_rate.fill(10.0);
  c.max_tracking_error.fill(0.02); c.max_force = 8.0; c.max_moment = 1.0;
  return c;
}
struct Fixture : testing::Test {
  Config c = config();
  Guard guard{c};
  Feedback fb;
  double now = 10.0, ros = 1000.0;
  void SetUp() override { ASSERT_TRUE(guard.activate(42, now, ros, fb)); }
  Target target(uint64_t sequence = 1) {
    Target t; t.session = 42; t.sequence = sequence;
    t.stamp = ros; t.received = now; t.valid_for = 0.08; t.q = fb.q; return t;
  }
  Joints tick(double step = 0.001) {
    now += step; ros += step; fb.robot_time += step;
    const auto tau = guard.update(now, ros, step, fb); fb.tau = tau; return tau;
  }
};
TEST_F(Fixture, StartupGraceAndFirstMeasuredHandshake) {
  for (int i = 0; i < 50; ++i) tick();
  EXPECT_EQ(guard.phase(), Phase::Holding);
  EXPECT_TRUE(guard.accept(target(), now, ros, fb));
  EXPECT_EQ(guard.phase(), Phase::Tracking);
}
TEST_F(Fixture, FirstTargetCannotJump) {
  auto t = target(); t.q[0] = 0.001;
  EXPECT_FALSE(guard.accept(t, now, ros, fb));
  EXPECT_EQ(guard.reason(), Reason::TrackingError);
}
TEST_F(Fixture, TimeoutLatchesAndLateTargetsCannotRevive) {
  EXPECT_TRUE(guard.accept(target(), now, ros, fb));
  for (int i = 0; i < 81; ++i) tick();
  EXPECT_EQ(guard.reason(), Reason::Timeout);
  EXPECT_FALSE(guard.accept(target(2), now, ros, fb));
}
TEST_F(Fixture, TimeoutCheckedBeforeAcceptingFreshTarget) {
  EXPECT_TRUE(guard.accept(target(), now, ros, fb));
  now += 0.081; ros += 0.081;
  EXPECT_FALSE(guard.accept(target(2), now, ros, fb));
  EXPECT_EQ(guard.reason(), Reason::Timeout);
}
TEST_F(Fixture, RejectsWrongSessionReplayAndInvalidValues) {
  auto t = target(); t.session = 41;
  EXPECT_EQ(guard.basic_target(t, now, ros), Reason::WrongSession);
  t.session = 42; t.q[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(guard.basic_target(t, now, ros), Reason::InvalidTarget);
  t = target(); t.valid_for = 5.0;
  EXPECT_EQ(guard.basic_target(t, now, ros), Reason::InvalidTarget);
  t = target(); t.stamp -= 0.2;
  EXPECT_EQ(guard.basic_target(t, now, ros), Reason::Timestamp);
  t = target(); t.stamp += 0.2;
  EXPECT_EQ(guard.basic_target(t, now, ros), Reason::Timestamp);
  EXPECT_TRUE(guard.accept(target(), now, ros, fb));
  EXPECT_EQ(guard.basic_target(target(), now, ros), Reason::Sequence);
}
TEST_F(Fixture, RejectsJointBoundsAndVelocityAndAcceleration) {
  auto t = target(); t.q[1] = 3.0;
  EXPECT_EQ(guard.basic_target(t, now, ros), Reason::JointLimit);
  t = target(); t.dq[1] = 0.2;
  EXPECT_EQ(guard.basic_target(t, now, ros), Reason::Velocity);
  EXPECT_TRUE(guard.accept(target(), now, ros, fb));
  for (int i = 0; i < 20; ++i) tick();
  t = target(2); t.dq[1] = 0.02;
  EXPECT_FALSE(guard.accept(t, now, ros, fb));
  EXPECT_EQ(guard.reason(), Reason::Acceleration);
}
TEST_F(Fixture, LoadsUseRawWrenchAndStopIsLatched) {
  fb.wrench[2] = 9.0; tick();
  EXPECT_EQ(guard.reason(), Reason::Load);
  fb.wrench.fill(0.0);
  EXPECT_FALSE(guard.accept(target(), now, ros, fb));
}
TEST_F(Fixture, StaleDeviceClockCannotReportStopped) {
  const double initial_robot_time = fb.robot_time;
  for (int i = 0; i < 200; ++i) {
    now += 0.001; ros += 0.001;
    guard.update(now, ros, 0.001, fb);
  }
  EXPECT_EQ(fb.robot_time, initial_robot_time);
  EXPECT_EQ(guard.reason(), Reason::StateStale);
  EXPECT_EQ(guard.phase(), Phase::Stopping);
}
TEST_F(Fixture, TorqueContinuityAndCancelMaintainHold) {
  fb.tau.fill(1.0);
  ASSERT_TRUE(guard.activate(42, now, ros, fb));
  auto out = tick();
  for (double tau : out) EXPECT_NEAR(tau, 0.99, 1e-12);
  ASSERT_TRUE(guard.accept(target(), now, ros, fb));
  for (int i = 0; i < 20; ++i) tick();
  auto t = target(2); t.q[0] = 0.001; t.dq[0] = 0.005;
  ASSERT_TRUE(guard.accept(t, now, ros, fb));
  tick();
  const auto before = guard.reference_velocity();
  guard.stop(Reason::Cancelled, fb);
  const auto tau_before = fb.tau;
  out = tick();
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_LE(std::abs(out[i] - tau_before[i]), c.max_torque_rate[i] * 0.001 + 1e-12);
    EXPECT_LE(std::abs(guard.reference_velocity()[i] - before[i]), c.max_acceleration[i] * 0.001 + 1e-12);
  }
  for (int i = 0; i < 500; ++i) tick();
  EXPECT_EQ(guard.phase(), Phase::Stopped);
  EXPECT_EQ(guard.reason(), Reason::Cancelled);
  EXPECT_FALSE(guard.accept(target(3), now, ros, fb));
}
TEST_F(Fixture, ReactivationUsesNewSessionAndMeasuredPose) {
  guard.stop(Reason::Cancelled, fb);
  fb.q[0] = 0.5; fb.tau.fill(0.0);
  ASSERT_TRUE(guard.activate(43, now, ros, fb));
  EXPECT_DOUBLE_EQ(guard.reference()[0], 0.5);
  EXPECT_EQ(guard.basic_target(target(), now, ros), Reason::WrongSession);
}
TEST_F(Fixture, LongControlPeriodDoesNotIntegrateAndLatches) {
  const auto before = guard.reference();
  tick(0.02);
  EXPECT_EQ(guard.reason(), Reason::Period);
  EXPECT_EQ(guard.reference(), before);
}
TEST_F(Fixture, RosClockReversalStops) {
  ros -= 1.0; tick();
  EXPECT_EQ(guard.reason(), Reason::Clock);
}
TEST_F(Fixture, InvalidFeedbackPreservesFiniteCommand) {
  fb.q[0] = std::numeric_limits<double>::quiet_NaN();
  const auto tau = tick();
  EXPECT_TRUE(finite(tau));
  EXPECT_EQ(guard.reason(), Reason::StateInvalid);
  EXPECT_EQ(guard.phase(), Phase::Stopping);
}
TEST(GuardConfiguration, RejectsUnverifiedDefaultLimitsAndNonfiniteThresholds) {
  EXPECT_FALSE(Config{}.valid());
  auto c = config(); c.max_force = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(c.valid());
}
}  // namespace
