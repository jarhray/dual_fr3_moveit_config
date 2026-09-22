#include <gtest/gtest.h>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>

TEST(Plugin, InstantiatesWithoutRobotOrRosNode) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  auto controller = loader.createUniqueInstance("dual_fr3_moveit_config/JointTargetController");
  ASSERT_NE(controller.get(), nullptr);
}
