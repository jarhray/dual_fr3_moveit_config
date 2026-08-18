cmake_minimum_required(VERSION 3.8)
project(dual_fr3_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fake_gripper_action_server.py
    scripts/fr3_controller.py
    scripts/fr3_controller_lin.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
