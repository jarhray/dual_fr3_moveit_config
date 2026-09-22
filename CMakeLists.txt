cmake_minimum_required(VERSION 3.8)
project(dual_fr3_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# rosidl installs the generated Python package. Install the existing helpers
# into that same package instead of registering a second ament Python target.
include(cmake/joint_target_controller.cmake)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(
  DIRECTORY config docs launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fake_gripper_action_server.py
    scripts/fr3_controller.py
    scripts/fr3_controller_lin.py
    scripts/check_usb_cable.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_maniskill_scene_launch test/test_maniskill_scene_launch.py)
  ament_add_pytest_test(test_insertion_controller_interfaces test/test_insertion_controller_interfaces.py)
endif()

ament_package()
