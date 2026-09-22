# The robot environment owns low-level control interfaces; task sequencing
# remains in dual_fr3_trunking_mtc/insertion_task.
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Franka REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTarget.msg" "msg/JointTargetState.msg"
  DEPENDENCIES builtin_interfaces)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(dual_fr3_joint_target_controller SHARED src/controllers/joint_target_controller.cpp)
target_compile_features(dual_fr3_joint_target_controller PUBLIC cxx_std_17)
target_compile_options(dual_fr3_joint_target_controller PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(dual_fr3_joint_target_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dual_fr3_joint_target_controller controller_interface hardware_interface
  pluginlib rclcpp rclcpp_lifecycle realtime_tools std_srvs Franka)
target_link_libraries(dual_fr3_joint_target_controller "${cpp_typesupport_target}")
pluginlib_export_plugin_description_file(controller_interface controllers.xml)
install(TARGETS dual_fr3_joint_target_controller EXPORT export_joint_target_controller
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_target_guard test/test_joint_target_guard.cpp)
  target_compile_features(test_joint_target_guard PRIVATE cxx_std_17)
  target_include_directories(test_joint_target_guard PRIVATE include)
  ament_add_gtest(test_joint_target_plugin test/test_joint_target_plugin.cpp)
  ament_target_dependencies(test_joint_target_plugin controller_interface pluginlib)
endif()
ament_export_targets(export_joint_target_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface hardware_interface pluginlib rclcpp
  rclcpp_lifecycle realtime_tools std_srvs Franka rosidl_default_runtime)
