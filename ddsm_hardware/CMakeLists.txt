cmake_minimum_required(VERSION 3.8)
project(ddsm_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# RS485 frame encoding, no ROS dependencies
add_library(ddsm_protocol STATIC src/ddsm_protocol.cpp)
target_include_directories(ddsm_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(ddsm_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ros2_control hardware plugin
add_library(${PROJECT_NAME} SHARED src/ddsm_hardware_interface.cpp)
target_link_libraries(${PROJECT_NAME} ddsm_protocol)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface ddsm_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME} ddsm_protocol DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ddsm_protocol test/test_ddsm_protocol.cpp)
  target_link_libraries(test_ddsm_protocol ddsm_protocol)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
