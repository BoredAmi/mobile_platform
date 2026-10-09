cmake_minimum_required(VERSION 3.8)
project(jgb_rover_hardware LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# The wire protocol is shared with the firmware and lives next to it (repository root, like
# robot_spec.yaml), so both sides always build from the same header.
set(PROTOCOL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/protocol)
if(NOT EXISTS ${PROTOCOL_DIR}/rover_link.h)
  message(FATAL_ERROR "firmware/protocol/rover_link.h not found at ${PROTOCOL_DIR}")
endif()

# ROS-independent link layer, also used by the fake-board test
add_library(jgb_rover_mcu_link STATIC
  src/mcu_link.cpp
  src/serial_port.cpp
  ${PROTOCOL_DIR}/rover_link.c)
target_include_directories(jgb_rover_mcu_link PUBLIC include ${PROTOCOL_DIR})
set_target_properties(jgb_rover_mcu_link PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(jgb_rover_mcu_link PUBLIC cxx_std_17)

add_library(jgb_rover_hardware SHARED src/jgb_rover_system.cpp)
target_link_libraries(jgb_rover_hardware jgb_rover_mcu_link)
ament_target_dependencies(jgb_rover_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface jgb_rover_hardware.xml)

# command-line probe of the link layer (used by test/test_mcu_link.py, handy on the robot too)
add_executable(mcu_link_probe test/mcu_link_probe.cpp)
target_link_libraries(mcu_link_probe jgb_rover_mcu_link)

install(TARGETS jgb_rover_hardware LIBRARY DESTINATION lib)
install(TARGETS mcu_link_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_mcu_link test/test_mcu_link.py
    ENV MCU_LINK_PROBE=$<TARGET_FILE:mcu_link_probe>)
endif()

ament_package()
