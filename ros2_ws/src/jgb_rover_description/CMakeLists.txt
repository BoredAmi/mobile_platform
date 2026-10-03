cmake_minimum_required(VERSION 3.8)
project(jgb_rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME})

# robot_spec.yaml is the single source of truth and lives in the repository root
# (next to ros2_ws/). Install it so xacro can find it via $(find jgb_rover_description).
set(ROBOT_SPEC ${CMAKE_CURRENT_SOURCE_DIR}/../../../robot_spec.yaml)
if(NOT EXISTS ${ROBOT_SPEC})
  message(FATAL_ERROR "robot_spec.yaml not found at ${ROBOT_SPEC}")
endif()
install(FILES ${ROBOT_SPEC} DESTINATION share/${PROJECT_NAME}/config)

install(PROGRAMS scripts/check_alignment.py DESTINATION lib/${PROJECT_NAME})

ament_package()
