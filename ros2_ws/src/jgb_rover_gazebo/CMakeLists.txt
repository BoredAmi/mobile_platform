cmake_minimum_required(VERSION 3.8)
project(jgb_rover_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gen_world.py scripts/check_spawn.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
