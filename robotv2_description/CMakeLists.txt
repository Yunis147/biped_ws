cmake_minimum_required(VERSION 3.8)
project(robotv2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config rviz worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
