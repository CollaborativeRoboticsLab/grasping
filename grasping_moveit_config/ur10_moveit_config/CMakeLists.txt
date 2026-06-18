cmake_minimum_required(VERSION 3.22)
project(ur10_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
