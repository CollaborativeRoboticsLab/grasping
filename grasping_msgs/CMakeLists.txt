cmake_minimum_required(VERSION 3.8)
project(grasping_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

set(message_files
  "msg/SceneReference.msg"
  "msg/SceneDescriptor.msg"
  "msg/ActiveScene.msg"
)

set(action_files
  "action/MoveToPose.action"
  "action/MoveToNamedPose.action"
  "action/MoveToJointPose.action"
  "action/ActivateScene.action"
  "action/ActivateSceneByName.action"
  "action/LoadSceneFromContent.action"
)

set(service_files
  "srv/CheckCartesianPoseFeasibility.srv"
  "srv/CheckJointPoseFeasibility.srv"
  "srv/GetActiveScene.srv"
  "srv/ListAvailableScenes.srv"
  "srv/ValidateWorkspaceDocument.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  ${action_files}
  ${service_files}
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()