cmake_minimum_required(VERSION 3.8)
project(msg_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerformanceMetrics.msg"
  "msg/PerformanceMetricsArray.msg"
  "msg/ClusterBetweenRobotHeartBeat.msg"
  "msg/ClusterToManagerState.msg"
  "msg/ClusterToRobotTrajectory.msg"
  "msg/ClusterToRvizShortestPath.msg"
  "msg/GazeboToManagerState.msg"
  "msg/ManagerToClusterStart.msg"
  "msg/ManagerToClusterStateSet.msg"
  "msg/RobotToClusterState.msg"
  "msg/RobotToGazeboCmd.msg"
  "msg/RobotToRvizStatus.msg"
  "srv/RegisterRobot.srv"
  "srv/ExecuteCommand.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()