cmake_minimum_required(VERSION 3.8)
project(fleet_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFormation.srv" "srv/SetRegions.srv" "srv/FleetCommand.srv"
  "msg/PeerState.msg" "msg/MapSnapshot.msg" "msg/RelativeConstraint.msg"
  "msg/MissionCommand.msg" "msg/CoordinationEvent.msg" "msg/MappingState.msg"
  "msg/LocalAuthorization.msg" "msg/MapContinuity.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
