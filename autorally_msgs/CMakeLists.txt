cmake_minimum_required(VERSION 2.8.3)
project(autorally_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

find_package(Boost REQUIRED thread)

add_message_files(
  DIRECTORY msg
  FILES
  chassisCommand.msg
  chassisState.msg
  wheelSpeeds.msg
  runstop.msg
  imageMask.msg
  line2D.msg
  point2D.msg
  pathIntegralParams.msg
  lapStats.msg
  pathIntegralStats.msg
  pathIntegralStatus.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

catkin_package()
