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
  safeSpeed.msg
  wheelSpeeds.msg
  imageMask.msg
  servoMSG.msg
  line2D.msg
  point2D.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

catkin_package()
