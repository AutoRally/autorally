cmake_minimum_required(VERSION 3.10)
project(autorally_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_transport
  pluginlib
  nodelet
  autorally_msgs
  tf
  dynamic_reconfigure
  cv_bridge
)

find_package(cmake_modules REQUIRED)

find_package(catkin REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/camera_auto_balance_params.cfg
  cfg/camera_trigger_params.cfg
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs image_transport autorally_msgs
    INCLUDE_DIRS include
    LIBRARIES SerialSensorInterface ar_diagnostics RingBuffer
)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

#add_subdirectory(src/arduino)
add_subdirectory(src/Diagnostics)
add_subdirectory(src/gps)
add_subdirectory(src/ocs)
add_subdirectory(src/RingBuffer)
add_subdirectory(src/RunStop)
#add_subdirectory(src/SafeSpeed)
add_subdirectory(src/SerialSensorInterface)
#add_subdirectory(src/servoInterface)
add_subdirectory(src/xbee)
add_subdirectory(src/ImageRepublisher)
add_subdirectory(src/StateEstimator)
add_subdirectory(src/autorally_chassis)
add_subdirectory(src/camera_trigger)
add_subdirectory(src/CameraAutoBalance)
add_subdirectory(src/WheelOdometry)


install(PROGRAMS
  src/systemStatus/systemStatus.py
  src/chronyStatus/chronyStatus.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
