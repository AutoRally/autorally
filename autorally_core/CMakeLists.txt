cmake_minimum_required(VERSION 2.8.3)
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
)

find_package(cmake_modules REQUIRED)

catkin_python_setup()

catkin_package(
    DEPENDS libqt4-dev lm-sensors Boost
    CATKIN-DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs image_transport qt-ros diagnostic_updater qt_build autorally_msgs
    INCLUDE_DIRS include
    LIBRARIES SerialSensorInterface SafeSpeed Diagnostics RingBuffer
)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(include)

add_subdirectory(src/arduino)
add_subdirectory(src/Diagnostics)
add_subdirectory(src/gps)
add_subdirectory(src/ocs)
add_subdirectory(src/RingBuffer)
add_subdirectory(src/RunStop)
add_subdirectory(src/SafeSpeed)
add_subdirectory(src/SerialSensorInterface)
add_subdirectory(src/servoInterface)
add_subdirectory(src/xbee)
add_subdirectory(src/ImageRepublisher)
add_subdirectory(src/StateEstimator)

install(TARGETS
 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  src/systemStatus/systemStatus.py
  src/ChronyStatus/chronyStatus.py
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
