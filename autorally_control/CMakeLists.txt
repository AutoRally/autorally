cmake_minimum_required(VERSION 2.8.3)
project(autorally_control)

find_package(catkin REQUIRED COMPONENTS
  autorally_msgs
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  autorally_core
  nav_msgs
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost 1.49.0 REQUIRED)
find_package(Eigen3 REQUIRED)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

generate_dynamic_reconfigure_options(cfg/gpsWaypoint_params.cfg
				     cfg/PathIntegralParams.cfg)

catkin_package(
  DEPENDS Boost
  CATKIN-DEPENDS roscpp std_msgs geometry_msgs nav_msgs nodelet dynamic_reconfigure autorally_core autorally_msgs
  INCLUDE_DIRS include
  LIBRARIES autorally_plant param_getter

)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS})

add_subdirectory(src/ConstantSpeedController)
add_subdirectory(src/joystick)
add_subdirectory(src/gpsWaypoint)
add_subdirectory(src/path_integral)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY cfg/cpp/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(FILES nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
