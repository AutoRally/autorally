cmake_minimum_required(VERSION 2.8.3)
project(autorally_vision)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        autorally_core
        autorally_msgs
        image_transport
        roslib
        cv_bridge
        nodelet
        tf
        dynamic_reconfigure
        rosbag_storage)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread serialization filesystem system program_options)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

catkin_package(
        DEPENDS
        CATKIN_DEPENDS autorally_core image_transport roscpp roslib std_msgs autorally_msgs
        cv_bridge nodelet sensor_msgs tf visualization_msgs dynamic_reconfigure geometry_msgs
        INCLUDE_DIRS include
        LIBRARIES
)

add_subdirectory(src)

# Mark cpp header files for installation
install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(TARGETS
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.launch"
        )

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY cfg/cpp/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
