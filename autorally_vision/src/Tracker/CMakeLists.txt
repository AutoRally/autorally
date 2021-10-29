find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

find_package(GTSAM)
if(GTSAM_FOUND)
    find_package(TBB)
    include_directories(include ${catkin_INCLUDE_DIRS} "/usr/local/include")

    #add_executable(VehicleTracker Tracker.cpp)
    #target_link_libraries(VehicleTracker ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so /usr/local/lib/libGeographic.so ${TBB_LIBRARIES})
    #add_dependencies(VehicleTracker autorally_msgs_gencpp)

    #install(TARGETS VehicleTracker
    #        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
