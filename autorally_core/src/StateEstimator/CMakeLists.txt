find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

if(GTSAM_FOUND)
  find_package(TBB)
  include_directories(include ${catkin_INCLUDE_DIRS} "/usr/local/include")
  include_directories(${GTSAM_INCLUDE_DIR})

  add_executable(StateEstimator StateEstimator.cpp)
  target_link_libraries(StateEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} gtsam ${GeographicLib_LIBRARIES} ${TBB_LIBRARIES} Diagnostics)
  add_dependencies(StateEstimator autorally_msgs_gencpp)

  install(TARGETS StateEstimator
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
