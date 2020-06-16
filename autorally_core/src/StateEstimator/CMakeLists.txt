find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package (Boost REQUIRED COMPONENTS system filesystem thread chrono date_time atomic serialization regex timer)
include_directories(SYSTEM ${Boost_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

find_package(GTSAM)
if(GTSAM_FOUND)
  find_package(TBB)
  include_directories(SYSTEM ${GTSAM_INCLUDE_DIR} "/usr/local/include" ${Boost_INCLUDE_DIRS}) 

  add_executable(StateEstimator StateEstimator.cpp)
message(==========${GeographicLib_LIBRARY_DIRS}/${GeographicLib_LIBRARIES})
  target_link_libraries(StateEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} gtsam ${GeographicLib_LIBRARY_DIRS}/libGeographic.so ${TBB_LIBRARIES} ar_diagnostics ${Boost_LIBRARIES})
  add_dependencies(StateEstimator autorally_msgs_gencpp)

  install(TARGETS StateEstimator
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
