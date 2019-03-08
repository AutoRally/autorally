find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen_INCLUDE_DIRS})

find_package(GTSAM)
if(GTSAM_FOUND)
  find_package(TBB)
  include_directories(include ${catkin_INCLUDE_DIRS} "/usr/local/include")

  add_executable(StateEstimator StateEstimator.cpp)
  target_link_libraries(StateEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so /usr/local/lib/libGeographic.so ${TBB_LIBRARIES} ar_diagnostics)
  add_dependencies(StateEstimator autorally_msgs_gencpp)

  install(TARGETS StateEstimator
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
