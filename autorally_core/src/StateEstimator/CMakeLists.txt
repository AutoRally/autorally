find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)
message("Hello World!!!!!!!!!!!!!!")

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

find_package(GTSAM)
if(GTSAM_FOUND)
  message("GTSAM FOUND!!!!!")
 # find_package(TBB)
  include_directories(include ${catkin_INCLUDE_DIRS} "/usr/local/include")

  add_executable(StateEstimator StateEstimator.cpp)
  target_link_libraries(StateEstimator ${catkin_LIBRARIES} ${ROS_LIBRARIES} /usr/local/lib/libgtsam.so /usr/local/lib/libGeographic.so  "gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}"
 Diagnostics)
  add_dependencies(StateEstimator autorally_msgs_gencpp)

  install(TARGETS StateEstimator
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
