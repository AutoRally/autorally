add_executable(WheelOdometry wheel_odometry.cpp)
target_link_libraries(WheelOdometry ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(WheelOdometry autorally_msgs_gencpp)

install(TARGETS
    WheelOdometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
