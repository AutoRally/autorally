add_executable(runStop RunStop.cpp)
target_link_libraries(runStop ${catkin_LIBRARIES} ${Boost_LIBRARIES} SerialSensorInterface Diagnostics)
add_dependencies(runStop autorally_msgs_gencpp)

install(TARGETS
  runStop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
