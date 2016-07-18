add_library(autorally_chassis autorally_chassis.cpp)
add_dependencies(autorally_chassis autorally_msgs_gencpp)
target_link_libraries(autorally_chassis ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer SerialSensorInterface Diagnostics)

install(TARGETS
  autorally_chassis

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
