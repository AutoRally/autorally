add_library(AutoRallyChassis AutoRallyChassis.cpp)
add_dependencies(AutoRallyChassis autorally_msgs_gencpp)
target_link_libraries(AutoRallyChassis ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer SerialSensorInterface Diagnostics)

install(TARGETS
  AutoRallyChassis

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
