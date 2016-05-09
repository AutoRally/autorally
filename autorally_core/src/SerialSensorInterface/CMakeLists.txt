add_library(SerialSensorInterface SerialSensorInterface.cpp SerialInterfaceThreaded.cpp SerialCommon.cpp)
target_link_libraries(SerialSensorInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(SerialSensorInterface autorally_msgs_gencpp)

install(TARGETS
  SerialSensorInterface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

