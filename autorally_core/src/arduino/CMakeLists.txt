add_library(ArduinoOnboard ArduinoOnboard.cpp)
target_link_libraries(ArduinoOnboard ${catkin_LIBRARIES} SerialSensorInterface Diagnostics)
add_dependencies(ArduinoOnboard autorally_msgs_gencpp)

install(TARGETS
  ArduinoOnboard
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
