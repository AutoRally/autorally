add_library(ServoInterface servoInterface.cpp PololuMaestro.cpp)
add_dependencies(ServoInterface autorally_msgs_gencpp)
target_link_libraries(ServoInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer SafeSpeed SerialSensorInterface Diagnostics)

#add_executable(servoInterface servoInterfaceMain.cpp)
#target_link_libraries(servoInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES} ServoInterface SerialSensorInterface SafeSpeed Diagnostics RingBuffer)
#add_dependencies(servoInterface autorally_msgs_gencpp)

install(TARGETS
  ServoInterface
#  servoInterface

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
