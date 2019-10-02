add_executable(xbeeCoordinator XbeeCoordinator.cpp XbeeInterface.cpp)
target_link_libraries(xbeeCoordinator ${catkin_LIBRARIES} ${Boost_LIBRARIES} SerialSensorInterface ar_diagnostics)
add_executable(xbeeNode XbeeNode.cpp XbeeInterface.cpp)
target_link_libraries(xbeeNode ${catkin_LIBRARIES} ${Boost_LIBRARIES} SerialSensorInterface ar_diagnostics)

install(TARGETS
  xbeeCoordinator
  xbeeNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

