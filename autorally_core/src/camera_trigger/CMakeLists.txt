add_library(CameraTrigger CameraTrigger.cpp)
target_link_libraries(CameraTrigger ${catkin_LIBRARIES} SerialSensorInterface Diagnostics)
add_dependencies(CameraTrigger autorally_msgs_gencpp  ${PROJECT_NAME}_gencfg)

install(TARGETS
  CameraTrigger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
