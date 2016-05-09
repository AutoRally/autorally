add_executable(joystickController
               joystickControlMain.cpp
               JoystickControl.cpp)
add_dependencies(joystickController autorally_msgs_gencpp)
target_link_libraries(joystickController ${catkin_LIBRARIES})

install(TARGETS
  joystickController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
