add_executable(chronyStatus chronyStatusMain.cpp ChronyStatus.cpp)
target_link_libraries(chronyStatus ${catkin_LIBRARIES} Diagnostics)
add_dependencies(chronyStatus autorally_msgs_gencpp)

install(TARGETS
  chronyStatus
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
