add_library(Diagnostics Diagnostics.cpp)
add_dependencies(Diagnostics autorally_msgs_gencpp)

install(TARGETS
  Diagnostics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
