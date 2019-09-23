add_library(ar_diagnostics Diagnostics.cpp)
add_dependencies(ar_diagnostics autorally_msgs_gencpp)

install(TARGETS
  ar_diagnostics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
