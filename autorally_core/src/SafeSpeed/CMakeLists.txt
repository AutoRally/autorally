add_library(SafeSpeed SafeSpeed.cpp)
add_dependencies(SafeSpeed autorally_msgs_gencpp)
target_link_libraries(SafeSpeed ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer)

install(TARGETS
  SafeSpeed
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
