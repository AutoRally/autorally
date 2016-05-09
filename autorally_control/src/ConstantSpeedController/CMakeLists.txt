add_library(ConstantSpeedController ConstantSpeedController.cpp)
add_dependencies(ConstantSpeedController autorally_msgs_gencpp)
target_link_libraries(ConstantSpeedController ${catkin_LIBRARIES} ${Boost_LIBRARIES} RingBuffer)

install(TARGETS
  ConstantSpeedController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
