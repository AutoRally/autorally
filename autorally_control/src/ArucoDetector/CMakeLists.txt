add_executable(ArucoDetector ArucoDetector.cpp)
add_dependencies(ArucoDetector autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(ArucoDetector ${catkin_LIBRARIES})


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(ArucoDetector ${OpenCV_LIBRARIES})

find_package(cv_bridge)
include_directories(${cv_bridge_INCLUDE_DIRS})
target_link_libraries(ArucoDetector ${cv_bridge_LIBRARIES})

install(TARGETS
  ArucoDetector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


