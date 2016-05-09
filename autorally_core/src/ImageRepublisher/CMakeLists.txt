add_library(ImageRepublisher ImageRepublisher.cpp)
target_link_libraries(ImageRepublisher ${catkin_LIBRARIES})
add_dependencies(ImageRepublisher autorally_msgs_gencpp)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(ImageRepublisher ${OpenCV_LIBRARIES})

find_package(cv_bridge)
include_directories(${cv_bridge_INCLUDE_DIRS})
target_link_libraries(ImageRepublisher ${cv_bridge_LIBRARIES})

install(TARGETS
  ImageRepublisher

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
