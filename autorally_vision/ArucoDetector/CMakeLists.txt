add_executable(aruco_detector ArucoDetector.cpp)
message(++++++++++++++++${OpenCV_LIBRARIES}})
target_link_libraries(aruco_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} opencv_aruco)
