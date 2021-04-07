add_executable(auto_label SingleShotPoseLabeler.cpp)
target_link_libraries(auto_label ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(auto_label autorally_msgs_gencpp)

set(CMAKE_BUILD_TYPE Debug)

install(TARGETS
        auto_label
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
