#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)

add_executable(gpsWaypoint gpsWaypoint.cpp)
add_dependencies(gpsWaypoint autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(gpsWaypoint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_install_python(PROGRAMS GenerateWaypoints
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS
  gpsWaypoint
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
