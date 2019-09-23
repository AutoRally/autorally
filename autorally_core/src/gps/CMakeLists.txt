add_executable(gpsHemisphereInterface GPSHemisphere.cpp)
target_link_libraries(gpsHemisphereInterface ${catkin_LIBRARIES} SerialSensorInterface ar_diagnostics)
add_dependencies(gpsHemisphereInterface autorally_msgs_gencpp)

#add_library(gps_LS20031 src/gps_LS20031/gps_LS20031.cpp)
#add_dependencies(gps_LS20031 autorally_msgs_gencpp)

#add_executable(gpsInterfaceLS src/gpsInterface/gpsInterfaceLS.cpp)
#target_link_libraries(gpsInterfaceLS ${catkin_LIBRARIES} gps_LS20031 SerialSensorInterface ar_diagnostics)

install(TARGETS
  gpsHemisphereInterface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
