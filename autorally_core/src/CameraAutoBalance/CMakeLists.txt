add_library(CameraAutoBalance CameraAutoBalance.cpp)
add_dependencies(CameraAutoBalance autorally_msgs_gencpp)
set_target_properties(CameraAutoBalance PROPERTIES COMPILE_FLAGS "-march=native -O3 -ffast-math")
target_link_libraries(CameraAutoBalance ${catkin_LIBRARIES})


find_library(FLYCAP_LIB flycapture)
if(FLYCAP_LIB)
  add_library(FlycapAdjuster FlycaptureAdjuster.cpp)
  target_link_libraries(FlycapAdjuster ${FLYCAP_LIB})

  add_library(CameraAutoBalancePtGrey CameraAutoBalancePtGrey.cpp)
  add_dependencies(CameraAutoBalancePtGrey FlycapAdjuster)
  target_link_libraries(CameraAutoBalancePtGrey CameraAutoBalance FlycapAdjuster)

  install(TARGETS
    CameraAutoBalancePtGrey
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(WARNING "Could not find FlyCapture library. Will not build CameraAutoBalance for Point Grey cameras")
endif()


find_library(Spinnaker_LIB Spinnaker)
if(Spinnaker_LIB)
  include_directories(/usr/include/spinnaker /usr/local/include/spinnaker)
  add_library(SpinnakerAdjuster SpinnakerAdjuster.cpp)
  target_link_libraries(SpinnakerAdjuster ${Spinnaker_LIB})

  add_library(CameraAutoBalanceFLIR CameraAutoBalanceFLIR.cpp)
  add_dependencies(CameraAutoBalanceFLIR SpinnakerAdjuster)
  target_link_libraries(CameraAutoBalanceFLIR CameraAutoBalance SpinnakerAdjuster)

  install(TARGETS
    CameraAutoBalanceFLIR
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(WARNING "Could not find Spinnaker library. Will not build CameraAutoBalance for FLIR cameras")
endif()
