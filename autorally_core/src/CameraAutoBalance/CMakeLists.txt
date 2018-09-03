
find_library(FLYCAP_LIB flycapture)

if(FLYCAP_LIB)
  add_library(CameraAutoBalance CameraAutoBalance.cpp)
  add_dependencies(CameraAutoBalance autorally_msgs_gencpp)
  set_target_properties(CameraAutoBalance PROPERTIES COMPILE_FLAGS "-march=native -O3 -ffast-math")
  target_link_libraries(CameraAutoBalance ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${cv_bridge_LIBRARIES} ${FLYCAP_LIB})

install(TARGETS
  CameraAutoBalance
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else()
  message(WARNING "Could not find FlyCapture library. Will not build CameraAutoBalance")
endif()
