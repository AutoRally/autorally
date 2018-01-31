cmake_minimum_required(VERSION 2.8.3)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

include_directories(include ${EIGEN_INCLUDE_DIRS})

################################################################################
#First make the helper libraries
################################################################################
add_library(autorally_plant autorally_plant.cpp)
add_dependencies(autorally_plant ${catkin_LIBRARIES})

install(TARGETS
  autorally_plant
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(param_getter param_getter.cpp)
add_dependencies(param_getter ${catkin_LIBRARIES})

install(TARGETS
  param_getter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
#Now build the CUDA executables (only if CUDA is installed)
################################################################################

find_package(CUDA)
if(CUDA_FOUND)

include(FindCUDA)

set(CUDA_PROPAGATE_HOST_FLAGS OFF)

set(
	CUDA_NVCC_FLAGS
	${CUDA_NVCC_FLAGS};
	-std=c++11
	-maxrregcount=32;
	-O3;
	-arch=sm_35;
	-w
)


####################################################################################
#Executable for the basis function model
####################################################################################

add_definitions(-DUSE_BASIS_FUNC_MODEL__)

cuda_add_executable(path_integral_bf
                    path_integral_main.cu
                    autorally_plant
                    param_getter
                    )

add_dependencies(path_integral_bf autorally_msgs_gencpp autorally_core ${PROJECT_NAME}_gencfg)

target_link_libraries(path_integral_bf /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

remove_definitions(-DUSE_BASIS_FUNC_MODEL__)

###################################################################################
#Executable for the neural network model
###################################################################################

add_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__)

cuda_add_executable(path_integral_nn
                   	path_integral_main.cu
                    autorally_plant
                    param_getter
                    )

add_dependencies(path_integral_nn autorally_msgs_gencpp autorally_core ${PROJECT_NAME}_gencfg)

target_link_libraries(path_integral_nn /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

remove_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__)

####################################################################################
#Executable for monitoring the status of mppi
####################################################################################

add_executable(path_integral_monitor
			   status_monitor.cpp)

add_dependencies(path_integral_monitor autorally_core autorally_msgs_gencpp)

target_link_libraries(path_integral_monitor ${catkin_LIBRARIES})

endif(CUDA_FOUND)
