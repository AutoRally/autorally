cmake_minimum_required(VERSION 2.8.3)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

################################################################################
#First make the helper libraries
################################################################################
add_library(autorally_plant autorally_plant.cpp)
target_link_libraries(autorally_plant ${catkin_LIBRARIES})

add_library(param_getter param_getter.cpp)
target_link_libraries(param_getter ${catkin_LIBRARIES})

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

cuda_add_executable(path_integral_bf
                    path_integral_main.cu
                    autorally_plant
                    param_getter
                    )
target_compile_definitions(path_integral_bf PRIVATE PRIVATE USE_BASIS_FUNC_MODEL__)
add_dependencies(path_integral_bf autorally_msgs_gencpp autorally_core ${PROJECT_NAME}_gencfg)

target_link_libraries(path_integral_bf /home/klee698/.local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

###################################################################################
#Executable for the neural network model
###################################################################################

cuda_add_library(mppi_nodelet
                 mppi_nodelet.cu
                 autorally_plant
                 param_getter)
target_compile_definitions(mppi_nodelet PRIVATE
		USE_NEURAL_NETWORK_MODEL__;
		MPPI_NNET_USING_CONSTANT_MEM__)
add_dependencies(mppi_nodelet autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(mppi_nodelet /home/klee698/.local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cuda_add_executable(path_integral_nn
                   	path_integral_main.cu
                    autorally_plant
                    param_getter
                    )
target_compile_definitions(path_integral_nn PRIVATE
		USE_NEURAL_NETWORK_MODEL__;
		MPPI_NNET_USING_CONSTANT_MEM__)
add_dependencies(path_integral_nn autorally_msgs_gencpp autorally_core ${PROJECT_NAME}_gencfg)

target_link_libraries(path_integral_nn /home/klee698/.local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

####################################################################################
#Executable for monitoring the status of mppi
####################################################################################

add_executable(path_integral_monitor
			   status_monitor.cpp)
add_dependencies(path_integral_monitor autorally_msgs_gencpp)
target_link_libraries(path_integral_monitor ${catkin_LIBRARIES})

endif(CUDA_FOUND)
