cmake_minimum_required(VERSION 2.8.3)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include ${EIGEN_INCLUDE_DIRS})

################################################################################
#First make the helper libraries
################################################################################
add_library(autorally_plant autorally_plant.cpp)
target_link_libraries(autorally_plant ${catkin_LIBRARIES})

add_library(autorally_pc_plant autorally_pc_plant.cpp autorally_plant.cpp)
target_link_libraries(autorally_pc_plant ${catkin_LIBRARIES})

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

add_definitions(-DUSE_BASIS_FUNC_MODEL__)

cuda_add_executable(path_integral_bf
                    path_integral_main.cu
                    autorally_pc_plant
										autorally_plant
                    param_getter
                    )
add_dependencies(path_integral_bf autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(path_integral_bf /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

remove_definitions(-DUSE_BASIS_FUNC_MODEL__)

###################################################################################
#Executable for the neural network model with point cloud
###################################################################################

add_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_POINT_CLOUD__)

cuda_add_library(mppi_nodelet
                 mppi_nodelet.cu
                 autorally_pc_plant
								 autorally_plant
                 param_getter)
add_dependencies(mppi_nodelet autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(mppi_nodelet /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cuda_add_executable(path_integral_nn
                   	path_integral_main.cu
                    autorally_pc_plant
										autorally_plant
                    param_getter
                    )
add_dependencies(path_integral_nn autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(path_integral_nn /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

remove_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_POINT_CLOUD__)

###################################################################################
#Executable for the neural network model without point cloud and with Adam
###################################################################################

add_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_ADAM__)

cuda_add_executable(path_integral_nn_adam
path_integral_main.cu
autorally_pc_plant
autorally_plant
param_getter
)
add_dependencies(path_integral_nn_adam autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(path_integral_nn_adam /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

remove_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_ADAM__)

###################################################################################
#Executable for the neural network model without point cloud and with SGD
###################################################################################

add_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_SGD__)

cuda_add_executable(path_integral_nn_sgd
path_integral_main.cu
autorally_pc_plant
autorally_plant
param_getter
)
add_dependencies(path_integral_nn_sgd autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(path_integral_nn_sgd /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

remove_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_SGD__)

###################################################################################
#Executable for the neural network model without point cloud and with RMSprop
###################################################################################

add_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_RMSPROP__)

cuda_add_executable(path_integral_nn_rmsprop
path_integral_main.cu
autorally_pc_plant
autorally_plant
param_getter
)
add_dependencies(path_integral_nn_rmsprop autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(path_integral_nn_rmsprop /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

remove_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_RMSPROP__)

########################################################################################################
#Executable for the neural network model without point cloud and with SGD and identity mapping for cost
########################################################################################################

add_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_SGD__ -DUSE_IDENTITY_COST__)

cuda_add_executable(path_integral_nn_sgd_identity
path_integral_main.cu
autorally_pc_plant
autorally_plant
param_getter
)
add_dependencies(path_integral_nn_sgd_identity autorally_msgs_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(path_integral_nn_sgd_identity /usr/local/lib/libcnpy.so ${CUDA_curand_LIBRARY} ${catkin_LIBRARIES}
${OpenCV_LIBRARIES})

remove_definitions(-DUSE_NEURAL_NETWORK_MODEL__ -DMPPI_NNET_USING_CONSTANT_MEM__ -DWITH_SGD__  -DUSE_IDENTITY_COST__)

####################################################################################
#Executable for monitoring the status of mppi
####################################################################################

add_executable(path_integral_monitor
			   status_monitor.cpp)
add_dependencies(path_integral_monitor autorally_msgs_gencpp)
target_link_libraries(path_integral_monitor ${catkin_LIBRARIES})

endif(CUDA_FOUND)
