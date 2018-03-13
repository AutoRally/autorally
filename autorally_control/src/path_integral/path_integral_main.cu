/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file path_integral_main.cpp
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Main file model predictive path integral control.
 *
 ***********************************************/

//Some versions of boost require __CUDACC_VER__, which is no longer defined in CUDA 9. This is
//the old expression for how it was defined, so should work for CUDA 9 and under.
#define __CUDACC_VER__ __CUDACC_VER_MAJOR__ * 10000 + __CUDACC_VER_MINOR__ * 100 + __CUDACC_VER_BUILD__ 

#include <autorally_control/path_integral/meta_math.h>
#include <autorally_control/path_integral/param_getter.h>

#include <autorally_control/path_integral/mppi_controller.cuh>
#include <autorally_control/path_integral/costs.cuh>
#include <autorally_control/path_integral/generalized_linear.cuh>
#include <autorally_control/path_integral/car_bfs.cuh>
#include <autorally_control/path_integral/car_kinematics.cuh>
#include <autorally_control/path_integral/run_control_loop.cuh>

#include <ros/ros.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
#endif

#include <autorally_control/path_integral/neural_net_model.cuh>

using namespace autorally_control;

#ifdef USE_NEURAL_NETWORK_MODEL__ /*Use neural network dynamics model*/

const int MPPI_NUM_ROLLOUTS__ = 1200;
const int BLOCKSIZE_X = 8;
const int BLOCKSIZE_Y = 16;
typedef NeuralNetModel<7,2,CarKinematics,3,6,32,32,4> DynamicsModel;

#elif USE_BASIS_FUNC_MODEL__ /*Use the basis function model*/

const int MPPI_NUM_ROLLOUTS__ = 2560;
const int BLOCKSIZE_X = 16;
const int BLOCKSIZE_Y = 4;
typedef GeneralizedLinear<CarBasisFuncs, 7, 2, 25, CarKinematics, 3> DynamicsModel;

#endif

//Convenience typedef for the MPPI Controller.
typedef MPPIController<DynamicsModel, MPPICosts, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;

int main(int argc, char** argv) {
  //Ros node initialization
  ros::init(argc, argv, "mppi_controller");
  ros::NodeHandle mppi_node("~");

  std::cout << __CUDACC_VER_MAJOR__ << ", " << __CUDACC_VER_MINOR__ << ", " << __CUDACC_VER__ << std::endl; 

  //Load setup parameters
  SystemParams params;
  loadParams(&params, mppi_node);

  //Define the mppi costs
  MPPICosts* costs = new MPPICosts(mppi_node);

  //Define the internal dynamics model for mppi
  float2 control_constraints[2] = {make_float2(-.99, .99), make_float2(-.99, params.max_throttle)};
  DynamicsModel* model = new DynamicsModel(1.0/params.hz, control_constraints);
  model->loadParams(params.model_path); //Load the model parameters from the launch file specified path

  //Define the controller
  float init_u[2] = {(float)params.init_steering, (float)params.init_throttle};
  float exploration_std[2] = {(float)params.steering_std, (float)params.throttle_std};
  Controller mppi(model, costs, params.num_timesteps, params.hz, params.gamma, exploration_std, init_u, params.num_iters);

  runControlLoop<Controller>(mppi, params, mppi_node);

  mppi.deallocateCudaMem();
}
