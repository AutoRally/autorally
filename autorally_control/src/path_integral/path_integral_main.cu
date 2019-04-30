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
#include <autorally_control/path_integral/autorally_pc_plant.h>
#include <autorally_control/path_integral/autorally_plant.h>
#include <autorally_control/PathIntegralParamsConfig.h>
#include <autorally_control/path_integral/costs.cuh>
#include <autorally_control/path_integral/costs_pc.cuh>

//Including neural net model
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
#endif
#include <autorally_control/path_integral/neural_net_model.cuh>
#include <autorally_control/path_integral/car_bfs.cuh>
#include <autorally_control/path_integral/car_kinematics.cuh>
#include <autorally_control/path_integral/generalized_linear.cuh>
#include <autorally_control/path_integral/run_control_loop.cuh>
#include <autorally_control/path_integral/mppi_controller.cuh>
#include <autorally_control/path_integral/mppi_adaptive_controller.cuh>
#include <autorally_control/path_integral/mppi_adaptive_identity_controller.cuh>
#include <autorally_control/path_integral/adam.cuh>
#include <autorally_control/path_integral/sgd.cuh>
#include <autorally_control/path_integral/rmsprop.cuh>

#include <ros/ros.h>
#include <atomic>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

using namespace autorally_control;

#ifdef USE_NEURAL_NETWORK_MODEL__ /*Use neural network dynamics model*/
const int MPPI_NUM_ROLLOUTS__ = 1920;
const int BLOCKSIZE_X = 8;
const int BLOCKSIZE_Y = 16;
typedef NeuralNetModel<7,2,3,6,32,32,4> DynamicsModel;
#elif USE_BASIS_FUNC_MODEL__ /*Use the basis function model* */
const int MPPI_NUM_ROLLOUTS__ = 2560;
const int BLOCKSIZE_X = 16;
const int BLOCKSIZE_Y = 4;
typedef GeneralizedLinear<CarBasisFuncs, 7, 2, 25, CarKinematics, 3> DynamicsModel;
#endif

#ifdef WITH_POINT_CLOUD__ /*Use plant and cost function with support for point cloud processing*/
  typedef AutorallyPCPlant ControllerPlant;
  typedef MPPIPCCosts ControllerCosts;
#else
  typedef AutorallyPlant ControllerPlant;
  typedef MPPICosts ControllerCosts;
#endif

int main(int argc, char** argv) {
  //Ros node initialization
  ros::init(argc, argv, "mppi_controller");

  ros::NodeHandle mppi_node("~");

  //Load setup parameters
  SystemParams params;
  loadParams(&params, mppi_node);

  //Define the mppi costs
  ControllerCosts* costs = new ControllerCosts(mppi_node);

  //Define the internal dynamics model for mppi
  float2 control_constraints[2] = {make_float2(-.99, .99), make_float2(-.99, params.max_throttle)};
  DynamicsModel* model = new DynamicsModel(1.0/params.hz, control_constraints);
  model->loadParams(params.model_path); //Load the model parameters from the launch file specified path

  int optimization_stride = getRosParam<int>("optimization_stride", mppi_node);

  bool use_cem;
  mppi_node.param<bool>("use_cem", use_cem, false);

  //Define the controller
  float init_u[2] = {(float)params.init_steering, (float)params.init_throttle};
  float exploration_std[2] = {(float)params.steering_std, (float)params.throttle_std};

  #ifdef WITH_ADAM__
    typedef AdamOptimizer Optimizer;
    Optimizer* optim = new Optimizer(params.num_timesteps*DynamicsModel::CONTROL_DIM, params.lr,
                                                  params.weight_decay, params.beta1, params.beta2, params.eps,
                                                  params.amsgrad);
  #elif WITH_SGD__
    typedef SGDOptimizer Optimizer;
    Optimizer* optim = new Optimizer(params.num_timesteps*DynamicsModel::CONTROL_DIM, params.lr, params.momentum,
                                     params.dampening, params.weight_decay, params.nesterov);
  #else
    typedef RMSpropOptimizer Optimizer;
    Optimizer* optim = new Optimizer(params.num_timesteps*DynamicsModel::CONTROL_DIM, params.lr, params.alpha,
                                     params.eps, params.weight_decay, params.momentum, params.centered);
  #endif

  #ifdef USE_IDENTITY_COST__ /*Use an identity mapping on cost instead of exponentiated*/
    typedef MPPIAdaptiveIdentityController<DynamicsModel, ControllerCosts, Optimizer, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;
    Controller* mppi = new Controller(model, costs, optim, params.num_timesteps, params.hz, params.gamma,
                                  exploration_std, init_u, params.num_iters, optimization_stride, 0,
                                  params.dist_type);
  #elif WITH_ADAM__ || WITH_SGD__ || WITH_RMSPROP__
    typedef MPPIAdaptiveController<DynamicsModel, ControllerCosts, Optimizer, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;
    Controller* mppi = new Controller(model, costs, optim, params.num_timesteps, params.hz, params.gamma,
                                  exploration_std, init_u, params.num_iters, optimization_stride, 0,
                                  params.dist_type, use_cem);
  #else
    typedef MPPIController<DynamicsModel, ControllerCosts, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;
    Controller* mppi = new Controller(model, costs, params.num_timesteps, params.hz, params.gamma, exploration_std,
                                      init_u, params.num_iters, optimization_stride, 0);
  #endif

  ControllerPlant* robot = new ControllerPlant(mppi_node, mppi_node, params.debug_mode, params.hz, false, MPPI_NUM_ROLLOUTS__);

  //Setup dynamic reconfigure callback
  dynamic_reconfigure::Server<PathIntegralParamsConfig> server;
  dynamic_reconfigure::Server<PathIntegralParamsConfig>::CallbackType callback_f;
  callback_f = boost::bind(&ControllerPlant::dynRcfgCall, robot, _1, _2);
  server.setCallback(callback_f);

  boost::thread optimizer;

  std::atomic<bool> is_alive(true);
  optimizer = boost::thread(&runControlLoop<Controller, ControllerPlant>, mppi, robot, &params, &mppi_node, &is_alive);

  ros::spin();

  //Shutdown procedure
  is_alive.store(false);
  optimizer.join();
  robot->shutdown();
  mppi->deallocateCudaMem();
  delete robot;
  delete mppi;
  delete costs;
  delete model;
}
