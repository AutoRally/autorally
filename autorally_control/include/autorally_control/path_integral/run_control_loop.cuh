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
 * @file run_control_loop.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Runs the control loop for a given feedback control law
 * parameter server.
 ***********************************************/

#ifndef RUN_CONTROLLER_CUH_
#define RUN_CONTROLLER_CUH_

#include "autorally_plant.h"
#include "param_getter.h"

#include <autorally_control/ddp/ddp_model_wrapper.h>
#include <autorally_control/ddp/ddp_tracking_costs.h>
#include <autorally_control/ddp/ddp.h>

//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <atomic>


#include <boost/thread/thread.hpp>
#include <unistd.h>
#include <chrono>

#include <ros/ros.h>

namespace autorally_control {

//Typedefs for tracking controller
typedef NeuralNetModel<7,2,3,6,32,32,4> DynamicsDDP;
typedef ModelWrapperDDP<DynamicsDDP> ModelDDP;
typedef TrackingCostDDP<ModelDDP> RunningCostDDP;
typedef TrackingTerminalCost<ModelDDP> TerminalCostDDP;

template <class CONTROLLER_T> 
void runControlLoop(CONTROLLER_T* controller, AutorallyPlant* robot, SystemParams* params, 
                    ros::NodeHandle* mppi_node, std::atomic<bool>* is_alive)
{
  //Initial condition of the robot
  Eigen::MatrixXf state(7,1);
  AutorallyPlant::FullState fs;
  state << params->x_pos, params->y_pos, params->heading, 0, 0, 0, 0;
  
  //Initial control value
  Eigen::MatrixXf u(2,1);
  u << 0, 0;

  std::vector<float> controlSolution;
  std::vector<float> stateSolution;

  //Counter, timing, and stride variables.
  int num_iter = 0;
  int optimization_stride;
  int status = 1;
  double avgOptimizationLoopTime = optimization_stride/(1.0*params->hz);
  ros::Time last_pose_update = robot->getLastPoseTime();
  ros::Duration optimizationLoopTime(optimization_stride/(1.0*params->hz));
  mppi_node->getParam("optimization_stride", optimization_stride);

  //Set the loop rate
  std::chrono::milliseconds ms{(int)(optimization_stride*1000.0/params->hz)};

  //Now define the DDP model, costs, and optimizer
  float2 control_constraints[2] = {make_float2(-.99, .99), make_float2(-.99, params->max_throttle)};
  DynamicsDDP* ddp_internal_model = new DynamicsDDP(1.0/params->hz, control_constraints);
  ddp_internal_model->loadParams(params->model_path); //Load the model parameters from the launch file specified path
  ModelDDP ddp_model(ddp_internal_model);
  util::DefaultLogger logger;
  bool verbose = false;
  DDP<ModelDDP> ddp_solver(1.0/params->hz, params->num_timesteps, 1, &logger, verbose);
  RunningCostDDP::StateCostWeight Q;
  Q.setIdentity();
  Q.diagonal() << 0.05, 0.05, 0.25, 0.0, 0.05, 0.01, 0.01;
  TerminalCostDDP::Hessian Qf;
  Qf.setIdentity();
  Qf.diagonal() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  RunningCostDDP::ControlCostWeight R;
  R.setIdentity();
  R.diagonal() << 10.0, 10.0;
  RunningCostDDP run_cost(Q,R, params->num_timesteps);
  TerminalCostDDP terminal_cost(Qf);
  Eigen::Matrix<float, DynamicsDDP::CONTROL_DIM, 1> U_MIN;
  Eigen::Matrix<float, DynamicsDDP::CONTROL_DIM, 1> U_MAX;
  U_MIN << -0.99, -0.99;
  U_MAX << 0.99, params->max_throttle;

  //Eigen matrices for holding the control and state solutions
  Eigen::MatrixXf control_traj(DynamicsDDP::CONTROL_DIM, params->num_timesteps);
  control_traj = Eigen::MatrixXf::Zero(DynamicsDDP::CONTROL_DIM, params->num_timesteps);

  //Start the control loop.
  while (is_alive->load()) {
    std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();
    num_iter ++;

    if (params->debug_mode){ //Display the debug window.
     cv::Mat debug_img = controller->costs_->getDebugDisplay(state(0), state(1));
     robot->setDebugImage(debug_img);
    }
    //Update the state estimate
    if (last_pose_update != robot->getLastPoseTime()){
      optimizationLoopTime = robot->getLastPoseTime() - last_pose_update;
      last_pose_update = robot->getLastPoseTime();
      fs = robot->getState(); //Get the new state.
      state << fs.x_pos, fs.y_pos, fs.yaw, fs.roll, fs.u_x, fs.u_y, fs.yaw_mder;
    }

    //Figure out how many controls have been published since we were last here and slide the 
    //control sequence by that much.
    int stride = round(optimizationLoopTime.toSec()*params->hz);
    if (status != 0){
      stride = optimization_stride;
    }
    if (stride >= 0 && stride < params->num_timesteps){
      controller->slideControlSeq(stride);
    }

    //Update the average loop time based on pose estimate timestamps
    avgOptimizationLoopTime = (num_iter - 1.0)/num_iter*avgOptimizationLoopTime + optimizationLoopTime.toSec()/num_iter; 

    //Compute a new control sequence
    controller->computeControl(state); //Compute the control

    //Get and set the updated solution
    controlSolution = controller->getControlSeq();
    stateSolution = controller->getStateSeq();

    //Now compute feedback gains
    for (int t = 0; t < params->num_timesteps; t++){
      for (int i = 0; i < DynamicsDDP::CONTROL_DIM; i++){
        control_traj(i,t) = controlSolution[DynamicsDDP::CONTROL_DIM*t + i];
      }
    }
    run_cost.setTargets(stateSolution.data(), controlSolution.data(), params->num_timesteps);
    terminal_cost.xf = run_cost.traj_target_x_.col(params->num_timesteps - 1);
    auto result = ddp_solver.run(state, control_traj, ddp_model, run_cost, terminal_cost, U_MIN, U_MAX);

    robot->setSolution(stateSolution, controlSolution, result.feedback_gain, last_pose_update, avgOptimizationLoopTime);

    status = robot->checkStatus();
    if (status != 0 && params->debug_mode){
      for (int t = 0; t < optimization_stride; t++){
        u << controlSolution[2*t], controlSolution[2*t + 1];
        controller->model_->updateState(state, u); 
      }
    }
    
    //Sleep 50 microseconds
    std::chrono::duration<double, std::milli> fp_ms = std::chrono::steady_clock::now() - loop_start;
    int count = 0;
    while(is_alive->load() && (fp_ms < ms || (last_pose_update == robot->getLastPoseTime() && status==0))) {
      usleep(50);
      fp_ms = std::chrono::steady_clock::now() - loop_start;
      count++;
    }
  }
}

}

#endif /*RUN_CONTROLLER_CUH_*/