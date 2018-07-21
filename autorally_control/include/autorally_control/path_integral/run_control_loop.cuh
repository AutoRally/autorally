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

#include <boost/thread/thread.hpp>
#include <unistd.h>
#include <chrono>

#include <ros/ros.h>

namespace autorally_control {

template <class CONTROLLER_T> 
void runControlLoop(CONTROLLER_T* controller, AutorallyPlant* robot, SystemParams* params, ros::NodeHandle* mppi_node)
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
  ros::Time last_pose_update = robot->getLastPoseTime();
  ros::Duration optimizationLoopTime(optimization_stride/(1.0*params->hz));
  mppi_node->getParam("optimization_stride", optimization_stride);

  //Set the loop rate
  std::chrono::milliseconds ms{(int)(optimization_stride*1000.0/params->hz)};

  //Start the control loop.
  while (ros::ok()) {
    std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();

    //if (params->debug_mode){ //Display the debug window.
    // controller->costs_->debugDisplay(state(0), state(1));
    //}

    //Figure out how many controls have been published since we were last here and slide the 
    //control sequence by that much.
    optimizationLoopTime = robot->getLastPoseTime() - last_pose_update;
    int stride = round(optimizationLoopTime.toSec()*params->hz);
    if (stride != 0){
      controller->slideControlSeq(stride);
    }

    //Update the state estimate
    if (last_pose_update != robot->getLastPoseTime()){
      last_pose_update = robot->getLastPoseTime();
      fs = robot->getState(); //Get the new state.
      state << fs.x_pos, fs.y_pos, fs.yaw, fs.roll, fs.u_x, fs.u_y, fs.yaw_mder;
    }

    //Compute a new control sequence
    controller->computeControl(state); //Compute the control

    //Get the updated solution
    controlSolution = controller->getControlSeq();
    stateSolution = controller->getStateSeq();
    robot->setSolution(stateSolution, controlSolution, last_pose_update);
  
    //controller->model_->updateState(state, u); //Update the state using motion model.

    //Print debug info
    
    //Sleep 50 microseconds
    std::chrono::duration<double, std::milli> fp_ms = std::chrono::steady_clock::now() - loop_start;
    while(ros::ok() && (fp_ms < ms || last_pose_update == robot->getLastPoseTime())){
      usleep(50);
      fp_ms = std::chrono::steady_clock::now() - loop_start;
    }
    num_iter += 1;
  }
}

}

#endif /*RUN_CONTROLLER_CUH_*/