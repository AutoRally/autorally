/*
* Software License Agreement (BSD License)
* Copyright (c) 2018, Georgia Institute of Technology
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
 * @file pid_waypoint_follower.cpp
 * @author Matthieu J. Capuano <capuanomat@gmail.com>
 * @date January 14, 2019
 * @copyright 2018 Georgia Institute of Technology
 * @brief PIDs for throttle and steering
 *
 * @details Autonomously drives system around a given set of waypoints using a PID to compute throttle values and another to compute steering values.
 ***********************************************/

#include "pid_waypoint_follower.h"
//using namespace std;

void pid_waypoint_follower_main::pubControl(float throttle, float steering) {
    autorally_msgs::chassisCommand cmd;
    cmd.throttle = throttle;
    cmd.steering = steering;
    cmd.frontBrake = _DEFAULT_FRONTBRAKE_VALUE;
    cmd.header.stamp = pose_msg.header.stamp;
    cmd.header.frame_id = "pid_waypoint_follower";
    cmd.sender = "pid_waypoint_follower";

    _m_chassisCommandPub.publish(cmd);
}

void pid_waypoint_follower_main::process_pose_main(nav_msgs::Odometry pose_msg) {
    // TODO: Check this, I need an instance of pid_waypoint_follower to  be able to call process_pose. Is there a better way?
    pid_waypoint_follower pid_instance = pid_waypoint_follower();

    tuple<float, float> throt_steer = pid_instance.process_pose(pose_msg);
    throttle = std::get<0>(throt_steer);
    steering = std::get<1>(throt_steer);
    pubControl(throttle, steering);
}

 int main(int argc, char** argv) {
     cout << "--- Starting pid_waypoint_follower_main.cpp ---" << endl;

     ros::init(argc, argv, "pid_waypoint_follower");

     pid_waypoint_follower pid_instance = pid_waypoint_follower();  // TODO: Don't need??

     // Creating NodeHandle, to create subscribers and publishers as below
     ros::NodeHandle n;
     _controller_sub = n.subscribe("/pose_estimate", 1, &pid_waypoint_follower_main::process_pose_main, this);
     // TODO: So then, do I need to move variables like _m_chassisCommandPub out of pid_waypoint_follower.h and into a new h file called pid_waypoint_follower_main.h?
     _m_chassisCommandPub = n.advertise<autorally_msgs::chassisCommand>("/pid_waypoint_follower/chassisCommand", 1);

     // Using cnpy to read in .npz file with waypoints to follow. npz file should be a 1D array.
     cnpy::npz_t path_array = cnpy::npz_load(_WAYPOINTS_SOURCE_FILE);
     cnpy::NpyArray path_raw = path_array["oneD_path"];
     _path = path_raw.as_vec<double>();

     ros::spin();

     cout << "--- Ending pid_waypoint_follower.cpp ---" << endl;
 }
