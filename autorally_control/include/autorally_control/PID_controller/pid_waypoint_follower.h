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
 * @date November 19, 2018
 * @copyright 2018 Georgia Institute of Technology
 * @brief PIDs for throttle and steering
 *
 * @details Autonomously drives system around a given set of waypoints using a PID compute throttle values and to compute steering values.
 ***********************************************/

// TODO: Make a launch file
// When debugging ROS: Look at topics your program is supposed to be taking in and check they're all there


#ifndef PID_WAYPOINT_FOLLOWER
#define PID_WAYPOINT_FOLLOWER

// General Imports:
#include <chrono>
#include <vector>

#include <iostream>
#include <string>
#include <cmath>
#include <math.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>

// ROS Imports
#include "cnpy.h"
#include <ros/ros.h>
#include <ros/time.h>
//#include <nodelet/nodelet.h>

// Autorally imports
#include <autorally_msgs/chassisCommand.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class pid_waypoint_follower {
    public:
        pid_waypoint_follower();
        ~pid_waypoint_follower();

        tuple<float, float> process_pose(/*nav_msgs::Odometry pose_msg, */Eigen::MatrixXf current_state, Eigen::MatrixXf desired_state); // Callback function

    private:
        /** --- VARIABLES --- **/
        // ROS Imports
        ros::Publisher _m_chassisCommandPub;
        ros::Subscriber _controller_sub;

        // Final variables
        double _DEFAULT_FRONTBRAKE_VALUE = -5;

        // Path variables
        string _WAYPOINTS_SOURCE_FILE = "/home/capuanomat/catkin_ws/src/autorally_private/autorally_private_control/src/matthieu_path_follower/FastDataShort1D.npz";
        int _STRIDE = 7040; // This should be equal to the number of waypoints in the above file.
        std::vector<double> _path;

        // Important PID terms
        double _last_throttle_I_term = 0;
        double _last_steering_I_term = 0;
        double _last_throttle_error = 0;
        double _last_steering_error = 0;

        // PID Gains
        double _throttle_kP = 0.05;
        double _throttle_kI = 0.0;
        double _throttle_kD = 0.0;

        double _steering_kP = 0.05;
        double _steering_kI = 0.0;
        double _steering_kD = 0.0;

        std::vector<double> _e_steering_history, _e_steering_times, _steering_history;

        // Look Ahead Term
        double _x_LA = 12.0;

        // Timer variables
        bool _time_flag = false;
        ros::Time _start_time;
        double _last_time;

        /** --- METHODS --- **/
        //vector<double> convert_quat_to_euler(geometry_msgs::Pose_<std::allocator<void> >::_orientation_type quat);
        tuple<float, float> run_main_controllers(double x_current, double y_current, double heading_current, double v_x_current, double v_y_current, Eigen::MatrixXf desired_state);
        tuple<double, double> find_nearest(std::vector<double> &t_path, double value);
        float throttlePID(double x_current, double y_current, double heading_current, double x_target, double y_target, double velocity_current, double velocity_target, double dtime);
        float steeringPID(double x_current, double y_current, double heading_current, double x_target, double y_target, double dtime);
        Eigen::Matrix3d get_transformation_matrix(double x_current, double y_current, double x_target, double y_target, double heading);
};

#endif
