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
 * @details Autonomously drives system around a given set of waypoints using a PID to compute throttle values and another to compute steering values.
 ***********************************************/

#include "pid_waypoint_follower.h"
using namespace std;


pid_waypoint_follower::pid_waypoint_follower() {
    // Nothing to see here people. Move along.
}


pid_waypoint_follower::~pid_waypoint_follower() {
    // De-constructor, no need for any code here.
    // NOTE: A default deconstructor (such as this one) is automatically generated if you don't manually implement one.
    //       You would usually manually define one like this if you wanted to perform additional tasks.
}


tuple<float, float> pid_waypoint_follower::process_pose(/*nav_msgs::Odometry pose_msg, */Eigen::MatrixXf current_state, Eigen::MatrixXf desired_state) {

    /*
    // Get the pose from the message. This is the actual real time pose estimate from the sensors.
    double x = pose_msg.pose.pose.position.x;
    double y = pose_msg.pose.pose.position.y;
    double x_dot = pose_msg.twist.twist.linear.x; // x_vel
    double y_dot = pose_msg.twist.twist.linear.y; // y_vel

    vector<double> r_p_ya = convert_quat_to_euler(pose_msg.pose.pose.orientation);
    double r = r_p_ya[0];
    double p = r_p_ya[1];
    double ya = r_p_ya[2];

    printf("(x, y): (%f, %f) -- (x_dot, y_dot): (%f, %f) -- Heading: %f \n", x, y, x_dot, y_dot, ya);

    // We rotate the coordinates into a local frame of reference
    Eigen::Matrix<double, 1, 2> vel_wf;
    vel_wf << x_dot, y_dot;
    Eigen::Matrix2d rot_mat;
    rot_mat << cos(ya), sin(ya),
               -sin(ya), cos(ya);

    // Obtaining velocities in x and y directions of local frame of reference
    Eigen::Matrix<double, 1, 2> vel_bf;
    vel_bf = vel_wf * rot_mat;
    double v_x = vel_bf[0];
    double v_y = vel_bf[1];
    */

    // The PIDs are called every time a new pose estimate is generated, which happens at 200Hz
    //tuple<float, float> throt_steer = run_main_controllers(x, y, ya, v_x, v_y);
    tuple<float, float> throt_steer = run_main_controllers(current_state.x_pos, current_state.y_pos, current_state.yaw, current_state.u_x, current_state.u_y, desired_state);

    return throt_steer;
}

/*
vector<double> pid_waypoint_follower::convert_quat_to_euler(geometry_msgs::Pose_<std::allocator<void>>::_orientation_type quat) {
    double q0 = quat.w;
    double q1 = quat.x;
    double q2 = quat.y;
    double q3 = quat.z;

    // Using the 1-2-3 euler angle convention
    double roll = atan2(2*q2*q3 + 2*q0*q1, pow(q3, 2) - pow(q2, 2) - pow(q1, 2) + pow(q0, 2));
    double pitch = -asin(2*q1*q3 - 2*q0*q2);
    double yaw = atan2(2*q1*q2 + 2*q0*q3, pow(q1, 2) + pow(q0, 2) - pow(q3, 2) - pow(q2, 2));
    return {roll, pitch, yaw};
}
*/

tuple<float, float> pid_waypoint_follower::run_main_controllers(double x_current, double y_current, double heading_current, double v_x_current, double v_y_current, Eigen::MatrixXf desired_state) {

    cout << "\n--- Running pid_waypoint_follower function un_main_controllers ---" << endl;

    /*
    // Getting the NOMINAL path around the track
    std::vector<double> t_path, x_path, y_path, heading_path, roll_path, v_x_path, v_y_path, negative_heading_rate_path, steering_path, throttle_path;

    // Extracting information from path. *This assumes the npz file was a 1D array with _STRIDE waypoints!*
    int count = 0;
    for (int i = 0; i < _STRIDE; i++) {
        t_path.push_back(                       _path[i]);
        x_path.push_back(                       _path[1 * _STRIDE + i]);
        y_path.push_back(                       _path[2 * _STRIDE + i]);
        heading_path.push_back(                 _path[3 * _STRIDE + i]);
        roll_path.push_back(                    _path[4 * _STRIDE + i]);
        v_x_path.push_back(                     _path[5 * _STRIDE + i]);
        v_y_path.push_back(                     _path[6 * _STRIDE + i]);
        negative_heading_rate_path.push_back(   _path[7 * _STRIDE + i]);
        steering_path.push_back(                _path[8 * _STRIDE + i]);
        throttle_path.push_back(                _path[9 * _STRIDE + i]);
    }
    */

    // If this is the firts time calling run_main_controllers, we start the chronometer.
    if (!_time_flag) {
        _time_flag = true;
        _start_time = ros::Time::now();
        _last_time = _start_time.toSec() - _start_time.toSec();
    }

    // Updating chronometer
    ros::Duration time_current_raw = ros::Time::now() - _start_time;
    double time_current = time_current_raw.toSec();

    printf("TIMMMEE ====> %f\n", time_current);

    /*
    // Find the entry in t (the time vector) that is closes to the current time and find corresponding x & y (WHERE WE WANT TO BE AT TIME T)
    tuple<double, double> closest_index_and_t = find_nearest(t_path, time_current);
    int index_target = get<0>(closest_index_and_t);
    int t_closest = get<1>(closest_index_and_t);

    // Where we want to be at the current time step:
    double t_target = t_path[index_target];
    double x_target = x_path[index_target];
    double y_target = y_path[index_target];
    */

    float x_target = desired_state(0);
    float y_target = desired_state(1);
    float velocity_target = desired_state(4);


    // 1. Compute throttle PID
    //double velocity_target = v_x_path[index_target];
    double velocity_current = v_x_current;

    float throttle = throttle_path[index_target] + throttlePID(x_current, y_current, heading_current, x_target, y_target, velocity_current, velocity_target, (time_current - _last_time));
    if (throttle > 1.0) {
        throttle = 1.0;
    } else if (throttle < -1.0) {
        throttle = -1.0;
    }

    // 2. Compute steering PID
    float steering = steering_path[index_target] + steeringPID(x_current, y_current, heading_current, x_target, y_target, (time_current - _last_time));
    if (steering > 1.0) {
        steering = 1.0;
    } else if (steering < -1.0) {
        steering = -1.0;
    }

    _last_time = time_current;

    _e_steering_times.push_back(time_current);
    _steering_history.push_back(steering);

    return std::make_tuple(throttle, steering);
}

/*
tuple<double, double> pid_waypoint_follower::find_nearest(std::vector<double> &t_path, double t_current) {
    // To find the element in t_path closest to the current time we:
    // 1. Subtract t_current from all entries in t_path
    // 2. Take the absolute value of all the results
    // 3. Look for the smallest entry (minimum)

    // 1. & 2.
    std::vector<double> differences;
    differences.resize(t_path.size());
    for (int i = 0; i < t_path.size(); i++) {
        differences[i] = abs(t_path[i] - t_current);
    }

    // 3.
    double min = differences[0];
    int min_index = 0;
    for (int i = 1; i < differences.size(); i++) {
        if (differences[i] < min) {
            min = differences[i];
            min_index = i;
        }
    }

    return std::make_tuple(min_index, min);
}
*/

float pid_waypoint_follower::throttlePID(double x_current, double y_current, double heading_current, double x_target, double y_target, double velocity_current, double velocity_target, double dtime) {
    double min_I_term = -5;
    double max_I_term = 5;

    // 1. P-term = position error:
    Eigen::Matrix3d transformation_matrix = get_transformation_matrix(x_current, y_current, x_target, y_target, heading_current);
    Eigen::Vector3d v(x_target, y_target, 1);
    Eigen::Vector3d target_in_body_frame = transformation_matrix * v;
    double P = target_in_body_frame[0];

    // 2. I-term = integral of position error
    double I = _last_throttle_I_term + P * dtime;
    if (I > max_I_term) {
        I = max_I_term;
    } else if (I < min_I_term) {
        I = min_I_term;
    }

    // We update the last I term
    _last_throttle_I_term = I;

    // 3. D-term = velocity error
    double D = velocity_target - velocity_current;

    // Finally, the PID value for the throttle is:
    float PID = _throttle_kP * P + _throttle_kI * I + _throttle_kD * D;
    return PID;
}


float pid_waypoint_follower::steeringPID(double x_current, double y_current, double heading_current, double x_target, double y_target, double dtime) {
    double min_I_term = -1.0;
    double max_I_term = 1.0;

    // Calculating the target heading
    double x_diff = x_target - x_current;
    double y_diff = y_target - y_current;
    double heading_target = atan2(y_diff, x_diff);

    printf("(x_diff, y_diff): (%f, %f)\n", x_diff, y_diff);

    // Calculating the heading error
    vector<double> delta_psi_possible = {(heading_target - heading_current), ((heading_target - heading_current) + 2.0 * M_PI), ((heading_target - heading_current) - 2.0 * M_PI)};
    int min_index = 0;
    double min = fabs(delta_psi_possible[0]);
    for (int i = 1; i < 3; i++) {
        if (fabs(delta_psi_possible[i]) < min) {
            min = fabs(delta_psi_possible[i]);
            min_index = i;
        }
    }
    double delta_psi = delta_psi_possible[min_index];

    printf("DELTA PSI: %f \n", delta_psi);
    //printf("HEADING CURRENT: %f\n", heading_current);
    //printf("HEADING TARGET: %f\n", heading_target);


    // Finding coordinates of the x-y target in the body frame of reference
    Eigen::Matrix3d transformation_matrix = get_transformation_matrix(x_current, y_current, x_target, y_target, heading_current);
    Eigen::Vector3d v(x_target, y_target, 1);
    Eigen::Vector3d target_in_body_frame = transformation_matrix * v;

    printf("TARGET IN BODY FRAME: (%f, %f, %f)\n", target_in_body_frame[0], target_in_body_frame[1], target_in_body_frame[2]);

    // 1. P-term = look-ahead error (e)
    double e = target_in_body_frame[1] + (_x_LA * delta_psi);

    _e_steering_history.push_back(e);

    double P = e;

    // 2. I-term = integral of look-ahead error
    double I = _last_steering_I_term + e * dtime;
    if (I > max_I_term) {
        I = max_I_term;
    } else if (I < min_I_term) {
        I = min_I_term;
    }

    // We update the last I term
    _last_steering_I_term = I;

    // 3. D-term = We do not use a derivative term for steering, too prone to tuning errors
    double D = 0.0;

    // Finally, the PID value for the throttle is:
    float PID = -_steering_kP * P - _steering_kI * I - _steering_kD * D;
    return PID;
}


Eigen::Matrix3d pid_waypoint_follower::get_transformation_matrix(double x_current, double y_current, double x_target, double y_target, double heading) {
    double tx = (cos(heading) * -x_current) + (sin(heading) * -y_current);
    double ty = (-sin(heading) * -x_current) + (cos(heading) * -y_current);
    Eigen::Matrix3d transformation_matrix;
    transformation_matrix << cos(heading), sin(heading), tx,
                            -sin(heading), cos(heading), ty,
                            0, 0, 1;
    return transformation_matrix;
}
