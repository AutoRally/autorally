/*
* Software License Agreement (BSD License)
* Copyright (c) 2016, Georgia Institute of Technology
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
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDseparatedseparatedING, BUT NOT LIMITED TO, THE
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
 * @file ArucoDetect3D.h
 * @author Henry Liao <hliao62@gatech.edu>
 * @date December 26, 2022
 * @copyright 2022 Georgia Institute of Technology
 * @brief Interface for an AutoRally chassis
 *
 * @details Definitiion of ArUco tag detection with 3D depth sensing 
 ***********************************************/


PLUGINLIB_EXPORT_CLASS( autorally_core::ArucoDetector,  nodelet::Nodelet)

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>

#include <map>
#include <vector>
#include <algorithm>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

namespace autorally_core
{

class ArucoDetector: public nodelet:Nodelet
{
  public:

  ~ArucoDetector();
  

  private:
  image_transport::SubscriberFilter  leftCamSub_;
  image_transport::SubscriberFilter rightCamSub_;
  message_filters::Subscriber left_info_sub_;
  message_filters::Subscriber right_info_sub_;

  ros::Publisher  detectionPub_;
  void videoCallback( const left_img_msg, const right_img_msg);
}
}
