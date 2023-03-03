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

#ifndef ARUCO_DETECTOR_H_
#define ARUCO_DETECTOR_H_



#include <ros/ros.h>
#include <nodelet/nodelet.h>


#include <image_transport/image_transport.h>

#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include "autorally_msgs/arucoDetection.h"
#include "autorally_msgs/arucoDetectionArray.h"
#include "autorally_msgs/point2D.h"
#include <std_msgs/Int32.h>

namespace autorally_control
{

class ArucoDetector 
{
  public:
  ArucoDetector();
  

  private:

  ros::NodeHandle nh;
  ros::NodeHandle nhPvt;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher  detection_pub_;
  ros::Publisher  debug_pub_;
  void videoCallback( const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

};
};

#endif 
