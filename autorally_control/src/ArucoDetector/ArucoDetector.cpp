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
 * @file ArucoDetect3D.cpp
 * @author Henry Liao <hliao62@gatech.edu>
 * @date December 26, 2022
 * @copyright 2022 Georgia Institute of Technology
 * @brief Interface for an AutoRally chassis
 *
 * @details Implementation of ArUco tag detection with 3D depth sensing 
 ***********************************************/


#include "ArucoDetector.h"

namespace autorally_control
{

  ArucoDetector::ArucoDetector(): nh("~"), it_(ros::NodeHandle())

  {
    
    detection_pub_ = nh.advertise<autorally_msgs::arucoDetectionArray>
                        ("detections", 1);

    debug_pub_ = nh.advertise<autorally_msgs::arucoDetection>
                        ("debug", 1);                    

    cam_sub_ = it_.subscribeCamera(
        "/left_camera/image_raw", 1,
        std::bind(
          &ArucoDetector::videoCallback, this, std::placeholders::_1,
          std::placeholders::_2));
  }


  void ArucoDetector::videoCallback(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg
  ){
    
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;

    auto detector_parameters = cv::aruco::DetectorParameters::create();

    const auto cv_image = cv_bridge::toCvShare(image_msg, "bgr8");

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(
      cv_image->image, dictionary, marker_corners, marker_ids,
      detector_parameters, rejected_candidates);


    


    const auto tag_count = marker_ids.size();
    if (tag_count > 0) {
      autorally_msgs::arucoDetectionArray tag_array_msg;

      tag_array_msg.header.frame_id = image_msg->header.frame_id;
      tag_array_msg.header.stamp = image_msg->header.stamp;

      

      for (auto tag_index = 0; tag_index < tag_count; tag_index++) {
        autorally_msgs::arucoDetection aruco_detection_msg;
        aruco_detection_msg.id = marker_ids[tag_index];

        for(int corner = 0; corner< 4; corner++){
          autorally_msgs::point2D point_msg;

          point_msg.x = marker_corners[tag_index][corner].x;
          point_msg.y = marker_corners[tag_index][corner].y;

          aruco_detection_msg.points.push_back(point_msg);
        }
s
        debug_pub_.publish(aruco_detection_msg);
        tag_array_msg.tags.push_back(aruco_detection_msg);
      }

      detection_pub_.publish(tag_array_msg);
    }

  }
}

int main (int argc, char** argv)
  {
    ros::init(argc, argv, "ArucoDetector");
    //ros::NodeHandle n;
    autorally_control::ArucoDetector arDetect;
    ros::spin();
  }
