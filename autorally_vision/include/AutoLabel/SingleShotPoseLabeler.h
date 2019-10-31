
/*
* Software License Agreement (BSD License)
* Copyright (c) 2019, Georgia Institute of Technology
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
/**
 * @file SingleShotPoseLabeler.h
 * @author Justin Zheng <justinyzheng@gmail.com>, Jason Gibson <jgibson37@gatech.edu>
 * @date October 28, 2018
 * @copyright 2019 Georgia Institute of Technology
 * @brief Auto Label Tool
 *
 * @details Labels images with vehicle present using pose estimates of chasing vehicle and vehicle being chased.
 **/

#ifndef SINGLESHOTPOSELABELER_H
#define SINGLESHOTPOSELABELER_H

//#include <autorally_vision/AutoLabel/AutoLabelerGeneric.h>
#include <AutoLabel/AutoLabelerGeneric.hpp>


namespace autorally_vision {
  struct CameraParameters {
    cv::Mat K = cv::Mat(3, 4, CV_64F);
    cv::Mat R = cv::Mat(3, 3, CV_64F);
    cv::Mat translation = cv::Mat(3, 1, CV_64F);
  };

  struct BoundingBox3D {
    cv::Mat centroid;
    std::array<cv::Mat, 8> bounding_box;
  };

  struct FlattenedBoundingBox3D {
    cv::Point2f centroid;
    std::array<cv::Point2f, 8> bounding_box;
    double x_range = 0;
    double y_range = 0;
  };

  class SingleShotPoseLabeler : AutoLabellerGeneric {
  public:
    SingleShotPoseLabeler(ros::NodeHandle pNh, std::string mode);
  private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> vehicleOdomSubs_;
    ros::Subscriber selfOdomSub_;
    ros::Subscriber imageSub_;
    ros::Publisher imagePub_;
    // chassis name to camera calibration
    std::map<std::string, CameraParameters> cameraParamMap_;
    double car_length_ = 0.5;
    double car_width_ = 0.5;

    BoundingBox3D getBoundingBoxWorld(const nav_msgs::Odometry& their_pose);
    FlattenedBoundingBox3D getBoundingBoxBody(const nav_msgs::Odometry& my_pose, const nav_msgs::Odometry& their_pose);
    FlattenedBoundingBox3D get2DProjection(const BoundingBox3D& bbox, const std::string& compute_box_name);
    void writeOutData(const FlattenedBoundingBox3D& bbox, std::ofstream& stream);
    void writeOutDebug(const FlattenedBoundingBox3D& bbox, cv::Mat& img);

    void genericPoseCallback(const nav_msgs::Odometry::ConstPtr &position, const std::string name);
    void selfPoseCallback(const nav_msgs::Odometry::ConstPtr &postiion);
    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);
  };
}



#endif //SINGLESHOTPOSELABELER_H
