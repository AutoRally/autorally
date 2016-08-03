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
 * @file CameraAutoBalance.h
 * @author Igor Franzoni Okuyama <franzoni315@gmail.com>
 * @date June 14, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief
 *
 * @details This file contains the CameraAutoBalance class
 ***********************************************/

#ifndef PROJECT_CAMERAAUTOBALANCE_H
#define PROJECT_CAMERAAUTOBALANCE_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <flycapture/FlyCapture2.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <autorally_core/camera_auto_balance_paramsConfig.h>

using namespace FlyCapture2;

namespace autorally_core
{

/**
 *  @class CameraAutoBalance CameraAutoBalance.h
 *  "CameraAutoBalance/CameraAutoBalance.h"
 *  @brief Custom auto exposure for PointGrey cameras.
 *
 *  This class controls the camera exposure to light using
 *  a feedback control law. The histogram of the gray image
 *  is used to calculate the Mean Sample Value (MSV) metric, which
 *  is used as exposure quality. For efficiency the histogram
 *  is calculated using a step size of 5 in both x and y directions.
 *  This class enables the user to control the region of interest (ROI)
 *  used to calibrate the camera, the controller gains and the MSV
 *  setpoint. There is also an option to plot the ROI and the
 *  gray histogram. A dynamic reconfigure server is created for
 *  changing these parameters.
 */
class CameraAutoBalance : public nodelet::Nodelet {
public:
    /**
     * Constructor that initializes image transport.
     * All othter initializations are done in method
     * onInit().
     */
    CameraAutoBalance();

    /**
     * Initializes variables, gets parameters from ROS server,
     * starts communicating with PointGrey camera and
     * initialize subscribers and publishers.
     */
    virtual void onInit();
protected:
    /**
     * This callback updates the camera parameters with a control algorithm.
     * @param msg ROS image message
     */
    void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

    /**
     * Imposes x to be in range (min, max), ie,
     * min <= x <= max.
     * @param x Value to be saturated
     * @param min Minimum value possible for x
     * @param max Maximum value possible for x
     */
    double saturate(double x, double min, double max);

    /**
     * Calculates the MSV of a image. The image is converted
     * to gray scale, then its histogram is calculated. The MSV
     * is given by \f$\frac{\sum_{i=0}^{i=N}(i+1)x_i}{\sum_{i=0}^{i=N}(i+1)}\f$.
     * the histogram is equally divided into N regions and \f$x_i\f$
     * represents the sum of histogram values in that region.
     * @param cv_ptr Pointer to opencv Mat()
     *
     */
    double MSV(cv_bridge::CvImageConstPtr cv_ptr);

    /**
     * Calculates the histogram of a image. The image is first
     * converted to gray scale, whose pixels should have intensities
     * between 0 and 255.
     *
     * @param cv_ptr Pointer to opencv Mat()
     * @param hist Vector that stores the histogram values
     * @param decimation_rate Decimation rate used to sample the image. Should be greater or equal to 1.
     *
     */
    void histogram(cv_bridge::CvImageConstPtr cv_ptr, std::vector<int> &hist, cv::Rect &roi, u_int decimation_rate);

    /**
     * Transforms the histogram into an image and publishes it in topic "histogram".
     *
     * @param hist Vector that stores the histogram values
     *
     */
    void plotHistogram(std::vector<int>& hist);

    /**
     * Initialize camera shutter speed and gain to minimum values and to manual mode.
     *
     */
    void cameraParametersInitialization();

    /**
     * Dynamic reconfigure callback.
     *
     * @param config Stores new parameter values sent by the dynamic reconfigure client
     * @param level Bit mask sent by dynamic reconfigure client
     *
     */
    void configCallback(const camera_auto_balance_paramsConfig &config, uint32_t level);

    /**
     * Controls gain and shutter speed of camera so that a MSV setpoint is reached.
     * This is done by a simple controller of the form \f$u = u(1+k e)\f$, where \f$u\f$
     * is the controller output, \f$k\f$ is the controller gain and \f$e\f$ is the error.
     *
     * @param cv_ptr Pointer to opencv Mat()
     *
     */
    void autoExposureControl(const cv_bridge::CvImageConstPtr &cv_ptr);

    ros::Subscriber sub_;   ///<Subscriber for image data
    image_transport::Publisher roi_pub_;    ///<Publisher for ROI image
    image_transport::Publisher hist_pub_;   ///<Publisher for histogram image
    Camera cam_;    ///<PointGrey camera handle
    Property prop_; ///<PointGrey property handle
    Error err_;     ///<PointGrey error handle
    unsigned long int frame_counter_; ///<Counts number of received frames
    int calibration_step_; ///<Determines how often auto exposure control is called.
    int camera_serial_number_;  ///<Camera serial number in decimal
    int roi_x_top_left_;    ///<ROI top left x coordinate
    int roi_y_top_left_;    ///<ROI top left y coordinate
    int roi_x_bottom_right_;    ///<ROI bottom right x coordinate
    int roi_y_bottom_right_;    ///<ROI bottom right y coordinate
    int hist_size_; ///<Number of elements in histogram vector
    int hist_width_;    ///<Number of columns in histogram image
    int hist_height_;   ///<Number of rows in histogram image
    double min_shutter_; ///<Minimum shutter possible for camera
    double max_shutter_; ///<Maximum shutter possible for camera
    double min_gain_;    ///<Minimum gain possible for camera
    double max_gain_;    ///<Maximum gain possible for camera
    double k_shutter_;   ///<Control gain for camera shutter speed
    double k_gain_;      ///<Control gain for camera gain
    double msv_reference_;   ///<Control setpoint for the MSV
    double msv_error_;  ///<Distance from the actual MSV and the desired one
    double msv_error_tolerance_; ///<MSV error tolerance. Controller won't be applied if the error is within tolerance.
    double epsilon_gain_;    ///<Variable used to find how close the camera gain is from its minimum allowed
    double epsilon_shutter_; ///<Variable used to find how close the camera shutter speed is from its maximum allowed
    double u_shutter_;   ///<Control output for shutter
    double u_gain_;      ///<Control output for gain
    bool show_roi_and_hist_;    ///<Enables/disables the publishing of ROI and histogram images.
    cv::Rect roi_;  ///<Defines the region of interest (ROI) used to calibrate the camera.
    std::vector<int> hist_; ///<Stores the histogram values.
    dynamic_reconfigure::Server<camera_auto_balance_paramsConfig    >* dynamic_reconfigure_server_;   ///<Dynamic reconfigure server handle
};

}


#endif //PROJECT_CAMERAAUTOBALANCE_H
