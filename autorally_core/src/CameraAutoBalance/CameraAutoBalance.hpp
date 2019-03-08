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
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <autorally_core/camera_auto_balance_paramsConfig.h>

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
 *
 *  It is templated on the CameraAdjuster implementation so that it
 *  can be built for either FlyCapture2 or Spinnaker independently
 */
template<typename CamAdjuster>
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
    CamAdjuster cam_adjuster_;   ///Handle for adjusting camera properties
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



// *********************************** Implementation ********************************************

/**
 * Imposes x to be in range (min, max), ie,
 * min <= x <= max.
 * @param x Value to be saturated
 * @param min Minimum value possible for x
 * @param max Maximum value possible for x
 */
double saturate(double x, double min, double max){
    if (x < min)
        x = min;
    else if (x > max)
        x = max;
    return x;
}


template<typename CamAdjuster>
CameraAutoBalance<CamAdjuster>::CameraAutoBalance() : cam_adjuster_()
{
}

template<typename CamAdjuster>
void CameraAutoBalance<CamAdjuster>::onInit()
{
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    frame_counter_ = 0;
    hist_size_ = 256;
    hist_width_ = 256;
    hist_height_ = 256;
    msv_error_tolerance_ = 3;
    epsilon_shutter_ = 1e-3;
    epsilon_gain_ = 1e-1;

    dynamic_reconfigure_server_ = new dynamic_reconfigure::Server<camera_auto_balance_paramsConfig> (pnh);;
    dynamic_reconfigure::Server<camera_auto_balance_paramsConfig>::CallbackType cb;
    cb = boost::bind(&CameraAutoBalance::configCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(cb);

    pnh.getParam("minShutter", min_shutter_);
    pnh.getParam("maxShutter", max_shutter_);
    pnh.getParam("minGain", min_gain_);
    pnh.getParam("maxGain", max_gain_);
    pnh.getParam("calibrationStep", calibration_step_);
    pnh.getParam("cameraSerialNumber", camera_serial_number_);

    roi_ = cv::Rect(roi_x_top_left_, roi_y_top_left_, roi_x_bottom_right_ - roi_x_top_left_, roi_y_bottom_right_ - roi_y_top_left_);

    cam_adjuster_.SetSerial(camera_serial_number_);

    cam_adjuster_.Connect();
    cameraParametersInitialization();


    sub_ = nh.subscribe("camera/image_color", 100, &CameraAutoBalance::imageCallback, this);
    image_transport::ImageTransport it(nh);
    roi_pub_ = it.advertise(pnh.getNamespace() + "/roi",100);
    hist_pub_ = it.advertise(pnh.getNamespace() + "/histogram", 100);
    NODELET_INFO_STREAM("autobalance nodelet launched with serial " << camera_serial_number_);
}

template<typename CamAdjuster>
void CameraAutoBalance<CamAdjuster>::configCallback(const camera_auto_balance_paramsConfig &config, uint32_t level)
{
    roi_x_top_left_ = config.roi_x_top_left;
    roi_y_top_left_ = config.roi_y_top_left;
    roi_x_bottom_right_ = config.roi_x_bottom_right;
    roi_y_bottom_right_ = config.roi_y_bottom_right;
    if (roi_x_bottom_right_ - roi_x_top_left_ <= 0){
        roi_x_bottom_right_ = roi_x_top_left_ + 1;
        NODELET_INFO_STREAM("ROI bottom right X can't be less than top left X. Setting bottom right X to " << roi_x_bottom_right_);
    }
    if (roi_y_bottom_right_ - roi_y_top_left_ <= 0){
        roi_y_bottom_right_ = roi_y_top_left_ + 1;
        NODELET_INFO_STREAM("ROI bottom right Y can't be less than top left X. Setting bottom right Y to " << roi_y_bottom_right_);
    }
    roi_ = cv::Rect(roi_x_top_left_, roi_y_top_left_, roi_x_bottom_right_ - roi_x_top_left_, roi_y_bottom_right_ - roi_y_top_left_);
    msv_reference_ = config.msvGrayReference;
    show_roi_and_hist_ = config.show_roi_and_hist;
    k_shutter_ = config.kShutter;
    k_gain_ = config.kGain;
    NODELET_INFO_STREAM("Camera autobalance dynamic reconfigure request received.");
}

template<typename CamAdjuster>
void CameraAutoBalance<CamAdjuster>::cameraParametersInitialization() {
    u_shutter_ = min_shutter_;
    cam_adjuster_.SetShutter(u_shutter_);

    u_gain_ = min_gain_;
    cam_adjuster_.SetGain(u_gain_);
}

template<typename CamAdjuster>
void CameraAutoBalance<CamAdjuster>::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if ((frame_counter_ % calibration_step_) == 0) {
        ros::Time t_start = ros::Time::now();
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        autoExposureControl(cv_ptr);

        if(show_roi_and_hist_){
            cv::Mat m (cv_ptr->image);
            cv::rectangle(m,roi_, cv::Scalar(0,0,255), 4);
            cv_bridge::CvImage cvi;
            cvi.header.frame_id = "image";
            cvi.encoding = "bgr8";
            cvi.image = m;
            sensor_msgs::ImageConstPtr im = cvi.toImageMsg();
            roi_pub_.publish(im);
        }
        if ((frame_counter_ % 60) == 0) {
            double processingTime = (ros::Time::now().nsec - t_start.nsec) * 1e-6;
            NODELET_INFO("msv_error: %.1f, shutter: %.3f, gain: %.1f, ProcessingTime: %.2f ms", msv_error_, u_shutter_, u_gain_,
                         processingTime);
        }
    }
    ++frame_counter_;
}

template<typename CamAdjuster>
void CameraAutoBalance<CamAdjuster>::autoExposureControl(const cv_bridge::CvImageConstPtr &cv_ptr) {
    double msv = MSV(cv_ptr);

    msv_error_ = msv_reference_ - msv;
    if (msv_error_ > msv_error_tolerance_) {
        if (fabs(max_shutter_ - u_shutter_) < epsilon_shutter_) {
            u_gain_ *= 1 + k_gain_ * msv_error_;
            u_gain_ = saturate(u_gain_, min_gain_, max_gain_);
            cam_adjuster_.SetGain(u_gain_);
        }
        else {
            u_shutter_ *= 1 + k_shutter_ * msv_error_;
            u_shutter_ = saturate(u_shutter_, min_shutter_, max_shutter_);
            cam_adjuster_.SetShutter(u_shutter_);
        }
    }
    else if (msv_error_ < -msv_error_tolerance_) {
        if (fabs(min_gain_ - u_gain_) < epsilon_gain_) {
            u_shutter_ *= 1 + k_shutter_ * msv_error_;
            u_shutter_ = saturate(u_shutter_, min_shutter_, max_shutter_);
            cam_adjuster_.SetShutter(u_shutter_);
        }
        else {
            u_gain_ *= 1 + k_gain_ * msv_error_;;
            u_gain_ = saturate(u_gain_, min_gain_, max_gain_);
            cam_adjuster_.SetGain(u_gain_);
        }
    }
}

template<typename CamAdjuster>
double CameraAutoBalance<CamAdjuster>::MSV(cv_bridge::CvImageConstPtr cv_ptr){
    histogram(cv_ptr, hist_, roi_, 5);
    std::vector<int>::iterator it;
    int i = 0;
    double msv = 0;
    double sum = 0;
    for (it = hist_.begin(); it != hist_.end(); ++it, ++i){
        msv += (i+1)*(*it);
        sum += (*it);
    }
    msv = msv/sum;

    if(show_roi_and_hist_)
        plotHistogram(hist_);
    return  msv;
}

template<typename CamAdjuster>
void CameraAutoBalance<CamAdjuster>::histogram(cv_bridge::CvImageConstPtr cv_ptr, std::vector<int> &hist, cv::Rect &roi,
                                  u_int decimation_rate) {
    hist.assign(hist_size_,0);
    int i,j;
    int maxI = roi.y + roi.height;
    int maxJ = 3*(roi.x + roi.width);
    int luminance;
    const uchar* p;
    for( i = roi.y; i < maxI; i += decimation_rate)
    {
        p = cv_ptr->image.ptr<uchar>(i);
        for ( j = 3*roi.x; j < maxJ; j += 3*decimation_rate)
        {
            luminance = (int)(0.114*p[j] + 0.587*p[j+1] + 0.299*p[j+2]);
            ++hist[luminance];
        }
    }
}

template<typename CamAdjuster>
void CameraAutoBalance<CamAdjuster>::plotHistogram(std::vector<int> &hist) {
    int max = 0;
    std::vector<int>::iterator it;
    for (it = hist.begin(); it < hist.end(); ++it){
        if(*it > max)
            max = *it;
    }
    cv::Mat histImage = cv::Mat(hist_height_, hist_width_, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 1; i < hist_width_; i++) {
        line(histImage, cv::Point(i - 1, hist_height_ - cvRound(hist[i - 1]*1.0*hist_height_/max)),
             cv::Point(i, hist_height_ - cvRound(hist[i]*1.0*hist_height_/max)),
             cv::Scalar(255, 0, 0), 2, 8, 0);
    }
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = histImage;
    sensor_msgs::ImageConstPtr im = cvi.toImageMsg();
    hist_pub_.publish(im);
}

}  // namespace autorally_core

#endif //PROJECT_CAMERAAUTOBALANCE_H
