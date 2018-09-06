#include "CameraAutoBalance.h"
#include <pluginlib/class_list_macros.h>

using namespace FlyCapture2;

PLUGINLIB_EXPORT_CLASS( autorally_core::CameraAutoBalance,  nodelet::Nodelet)

namespace autorally_core
{
CameraAutoBalance::CameraAutoBalance()
{
}

void CameraAutoBalance::onInit()
{
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    frame_counter_ = 0;
    hist_size_ = 256;
    hist_width_ = 256;
    hist_height_ = 256;
    min_gain_ = 1e-2;
    max_gain_ = 24;
    msv_error_tolerance_ = 3;
    epsilon_shutter_ = 1e-3;
    epsilon_gain_ = 1e-1;

    dynamic_reconfigure_server_ = new dynamic_reconfigure::Server<camera_auto_balance_paramsConfig> (pnh);;
    dynamic_reconfigure::Server<camera_auto_balance_paramsConfig>::CallbackType cb;
    cb = boost::bind(&CameraAutoBalance::configCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(cb);

    pnh.getParam("minShutter", min_shutter_);
    pnh.getParam("maxShutter", max_shutter_);
    pnh.getParam("calibrationStep", calibration_step_);
    pnh.getParam("cameraSerialNumber", camera_serial_number_);
    roi_ = cv::Rect(roi_x_top_left_, roi_y_top_left_, roi_x_bottom_right_ - roi_x_top_left_, roi_y_bottom_right_ - roi_y_top_left_);

    BusManager busMgr;
    PGRGuid guid;
    busMgr.GetCameraFromSerialNumber((unsigned int)camera_serial_number_, &guid);
    err_ = cam_.Connect(&guid);
    cameraParametersInitialization();


    sub_ = nh.subscribe("camera/image_color", 100, &CameraAutoBalance::imageCallback, this);
    image_transport::ImageTransport it(nh);
    roi_pub_ = it.advertise(pnh.getNamespace() + "/roi",100);
    hist_pub_ = it.advertise(pnh.getNamespace() + "/histogram", 100);
    NODELET_INFO_STREAM("autobalance nodelet launched with serial " << camera_serial_number_);
}

void CameraAutoBalance::configCallback(const camera_auto_balance_paramsConfig &config, uint32_t level)
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

void CameraAutoBalance::cameraParametersInitialization() {
    prop_.onOff = true;
    prop_.autoManualMode = false;
    prop_.absControl = true;

    u_shutter_ = min_shutter_;
    prop_.type = SHUTTER;
    prop_.absValue = u_shutter_;
    err_ = cam_.SetProperty(&prop_);

    u_gain_ = min_gain_;
    prop_.type = GAIN;
    prop_.absValue = u_gain_;
    err_ = cam_.SetProperty(&prop_);
}

void CameraAutoBalance::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
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

void CameraAutoBalance::autoExposureControl(const cv_bridge::CvImageConstPtr &cv_ptr) {
    double msv = MSV(cv_ptr);

    msv_error_ = msv_reference_ - msv;
    if (msv_error_ > msv_error_tolerance_) {
        if (fabs(max_shutter_ - u_shutter_) < epsilon_shutter_) {
            u_gain_ *= 1 + k_gain_ * msv_error_;
            u_gain_ = saturate(u_gain_, min_gain_, max_gain_);
            prop_.type = GAIN;
            prop_.absValue = u_gain_;
            err_ = cam_.SetProperty(&prop_);
        }
        else {
            u_shutter_ *= 1 + k_shutter_ * msv_error_;
            u_shutter_ = saturate(u_shutter_, min_shutter_, max_shutter_);
            prop_.type = SHUTTER;
            prop_.absValue = u_shutter_;
            err_ = cam_.SetProperty(&prop_);
        }
    }
    else if (msv_error_ < -msv_error_tolerance_) {
        if (fabs(min_gain_ - u_gain_) < epsilon_gain_) {
            u_shutter_ *= 1 + k_shutter_ * msv_error_;
            u_shutter_ = saturate(u_shutter_, min_shutter_, max_shutter_);
            prop_.type = SHUTTER;
            prop_.absValue = u_shutter_;
            err_ = cam_.SetProperty(&prop_);
        }
        else {
            u_gain_ *= 1 + k_gain_ * msv_error_;;
            u_gain_ = saturate(u_gain_, min_gain_, max_gain_);
            prop_.type = GAIN;
            prop_.absValue = u_gain_;
            err_ = cam_.SetProperty(&prop_);
        }
    }
}
    
double CameraAutoBalance::saturate(double x, double min, double max){
    if (x < min)
        x = min;
    else if (x > max)
        x = max;
    return x;
}

double CameraAutoBalance::MSV(cv_bridge::CvImageConstPtr cv_ptr){
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

void CameraAutoBalance::histogram(cv_bridge::CvImageConstPtr cv_ptr, std::vector<int> &hist, cv::Rect &roi,
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

void CameraAutoBalance::plotHistogram(std::vector<int> &hist) {
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


}
