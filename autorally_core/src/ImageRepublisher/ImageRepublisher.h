#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

namespace autorally_core
{

  class ImageRepublisher : public nodelet::Nodelet
  {
    public:
      ImageRepublisher();
      virtual void onInit();
    protected:
      void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

      void copyVectorToMat(const std::vector<uint8_t>& v, cv::Mat& m);

      ros::NodeHandle nh;
      int fps;
      int resizeHeight;
      ros::Time lastFrameTime;

    ros::Subscriber sub;
    image_transport::ImageTransport it;
    image_transport::Publisher pub;
  };

}
