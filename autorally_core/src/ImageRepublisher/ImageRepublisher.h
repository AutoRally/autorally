#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace autorally_core
{
  /**
   * \brief ImageRepublisher republishes an image topic with a resized image and lower FPS.
   *
   * Transmitting full-resolution, high framerate image topics in a distributed launch can overload a network and hurt
   * other nodes' ability to communicate. ImageRepublisher can be used to subscribe to a local image topic and expose it
   * through a new display topic with scaled-down image sizes and a limitted framerate. These smaller, less frequent,
   * images can be sent across the network without significant performance penalties.
   *
   *
   * __parameters__
   * _fps_ : desired maximum framerate for display topic
   * _resizeHeight_ : desired height of display images. Aspect ratio will be preserved from original images.
   */
  class ImageRepublisher : public nodelet::Nodelet
  {
    public:
      ImageRepublisher();
      virtual void onInit() override;

    protected:
      void imageCallback(const sensor_msgs::Image::ConstPtr &msg);

      void copyVectorToMat(const std::vector<uint8_t>& v, cv::Mat& m);

      void subscriberCallback();

      ros::NodeHandle nh;
      ros::NodeHandle pnh;

      double secondsPerFrame;
      int resizeHeight;
      ros::Time lastFrameTime;

      image_transport::ImageTransport it;
      image_transport::Publisher pub;
      image_transport::Subscriber sub;

      std::mutex connectMutex;
  };

}
