/**********************************************
 * @file XbeeCoordinator.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date May 17, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief
 *
 * @details This file contains the XbeeCoordinator class definition
 ***********************************************/

#include "XbeeInterface.h"

#include <boost/lexical_cast.hpp>
#include <autorally_msgs/safeSpeed.h>
#include <std_msgs/ByteMultiArray.h>
 #include <nav_msgs/Odometry.h>

/**
 *  @class XbeeCoordinator XbeeCoordinator.h
 *  "xbee/XbeeCoordinator.h"
 *  @brief Publishes ROS messages from a Coordinator xbee into system
 *
 *  XbeeCoordinator is designed to send periodic information to any XbeeNodes within
 *  range. It gives us the ability to send information to multiple robot while each
 *  independant ROS systems. The current information sent from the coordinator is
 *  a safeSpeed message based on the state of the runStop and gps corrections as
 *  RTCM3 messages for RTK-enabled gps devices on each robot to use.
 *
 */
class XbeeCoordinator
{
  struct RobotState
  {
    std::string address;
    std::string name;
    double velocity;
    ros::Time lastHeartbeat;

    RobotState()
    {}

    RobotState(const std::string& _address, const std::string& _name):
      velocity(-1.0)
    {
      address = _address;
      name = _name;
    }

    RobotState(RobotState const& copy):
      address(copy.address),
      name(copy.name),
      velocity(copy.velocity),
      lastHeartbeat(copy.lastHeartbeat)
    {}

    RobotState& operator=(RobotState const& copy)
    {
      address = copy.address;
      name = copy.name;
      velocity = copy.velocity;
      lastHeartbeat = copy.lastHeartbeat;
      return *this;
    }

    void update(const std::string& /*msg*/)
    {
      lastHeartbeat = ros::Time::now();
    }
  };

 public:

  XbeeCoordinator(ros::NodeHandle &nh, const std::string& port);
  ~XbeeCoordinator();

 private:

  XbeeInterface m_xbee;  ///< Xbee object that manages sending/receiving data
  std::map<std::string, RobotState> m_robotInfos;  ///< Info for all XbeeCoordinators in system
  ros::Subscriber m_safeSpeedSubscriber; ///< Subscriber for safeSpeed
  ros::Subscriber m_baseStationRTKSubscriber; ///< Subscriber for RTK corrections

  std::map<std::string, ros::Publisher> m_recOdomPublishers;

  /**
   * @brief Parse a message received over USB from xbee
   * @param sender identifier for xbee that sent message
   * @param networkAddress address of the originating xbee network
   * @param data message payload
   * @param broadcast if the message was targeted or broadcast
   *
   */
  void processXbeeMessage(const std::string &sender,
                          const std::string &networkAddress,
                          const std::string &data,
                          const bool broadcast);

  /**
  * @brief Callback for new safeSpeed messages
  * @param msg The new vehicle speed
  */
  void safeSpeedCallback(const autorally_msgs::safeSpeedConstPtr& msg);
  
  /**
  * @brief Broadcast RTK correction messages to all XbeeCoordinator
  * @param correction RTCM3.0 RTK correction message to be broadcast
  */
  void gpsCorrectionsCallback(const std_msgs::ByteMultiArray::ConstPtr& correction);

  double unscaleAndClip(int number, double range, double resolution);
  void processXbeeOdom(const std::string& message, const std::string& sender);
};
