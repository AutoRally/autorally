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
 * @file SafeSpeed.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date June 6, 2012
 * @copyright 2012 Georgia Institute of Technology
 * @brief SafeSpeed class definition
 *
 ***********************************************/
#ifndef SAFE_SPEED_H_
#define SAFE_SPEED_H_

#include <vector>

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <autorally_msgs/safeSpeed.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_core/RingBuffer.h>

#warning autorally_core/safeSpeed.h has been deprecated, refer to autorally_chassis

namespace autorally_core
{

/**
 *  @class SafeSpeed SafeSpeed.h
 *  "autorally_core/SafeSpeed.h"
 *  @brief Given current wheel speed information, generate a maximum throttle
 *         servo position to not drive faster than is considered "safe"
 *
 *  Each node can publish what it believes to be the maximum safe speed the
 *  vehicle can travel, and safe speed computes a throttle servo position that
 *  limits the velocity of the vehicle to the minimum safe speed published. If
 *  there are no outside nodes commanding the throttle, SafeSpeed also acts as
 *  a constant speed controller for the vehicle, driving it at the minimum safe
 *  speed it receives.
 */
class SafeSpeed : public nodelet::Nodelet
{
  /**
    *@brief Internal container for received safeSpeed values
    */
  struct SafeSpeedData
  {
    ros::Time time;
    double safeSpeed;
  };

 public:
  /**
   * @brief Constructor for SafeSpeed
   *
   */
  ROS_DEPRECATED SafeSpeed();
  ~SafeSpeed();

  virtual void onInit();
  void init(ros::NodeHandle &nh);

  /**
   * @brief Calculates the current maximum safe speed.
   *
   * @return double The maximum safe speed in m/s
   */
  double getSafeSpeed() const;

  /**
   * @brief Sets the maximum safe speed.
   *
   * @param maxSafeSpeed The maximum safe speed
   */
  void setMaxSpeed(const double& maxSafeSpeed);

  /**
   * @brief Returns the global maximum speed configured on startup
   *
   * @return double The maximum global safe speed in m/s
   */
  double maxSpeed() const;

  /**
   * @brief Calculates the current maximum safe speed.
   *
   * @param throttleCommand the desired throttle value (from external source)
   * @return double The safeThrottleValue. It will be less than or equal to
   *         throttleCommand
   *
   * In addition to monitoring that the vehicle does not exceed the safeSpeed,
   * safeThrottle can produce an alternate throttle position that drives the
   * vehicle at the safeSpeed.
   */
  double safeThrottle(const double& throttleCommand);

 private:
  struct WheelData {
    autorally_msgs::wheelSpeeds msg;
    double speed;
  };
 
  ros::Subscriber m_safeSpeedSub; ///< Subscriber for SafeSpeed messages
  ros::Subscriber m_speedSub; ///< Subscriber for current vehicle speed
  std::map<std::string, SafeSpeedData > m_safeSpeeds; ///< SafeSpeeds
  double m_safeSpeedTimeout; ///< Maximum time to consider a safeSpeed valid
  double m_maxSafeSpeed; ///< Maximum speed that vehicle can go

  boost::circular_buffer<WheelData> m_vehicleSpeeds;

  double m_prevGoodThrottle;
  bool m_safeSpeedIsInControl;
  
  RingBuffer<double> m_throttleMappings;
    
  /**
   * @brief Callback for safe speed values
   * @param msg the message received from ros comms
   */
  void safeSpeedCallback(const autorally_msgs::safeSpeedConstPtr& msg);

  /**
   * @brief Callback for current vehicle speed information
   * @param msg the message received from ros comms
   */
  void wheelSpeedsCallback(const autorally_msgs::wheelSpeedsConstPtr& msg);
  
  void loadThrottleCalibration(ros::NodeHandle &nh);
};

}
#endif //SAFE_SPEED_H_
