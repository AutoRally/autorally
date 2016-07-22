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
 * @file ArduinoOnboard.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 22, 2014
 * @copyright 2014 Georgia Institute of Technology
 * @brief
 *
 * @details This file contains the ArduinoOnboard class
 ***********************************************/

#ifndef Arduino_Onboard
#define Arduino_Onboard

#define PI 3.141592653589793238462;

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
//#include <std_msgs/Float64.h>
#include <autorally_core/SerialInterfaceThreaded.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/servoMSG.h>

#include <boost/tokenizer.hpp>
//#include <boost/circular_buffer.hpp>

#include <stdio.h>
#include <string>

#warning autorally_core/ArduinoOnboard.h has been deprecated, refer to autorally_chassis

namespace autorally_core
{

/**
 *  @class ArduinoOnboard ArduinoOnboard.h
 *  "ArduinoOnboard/ArduinoOnboard.h"
 *  @brief Interacts with the onboard Arduino card
 *
 *  The Arduino card publishes data on startup. This class
 *  publishes that data as an arduinoData message.
 */
class ArduinoOnboard : public nodelet::Nodelet
{

 public:
  /**
   * @brief Constructor requires all configuration needed to connect to arduino
   *
   */
  ROS_DEPRECATED ArduinoOnboard();

  ~ArduinoOnboard();

  void onInit();

 private:
  ros::NodeHandle m_nhPvt;
    
  ros::Publisher m_arduinoPub;  ///<Publisher for arduino data
  ros::Publisher m_speedPub;
  ros::Publisher m_wheelSpeedPub;
  ros::Publisher m_irPub;  ///<Publisher for arduino data
  ros::Publisher m_diagPub; ///<Publisher for diagnostics
  ros::Publisher m_servoPub;  ///<Publish servo messages from the RC transmitter

  ros::Time m_lastTime;

  SerialInterfaceThreaded m_port; ///<Serial port for arduino data
  //autorally_msgs::arduinoDataPtr m_arduinoData; ///< local imu data message to be published
  //autorally_msgs::vehicleSpeedPtr m_vehicleSpeed; ///<Local speed message
  //autorally_msgs::wheelSpeedsPtr m_wheelSpeeds; ///<Local wheel speeds message
  //std_msgs::Float64Ptr m_irData;

  double m_wheelDiameter; ///<Diameter of wheels on vehicle in m
  int m_numMovingAverageValues; ///< Number of values used in moving average

  int m_triggerFPS; ///< Frame rate for the camera external trigger

  double m_lfSum; ///< Moving average sum for the front left sensor
  double m_rfSum; ///< Moving average sum for the front right sensor
  double m_lbSum; ///< Moving average sum for the back left sensor
  double m_rbSum; ///< Moving average sum for the back right sensor

  double srvBatteryCrit;
  double srvBatteryLow;
  double camBatteryCrit;
  double camBatteryLow;

  //boost::circular_buffer<double> m_lf;
  //boost::circular_buffer<double> m_rf;
  //boost::circular_buffer<double> m_lb;
  //boost::circular_buffer<double> m_rb;

  bool m_lfEnabled;
  bool m_rfEnabled;
  bool m_lbEnabled;
  bool m_rbEnabled;

  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    /**
   * @brief Process incoming data stream to pull out complete message strings
   * @return bool if a complete message string was found and processed
   */
  void arduinoDataCallback();
  bool findMessage(std::string& msg);
  void loadServoParams();

  struct ServoSettings
  {
    unsigned short center; ///< calibrated zero of servo in us
    unsigned short min;    ///< calibrated minimum of servo in us (left)
    unsigned short max;    ///< calibrated maximum of servo in us (right)
    unsigned short range;  ///< range of servo signal (max-min)
    unsigned char port;    ///< port on servo controller
    bool reverse;          ///< if the servo should be reversed

    // center should be (min+max)/2
    ServoSettings():
      center(1500),
      min(1050),
      max(1950),
      range(max-min),
      port(0),
      reverse(false)
    {}
  };

  std::map<std::string, ServoSettings> m_servoSettings;
};

}

#endif //Arduino_Onboard
