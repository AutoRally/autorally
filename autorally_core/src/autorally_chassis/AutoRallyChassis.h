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
 * @file AutoRallyChassis.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 10, 2016
 * @copyright 2016 Georgia Institute of Technology
 * @brief Interface for an AutoRally chassis
 *
 * @details This file contains the AutoRallyChassis class definition
 ***********************************************/
#ifndef AUTORALLY_CHASSIS_H_
#define AUTORALLY_CHASSIS_H_

#include <map>
#include <vector>
#include <algorithm>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>

#include <autorally_msgs/wheelSpeeds.h>
#include <autorally_msgs/runstop.h>
#include <autorally_msgs/chassisCommand.h>
#include <autorally_msgs/chassisState.h>

#include <autorally_core/SerialInterfaceThreaded.h>

#define PI 3.141592653589793238462;

namespace autorally_core
{

/*
 * @class AutoRallyChassis
 * @brief ROS interface to an AutoRally chassis connected by USB to a computer
 *        
 * The program allows ROS nodes to control throttle, steering, and front brake on the chassis through
 * autorally_msgs::chassisCommand messages and an associated priority. Multiple commanders may be sending command
 * messages at any one time, only the highest priority command is passed to the actuators. Each actuator can be
 * controlled by a separate commander.
 *
 * autorally_msgs::runstop messages control whether motion is enabled through software. For the chassis to be driven 
 * autonomously, there must be at least on publisher of a runstop message with its motion enabled variable set to true
 * 
 * This program publishes:
 * - autorally_msgs::wheelSpeeds messages with the current speed of each wheel in m/s
 * - autorally_msgs::chassisState messages with current control states and commanded actuator values
 * - diagnostics includes a lot of chassis information and message rate information
 */
class AutoRallyChassis : public nodelet::Nodelet
{
 public:
  /*
   * @struct ActuatorConfig 
   * @brief calibrated min, center, and max for the pulse witdth, in us, for an actuator
   *        
   * @note reverse is currently not used
   */
  struct ActuatorConfig
  {
    unsigned short center; ///< calibrated zero of servo in us
    unsigned short min;    ///< calibrated minimum of servo in us (left)
    unsigned short max;    ///< calibrated maximum of servo in us (right)
    bool reverse;          ///< if the servo should be reversed

     ActuatorConfig():
      center(1500),
      min(1000),
      max(2000),
      reverse(false)
    {}
  };

  ~AutoRallyChassis();

  virtual void onInit();

 private:

  /*
   * @struct priorityEntry 
   * @brief Entry for each chassis commander loaded from the commanders file. The highest priority is 0.
   *        
   * @note the id must be the same as the sender in received chassisCommand messages
   */
  struct priorityEntry
  {
    std::string id; ///< Unique identifying string for a program that will control the chassis
    unsigned int priority; ///< Priority of the commander, 0 is highest priotity
  };
  
  struct priorityComparator
  {
    bool operator() (const priorityEntry& a, const priorityEntry& b)
    {
      return a.priority < b.priority;
    }
  };

  SerialInterfaceThreaded serialPort_; ///< USB connection for communication with chassis

  std::map<std::string, ros::Subscriber> chassisCommandSub_; ///< Map of chassisCommand subscribers, one for each
                                                             ///< for each chassis commander in the priorities file
  ros::Subscriber runstopSub_; ///< Subscriber for all incoming runstop messages
  ros::Publisher chassisStatePub_; ///< Publisher for chassisState
  ros::Publisher wheelSpeedsPub_;  ///< Publisher for wheelSpeeds
  ros::Publisher chassisCommandPub_; ///< Publisher for RC chassis commands received from the chassis
  ros::Timer chassisControlTimer_; ///<Timer to trigger throttle set

  std::map<std::string, ActuatorConfig> actuatorConfig_; ///< Map of actuator configs (min, center, max) for each
  ros::Duration chassisCommandMaxAge_; ///< Maximum age to consider a received chassisCommand message valid
  ros::Duration runstopMaxAge_; ///< Maximum age to consider a received runstop message valid
  
  std::map<std::string, autorally_msgs::chassisCommand> chassisCommands_; ///< Map of the most recently received chassis
                                                                          ///< command from each commander
  std::map<std::string, autorally_msgs::runstop> runstops_; ///< Map of the most recently received runstop message from
                                                            ///< all nodes publishing the message
  std::vector<priorityEntry> chassisCommandPriorities_; ///< Priority list used to choose which commander controls the
                                                        ///< actuators

  ///< Text descriptions and multiplier values for each data register received from the ESC. This information comes from
  ///< the Castl Serial Link documenation
  std::vector<std::pair<std::string, double> > escRegisterData_ =
        { {"ESC Input Voltage", 20.0},
          {"ESC Input Ripple Voltage", 4.0},
          {"ESC Current", 50.0},
          {"Throttle ms", 1.0},
          {"Output Power %", 0.2502},
          {"Motor RPM", 20416.66},
          {"Temperature", 30.0},
          {"BEC Voltage", 4.0},
          {"BEC Current", 4.0}, };

  boost::mutex chassisEnableMutex_; ///< mutex for accessing chassis state variables
  bool throttleRelayEnabled_; ///< indicated whther the throttle relay is engaged or not
  bool autonomousEnabled_; ///< indicates if the chassis is in autonomous or manual mode

  boost::mutex rcMutex_; ///< mutex for most recent RC actuator values
  std::map<std::string, double> mostRecentRc_;
  double mostRecentRcSteering_; ///< Most recent RC steering command received from the chassis
  double mostRecentRcThrottle_; ///< Most recent RC throttle command received from the chassis
  double mostRecentRcFrontBrake_; ///< Most recent RC front brake command received from the chassis
  double wheelDiameter_; ///<Diameter of wheels on vehicle in m
  int escDataFailCounter_;  

  std::map<std::string, std::pair<bool, int> > invalidActuatorPulses_;

  /**
   * @brief Callback for receiving control messages
   * @see autorally_core::chassisCommand
   * @param msg the chassisCommand message received from ros comms
   */
  void chassisCommandCallback(const autorally_msgs::chassisCommandConstPtr& msg);

  /**
   * @brief Callback for incoming runstop messages
   * @see autorally_core::runstop
   * @param msg the runstop message received from ros comms
   */
  void runstopCallback(const autorally_msgs::runstopConstPtr& msg) {runstops_[msg->sender] = *msg;}

  /**
   * @brief Callback triggered by the serial interface when data from the chassis is available to read
   */
  void chassisFeedbackCallback();
  
  /**
   * @brief processes a message received from the chassis and publishes it into the ROS system
   * @param msgType one character that is used to parse the information out of the message
   * @param msg the message body to be parsed
   */
  void processChassisMessage(std::string msgType, std::string msg);

  /**
   * @brief Time triggered callback to set chassis actuators
   * @param time information about callback firing
   */
  void setChassisActuators(const ros::TimerEvent& time);
  
  /**
   * @brief Send a set of actuator commands down to the chassis for control
   * @param state actuator values to send down to chassis
   */
  void sendCommandToChassis(autorally_msgs::chassisStatePtr& state);
  
  /**
   * @brief Convert an actuator command to a pulse width in us
   * @param actuatorValue desired actuator command on [-1.0, 1.0]
   * @param actuator name of the actuator to control
   * @return pulseWidth is a scale version of th actuatorValue using the actuator config
   *
   * An actuator command on [-1.0, 1.0] are converted to a pulse width in us to set the PWM signal for the
   * specified actuator using the configuration for the specified actuator.
   */
  short actuatorCmdToMs(double actuatorValue, std::string actuator);
  

  /**
   * @brief Convert pulse width in us to an actuator command on [-1.0, 1.0]
   * @param pulseWidth the pulse width in us of the desired command, normally on roughly [1000, 2000]
   * @param actuator name of the actuator to control
   * @return actuatorValue computed actuator command on [-1.0, 1.0]
   *
   * Actuator command pulse width in us is scaled to [-1, 1]  for the specified actuator according
   * to the configuration for the specified actuator.
   */
  double actuatorUsToCmd(int pulseWidth, std::string actuator);

  /**
   * @brief Loads the actuator configuration for the chassis from the config file loaded onto the parameter
   *        server.
   */
  void loadChassisConfig();

  /**
   * @brief Loads list of actuator commanders with priorities loaded onto the parameter server from the .yaml
   *        file specified in the launch file.
   */
  void loadChassisCommandPriorities();
};

}//end autorally_core
#endif //AUTORALLY_CHASSIS_H_
