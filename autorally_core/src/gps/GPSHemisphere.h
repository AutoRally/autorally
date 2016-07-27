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
 * @file GPSHemisphere.h
 * @author Brian Goldfain <bgoldfain3@gatech.edu>
 * @date November 5, 2013
 * @copyright 2013 Georgia Institute of Technology
 * @brief GPSHemisphere class definition
 *
 ***********************************************/
#ifndef GPS_HEMISPHERE_H_
#define GPS_HEMISPHERE_H_

#include <autorally_core/SerialInterfaceThreaded.h>
#include <autorally_core/Diagnostics.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <std_msgs/ByteMultiArray.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>
#include <queue>
#include <vector>

/**
 *  @class GPSHemisphere GPSHemisphere.h
 *  "autorally_core/GPSHemisphere.h"
 *  @brief Interact with a hemisphere R320 GPS Base station
 *
 * Interacts with a RTK-compatible GPS base station, specifically the
 * Hemisphere R320. The main porpose of the base station is to send RTK
 * corrections to other GPS devices onboard running robot. Additionally,
 * position data for the base station is published. The position data is
 * received in the form of NMEA 0183 messages, and the corrections are RTCM 3.0
 *
 * @note It is assumed that the device is already configured to stream GPGGA
 *  messages on portA and the correction data on portB. See the wiki pages for
 *  additional GPS setup information.
 *
 * @note Covariance types in navSat message are explained here:
 *      http://answers.ros.org/question/10310/calculate-navsatfix-covariance/
 *
 * Useful Commands:
 *  Base:
 *    $jmode,base,yes - put in base mode
 *    $jasc,rtcm3,1 - enable rtcm3 corrections
 *    $JRTK,5,1 - start correction data stream
 *  Rover:
 *    $JDIFF,BEACON - enable local RTCM corrections
 *    $JDIFF,WAAS - diable local RTCM corrections
 *  Both:
 *    $JSAVE - save current configuration
 *    $japp - use to confirm sbasrtkb program?
 *    $JRTK,6 - use to check if rtk ready
 *    $JRTK,1,P - set receiver reference coordinates to current position
 *    $JRTK,1 - get receiver reference position
 *    $JASC,GPGGA,20 - get gpgga message at 20hz
 *    $JASC,GPGSA,1 - get gpgsa message at 1hz
 *    $JASC,GPGST,1 - get gpgst message at 1hz
 *
 *
 */

 
class GPSHemisphere
{

 public:
  /**
   * Constructor reads connection information for each port and connects to the
   * Hemisphere GPS R320, and performs other initialization.
   * @param nh NodeHandle

   */
  GPSHemisphere(ros::NodeHandle &nh);

  ~GPSHemisphere();

  /**
   * @brief Timer triggered callback to request RTK status from base station
   * @param time information about callback execution
   */
  void rtkStatusCallback(const ros::TimerEvent& time);

  /**
  * @brief Timer triggered callback to update the device reference location
  * @param time information about callback execution
  *
  * This may be periodically needed for the base station??????
  */
  void updateReferenceLocationCallback(const ros::TimerEvent& time);

 private:
  ros::Timer m_rtkStatusTimer; ///<Timer to trigger RTK status updates
  ros::Timer m_refLocTimer; ///<Timer to trigger reference location updates

  ros::Publisher m_statusPub; ///<Publisher for base pose data.
  ros::Publisher m_rtcm3Pub; ///<Publisher for RTCM3 correction data.
  ros::Publisher m_utcPub; ///<Publisher for UTC time data.
  ros::Subscriber m_rtcm3Sub; ///<Subscriber for RTCM3 correction data.

  sensor_msgs::NavSatFix m_navSatFix; ///<Base station position information
  std_msgs::ByteMultiArray m_rtkCorrection; ///<Outgoing RTK correction data
  sensor_msgs::TimeReference m_timeUTC; ///<Base station position information
  int m_secondsToToday; //used to find UTC time if needed

  SerialInterfaceThreaded m_portA; ///<Serial port for status updates
  SerialInterfaceThreaded m_portB; ///<Serial port to receive RTK corrections

  ros::Time m_previousCovTime;
  double m_accuracyRTK;
  double m_accuracyWAAS;
  double m_accuracyAutonomous;
  double m_gpsTimeOffset;
  std::string m_utcSource;
  std::string m_statusPositionSource;

  ros::Time m_mostRecentRTK;
  bool m_rtkEnabled;
  bool m_showGsv;
  /**
  * @brief Callback for incoming data on portA
  *
  */
  void gpsInfoCallback();

  /**
  * @brief Callback for incoming data on portB
  *
  */
  void rtcmDataCallback();

  /**
  * @brief Callback for correction data received from a base station
  *
  * @param msg correction data from base station
  */
  void rtcmCorrectionCallback(const std_msgs::ByteMultiArray& msg);

  /**
  * @brief Process a NMEA 0183 message from the base station
  * @param msg The full message stripped of leading $ and ending cr lf
  *
  */
  void processGPSMessage(std::string& msg);

  /**
  * @brief Process the quality component from a NMEA 0183 GPGGA message
  * @param qual The string containing the quality information
  * @return std::string the converted quality
  */
  std::string processQuality(const std::string& qual);
  
  std::string processMode(const std::string& modeIndicator);

  /**
  * @brief Process the latitude component from a NMEA 0183 GPGGA message
  * @param lat The string containing the latitude data in format DDMM.MMMMM
  *        (degrees, minutes, decimal minutes)
  * @param latInd Indicates north or south latitude
  * @return float Latitude in degrees
  */
  double processLatitude(const std::string& lat, const std::string& latInd);

  /**
  * @brief Process the longitude component from a NMEA 0183 GPGGA message
  * @param lon The string containing the longitude data in format DDDMM.MMMMM
  *        (degrees, minutes, decimal minutes)
  * @param lonInd Indicates east or west longitude
  * @return float Longitude in degrees
  */
  double processLongitude(const std::string& lon, const std::string& lonInd);

    /**
  * @brief Process the full altitude component from a NMEA 0183 GPGGA message
  * @param antAlt Antenna altitude
  * @param antAltUnits Units of altitude
  * @param geodSep Geoidal separation
  * @param geodSepUnits Units of geoidal separation
  * @return float Altitude above sea level in meters
  * @note Currently, only meters (m) are supported as input units
  *
  * The true elevation is the antAlt+geodSep which compensates for deviations
  * from the earth's ideal elevation ellipsoid.
  */
  double processAltitude(const std::string& antAlt, const std::string &antAltUnits,
                        const std::string& geodSep, const std::string &geodSepUnits);

  void processUTC(const std::string& utc, const std::string& source);
  double GetUTC(const std::string& utc);
};
#endif //GPS_HEMISPHERE_H_
