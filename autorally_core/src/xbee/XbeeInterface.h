/**********************************************
 * @file XbeeInterface.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date May 17, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief Interface for Xbee Pro 900 in API mode
 *
 * @details This file contains the XbeeInterface class definition
 ***********************************************/
#ifndef XBEE_INTERFACE
#define XBEE_INTERFACE

#include <boost/function.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <autorally_core/SerialInterfaceThreaded.h>

/**
 *  @class XbeeInterface XbeeInterface.h
 *  "xbee/XbeeInterface.h"
 *  @brief Interact with an Xbee in API mode
 *
 *  Implements an interface for Xbee modules that supports api mode. The Xbee
 *  api specification is mostly implemented. You can send broadcast or targeted
 *  messages to any other Xbee in the network. You can also get and set any of
 *  the Xbee parameters available in AT mode. This uses SerialSensorInterface
 *  to send and receieve Xbee data over USB. There are also startup Xbee params
 *  that can be verified, as well as params that can be updated periodically in
 *  the diagnostic information. These data items are set in the launch files.
 *
 *  @todo Make return statuses and such more visible outside of this class
 */
class XbeeInterface
{
 public:
  SerialInterfaceThreaded m_port;
  
  boost::function<void(const std::string &sender,
                       const std::string &networkAddress,
                       const std::string &data,
                       const bool broadcast)> m_receiveMessageCallback;

  /**
   * @brief XbeeInterface constructor
   * @param nh NodeHandle used to register stuff
   * @param port name of the xbee
   *
   * Connects to the specified serial device, sets up diagnostics, starts
   * polling timers, verifies startup configuration of Xbee
   */
  XbeeInterface(ros::NodeHandle &nh, const std::string& port);
  ~XbeeInterface();

  /**
   * @brief Set a function pointer to call to process a newly received message
   * @param callback a boost::function to save and be call to process messages
   *
   * The function set to be triggered must accept 3 const std::string, for the
   * information given in addition to the received message.
   */
  void registerReceiveMessageCallback(
                 boost::function<void(const std::string &sender,
                                      const std::string &networkAddress,
                                      const std::string &data,
                                      const bool broadcast)> callback);

  /**
   * @brief Get the network address of the Xbee it is connected to.
   * @return std::string the network address of this Xbee
   *
   * The network address is a concatenation of the AT commands SH and SL, which
   * are retreived and verified on startup (serial number high byte, serial
   * number low byte)
   */
  std::string getAddress() {return m_diagCommands["SH"]+m_diagCommands["SL"];}

  /**
   * @brief Get the Node Identifier parameter for the connected Xbee
   * @return std::string the ASCII string node identifier of the Xbee
   *
   * The node identifier is a user configurable ASCII string used to identify
   * a particular xbee node in a more user-firendly manner than serial number
   */
  std::string getNodeIdentifier() {return m_diagCommands["NI"];}

  /**
   * @brief Send an arbitrary data packet up to 72 bytes over RF
   * @param message data that should be sent over the network
   * @param destAddress the address of the Xbee the message should be sent to
   * @param networkAddress the network address that the destination node is on
   * @return bool whether the message was sent out or not
   *
   * Send message over RF using the Transmit Request Xbee API command. If
   * networkAddress is not specified, it defaults to any. If no destination
   * address is specified, the message is sent out as a broadcast, which is
   * then received by all Xbees in the network.
   */
  bool sendTransmitPacket(const std::string &message,
                          const std::string &destAddress = "000000000000FFFF",
                          const std::string &networkAddress = "FFFE");
 
  bool sendTransmitPacket(const std::vector<unsigned char> &message,
                          const std::string &destAddress = "000000000000FFFF",
                          const std::string &networkAddress = "FFFE");

  /**
   * @brief Prints the char buffer as hex data to std::out
   * @param message the message to be printed
   * @param size the size of the message to be printed
   */
  void printMessage(const char* message, int size);

  /**
   * @brief Converts raw hex data to a more friendly std::string (ASCII
   *        character) representation
   * @param input std::vector<char> of characters to be converted
   * @return std::string converted data
   * @note this function does the same thing as hexToString below, but takes different input
   *
   * A vector with character values [0x00, 0x05, 0xFF] would be converted to:
   * "0005FF"
   */
  std::string hexCharToString(const std::vector<signed char>& input);
  
/**
   * @brief Converts a string of hex data in ASCII format to its actual hex values
   * @param input string to be converted
   * @return std::vector<char> converted data
   * @note this is similar to the below function stringToHex
   *
   * A string "0005FF" would be converted to the character sequence:
   * [0x00, 0x05, 0xFF]
   */
  std::vector<char> stringToHexChar(const std::string& input);

 private:
  ros::NodeHandle m_nh; ///< local copy of the nodeHandle for timer starting
  char m_frameID; ///< default frameID used in Xbee messages if none specified
  ///< Set of Xbee AT commands to request periodically for diagnostic info
  std::map<std::string, std::string> m_diagCommands;
  /** Map of functions to call within XbeeInterface based on what type of
   *  message is received from the Xbee.
   */
  std::map<char, boost::function
      <bool (XbeeInterface*, const std::string& message)> > m_apiFrameFunctions;

  ///< Complete messages from Xbee that have been received, but not yet processed
  std::vector<std::string> m_unparsedMessages;
  ros::Timer m_diagTimer; ///< timer to send out diagnostic info
  ros::Timer m_diagInfoTimer; ///< timer to request diagnostic info from Xbee
  int m_bytesReceived;
  int m_bytesTransmitted;

  /**
   * @brief Callback triggered by SerialInterfaceThreaded when USB data is available
   */
  void xbeeDataCallback();

  /**
   * @brief Sends an AT command to the Xbee in api form
   * @param command the AT command to send to the Xbee
   * @param param value to set the param to, if setting (not needed to query)
   * @return bool whether the packet was successfully send to the Xbee
   *
   * This same function is used to query and set params on the Xbee. The only
   * difference is that to set a parameter, specify the value to set it to.
   */
  bool sendCommandPacket(const std::string& command,
                         const std::string &param = "");

  /**
   * @brief Processes an AT command response received from Xbee
   * @return bool whether the message was processed without errors
   *
   */
  bool processATCommandResponse(const std::string& command);

  /**
   * @brief Processes a remote AT command response received from Xbee
   * @return bool whether the message was processed without errors
   *
   */
  bool processRemoteATCommandResponse(const std::string& command);

  /**
   * @brief Processes a transmit status response received from Xbee
   * @return bool whether the message was processed without errors
   *
   */
  bool processTransmitStatus(const std::string& command);

  /**
   * @brief Processes a receive packet response received from Xbee
   * @return bool whether the message was processed without errors
   *
   */
  bool processReceivePacket(const std::string& message);

  /**
   * @brief Computes checksum of Xbee message in std::string format
   * @param msg the complete message to compute the checksum over
   * @return unsigned char the checksum value
   *
   */
  unsigned char computeChecksum(const std::string& msg);

  /**
   * @brief Computes checksum of Xbee message in char buffer format
   * @param msg the complete message to compute the checksum over
   * @return unsigned char the checksum value
   *
   * Computes checksum as per Xbee API specification by summing all data frame
   * bytes, the subtracting the least significant byte of the sum from 0xFF
   */
  unsigned char computeChecksum(const unsigned char* msg);

  /**
   * @brief Update diagnostic information for this node
   * @param time the timer event info
   */
  void diagUpdate(const ros::TimerEvent& time);

  /**
   * @brief Request diagnostic AT commands from Xbee
   * @param time the timer event info
   */
  void diagInfoRequest(const ros::TimerEvent& time);

  /**
   * @brief Converts raw hex data to a more friendly std::string (ASCII
   *        character) representation
   * @param input string to be converted
   * @return std::string converted data
   * @note this was taken from http://stackoverflow.com/questions/3381614/c-convert-string-to-hexadecimal-and-vice-versa
   *
   * A string with character values [0x00, 0x05, 0xFF] would be converted to:
   * "0005FF"
   */
  std::string stringToHex(const std::string& input);

  /**
   * @brief Converts a string of hex data in ASCII format to its actual hex values
   * @param input string to be converted
   * @return std::string converted data
   * @note this was taken from http://stackoverflow.com/questions/3381614/c-convert-string-to-hexadecimal-and-vice-versa
   *
   * A string "0005FF" would be converted to the character sequence:
   * [0x00, 0x05, 0xFF]
   */
  std::string hexToString(const std::string& input);

  /**
   * @brief Prints the Xbee API packet as hex data to std::out
   * @param message the message to be printed
   */
  void printMessage(const std::string& message);
};

#endif //XBEE_INTERFACE