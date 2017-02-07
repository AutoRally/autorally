/* API MODE STUFF
FRAME:
	0x7E | MSB|LSB | API SPEC STRUCTURE = X | checksum

Modem Status 0x8A - Reset Status

AT Command 0x08 - Send AT Command. cmdData contains two command characters
	X=0x08 | Arbitrary Byte | command | param |

AT Command - Queue Parameter Value 0x09
	Same, but nothing is applied until AC( apply changes) is issued

AT Command Response 0x88
	Response from module. X = 0x88 | CommandType | 0...3 (OK, Err, bad cmd, bad prm)| option value

Remote Command Request 0x17
	Same, but remote to another Xbee

Remote Command Response 0x97
	Same, but remote to another Xbee

Transmit Request
 0x10
	Here is what we want! Used to send messages to other xbees
	X= 0x10 | Arbitrary Byte | 64 bit address (0x000000000000FFFF for all) | 16 bit address| 0 | 0 | 54 byte message


Explicit Addressing Command Frame 0x11
	More specific way of sending transmissions. Probably dont need

Transmit Status  0x8B
	Auto response after transmission

Receive Packet (AO=0) 0x90
	What UART spits out when a packet is recieved
	X= 0x90|64 bit sender|16 bit sender | 0x01,0x02 (options) | data

Explicit Rx Indicator (AO=1) 0x91
	Explicit setting. Probably will not need
*/
#include "XbeeInterface.h"
#include <sstream>
#include <numeric>

XbeeInterface::XbeeInterface(ros::NodeHandle &nh, const std::string& port):
  m_nh(nh),
  m_bytesReceived(0),
  m_bytesTransmitted(0),
  m_atResponseFailures(0)
{
  //Create function pointers to call based on what type of message is
  m_apiFrameFunctions[(char)0x88] = &XbeeInterface::processATCommandResponse;
  m_apiFrameFunctions[(char)0x97] = &XbeeInterface::processRemoteATCommandResponse;
  m_apiFrameFunctions[(char)0x8B] = &XbeeInterface::processTransmitStatus;
  m_apiFrameFunctions[(char)0x90] = &XbeeInterface::processReceivePacket;
  
  std::string frameID;
  std::string diagInfo;
  std::string nName = ros::this_node::getName();
  if(!nh.getParam(nName+"/frameID", frameID) ||
     !nh.getParam(nName+"/diagnosticInfo", diagInfo) )
  {
    ROS_ERROR("Could not get all xbee parameters for %s", nName.c_str());
  }

  m_frameID = hexToString(&frameID[0])[0];
  
  //create vector of all periodically update diagnostic info
  size_t space;
  while( (space = diagInfo.find_first_of(" ")) != std::string::npos)
  {
    m_diagCommands[diagInfo.substr(0, space)] = "-";
    diagInfo.erase(0, space+1);
  }
  m_diagCommands[diagInfo.substr(0, space)] = "-";

  m_port.init(nh, ros::this_node::getName(), "", "Xbee Pro 900", port, true);
  m_port.registerDataCallback(
                    boost::bind(&XbeeInterface::xbeeDataCallback, this));

  if(!m_port.connected())
  {
    ROS_ERROR("The serial port isn't open, stuff might not work");
    m_port.diag_error("The serial port isn't open, stuff might not work");
  }
  
  m_diagTimer = nh.createTimer(ros::Duration(1.0),
	                             &XbeeInterface::diagUpdate,
	                             this);
  m_diagInfoTimer = nh.createTimer(ros::Duration(5.0),
                                   &XbeeInterface::diagInfoRequest,
	                                 this);
}

XbeeInterface::~XbeeInterface()
{}

void XbeeInterface::registerReceiveMessageCallback(
                    boost::function<void(const std::string &sender,
                                         const std::string &networkAddress,
                                         const std::string &data,
                                         const bool broadcast)> callback)
{
  m_receiveMessageCallback = callback;
}

bool XbeeInterface::sendCommandPacket(const std::string &command, const std::string &param)
{
  
  int length = 0;
  unsigned char apiFrame[9];
  if(param.size() == 0 && command.size() == 2)
  {
    length = 8;
    apiFrame[2]=0x04;
  } else if(param.size() == 1 && command.size() == 2)
  {
    length = 9;
    apiFrame[2]=0x05;
    apiFrame[7]=param[0];
  } else
  {
    ROS_ERROR("Xbee: could not generate command packet, command:%s param:%s",
              command.c_str(),
              param.c_str());
    return false;
  }

	apiFrame[0]=0x7E;
	apiFrame[1]=0x00;
	apiFrame[3]=0x08; //AT command
	apiFrame[4]=m_frameID; //frame id
	apiFrame[5]=command[0];//0x41;
	apiFrame[6]=command[1];//0x50;

  apiFrame[length-1] = computeChecksum(apiFrame);
  return (m_port.writePort(apiFrame, length) != -1);
}

void XbeeInterface::xbeeDataCallback()
{
  m_port.lock();
  size_t startPosition = m_port.m_data.find("~");
  if(startPosition == 0 && m_port.m_data.length() >= 3)
	{
	  int size = (m_port.m_data[1]<<8)|m_port.m_data[2];
	  size_t totalSize = size+4;

	  if(m_port.m_data.length() >= totalSize)
	  {
      std::string msgToAdd = m_port.m_data.substr(3, size);

	    unsigned char checksum = computeChecksum(m_port.m_data.substr(0, totalSize));
      if(checksum == (unsigned char)(m_port.m_data[totalSize-1]&0xFF))
      {
        //call appropritate callback
        if(m_apiFrameFunctions.find(msgToAdd[0]&0xFF) != m_apiFrameFunctions.end())
        {
   	      if(!m_apiFrameFunctions[msgToAdd[0]&0xFF](this, msgToAdd))
   	      {
            m_port.diag_error("Xbee: error processing message");
            ROS_ERROR("Xbee: error processing message:%s", stringToHex(msgToAdd).c_str());
          }
        } else
        {
          m_port.diag_error("Unrecognized xbee message type");
          ROS_ERROR_STREAM("Unrecognized xbee message type:" << (int)(msgToAdd[0]&0xFF));
        }
   	    //append incoming message to vector, stripped of
        //start delimiter, length, checksum
   	    m_port.m_data.erase(0, totalSize);
        m_port.unlock();
   	    return;
   	  } else
   	  {
   	    m_port.diag_error("Xbee: computed checksum does not match message checksum");
   	    ROS_ERROR("Xbee: computed checksum %x does not match message checksum %x",
   	              checksum,
   	              m_port.m_data[totalSize-1]&0xFF);
   	  }
   	  m_port.m_data.erase(0, totalSize);

	  }

	} else if(startPosition != std::string::npos)
	{
	  m_port.m_data.erase(0, startPosition);
	}
  m_port.unlock();
}

bool XbeeInterface::processATCommandResponse(const std::string &message)
{
  if( message[0] == (char)0x88 && //AT command reply byte
      message[1] == m_frameID)    //frameid
  {
    if(message[4] != (char)0x00)//Check no error code
    {
      //sometimes these AT responses fail (specifically NI request) for some reason,
      //and I can't figure out why but they are just debug info, so I send the message
      // to debug insted of raising an error
      ROS_DEBUG_STREAM("Xbee AT response " << message[2] << message[3] << " error code:" <<
                       message[4] );
      //m_port.diag_error("Xbee AT response " + message.substr(2,2) +
      //                  " error code:" + std::to_string(message[4]));
      ++m_atResponseFailures;
    } else
    {
      std::string command = message.substr(2, 2);
      std::string value = message.substr(5, message.length()-5);

      std::map<std::string, std::string>::iterator mapIt;
      if( (mapIt = m_diagCommands.find(command)) != m_diagCommands.end())
      {
        if(command == "NI")
        {
          mapIt->second = value;
        } else
        {
          mapIt->second = stringToHex(value);
        }
      } else
      {
        m_port.diag_warn("Received unknown diagnostic command: " + command);
      }
    }
    return true;
  }
  return false;
}

bool XbeeInterface::processRemoteATCommandResponse(const std::string &message)
{
  if( message[0] == (char)0x97 && //AT command reply byte
      message[1] == m_frameID)    //frameid
  {
    if(message[15] != (char)0x00)//Check no error code
    {
      ROS_ERROR("Xbee: %s error code:%d", message.c_str(), message[4] );
      m_port.diag_error(message + " error code:" + message[4]);
    } else
    {
      std::string responder = message.substr(3, 8);
      std::string responderNetwork = message.substr(11, 2);
      std::string command = message.substr(13, 2);
      if(message.size() > 16)
      {
        std::string value = message.substr(15, message.length()-16);
      }
      return true;
    }
    
  }
  return false;

}

bool XbeeInterface::processTransmitStatus(const std::string &message)
{
  //check the message is an AT command reply for the desired command
  if( message[0] == (char)0x8B && //AT command reply byte
      message[1] == m_frameID ) //frameid
  {
    ROS_DEBUG_STREAM("Xbee: transmit status:" << message << ":" << stringToHex(message));
    if(message[5] != (char)0x00)
    {
      ROS_WARN("Xbee: transmit error (likely wrong address):%x", message[5]);
      m_port.diag_warn("Xbee: transmit error (likely wrong address):" + message[5]);
    }
    return true;
  }
  return false;
}

bool XbeeInterface::processReceivePacket(const std::string &message)
{
  ROS_DEBUG_STREAM("Xbee: received Packet:" << stringToHex(message));
  m_bytesReceived += message.size()+6;
  if(message[0] == (char)0x90)
  {
    std::string sender = message.substr(1, 8);
    std::string responderNetwork = message.substr(9, 2);
    std::string packetType = message.substr(11, 1);
    std::string data = message.substr(12);

    if(m_receiveMessageCallback)
    {
      m_receiveMessageCallback(stringToHex(sender),
                               responderNetwork,
                               data,
                               (packetType[0]==(char)0x02));
      return true;
    }
  }
  return false;
}

bool XbeeInterface::sendTransmitPacket(const std::string &message,
                                       const std::string &destAddress,
                                       const std::string &networkAddress)
{
  if(message.size() > 72)
  {
    ROS_ERROR("Xbee: Transmit packet length %ld too long (max 72)", message.size());
  	return false;
  }
  if(destAddress.size() != 16)
  {
    ROS_ERROR("Xbee: Destination address %s length %ld != 16",
              destAddress.c_str(),
              destAddress.size());
  	return false;
  }
  if(networkAddress.size() != 4)
  {
    ROS_ERROR("Xbee: Network address %s length %ld != 4",
              networkAddress.c_str(),
              networkAddress.size());
  	return false;
  }

  int payloadSize = message.size() + 14;
  int totalSize = payloadSize + 4;
  unsigned char buff[totalSize];

  buff[0] = 0x7E; //frame delimiter
  buff[1]=0x00;
  buff[2]=static_cast<unsigned char>(payloadSize);
  buff[3]=0x10; //Transmit command
  buff[4]=m_frameID; //Random selection

  //5-12 64bit address
  std::string converted = hexToString(destAddress);
  for(size_t i = 0; i < converted.size(); i++)
  {
    buff[5+i] = converted[i];
  }
  converted = hexToString(networkAddress);
  buff[13] = converted[0];
  buff[14] = converted[1];

  buff[15]=0x00; //broadcast radius set to 0
  buff[16] = 0x00; //multicast set to 0 (unicast)

  for(size_t i = 0; i < message.size(); i++)
  {
    buff[17+i] = message[i];
  }

  buff[totalSize-1] = computeChecksum(buff);

  ROS_DEBUG_STREAM("Xbee: Sending:" << message << " with checksum:" << (int)buff[totalSize-1] << " to:" << destAddress <<
                   " in network:" << networkAddress);

  if(m_port.writePort(buff, totalSize) >= 0)
	{
    m_bytesTransmitted += totalSize;
  	return true;
  }

  ROS_WARN("Xbee: Writing Message to port failed");
  return false;
}

bool XbeeInterface::sendTransmitPacket(const std::vector<unsigned char> &message,
                                       const std::string &destAddress,
                                       const std::string &networkAddress)
{
  if(message.size() > 72)
  {
    ROS_ERROR("Xbee: Transmit packet length %ld too long (max 72)", message.size());
    return false;
  }
  if(destAddress.size() != 16)
  {
    ROS_ERROR("Xbee: Destination address %s length %ld != 16",
              destAddress.c_str(),
              destAddress.size());
    return false;
  }
  if(networkAddress.size() != 4)
  {
    ROS_ERROR("Xbee: Network address %s length %ld != 4",
              networkAddress.c_str(),
              networkAddress.size());
    return false;
  }

  int payloadSize = message.size() + 14;
  int totalSize = payloadSize + 4;
  unsigned char buff[totalSize];

  buff[0] = 0x7E; //frame delimiter
  buff[1]=0x00;
  buff[2]=static_cast<unsigned char>(payloadSize);
  buff[3]=0x10; //Transmit command
  buff[4]=m_frameID; //Random selection

  //5-12 64bit address
  std::string converted = hexToString(destAddress);
  for(u_int i = 0; i < converted.size(); i++)
  {
    buff[5+i] = converted[i];
  }
  converted = hexToString(networkAddress);
  buff[13] = converted[0];
  buff[14] = converted[1];

  buff[15]=0x00; //broadcast radius set to 0
  buff[16] = 0x00; //multicast set to 0 (unicast)

  std::memcpy(&buff[17], &message.front(), message.size());

  buff[totalSize-1] = computeChecksum(buff);

  ROS_DEBUG_STREAM("Xbee: Sending " << message.size() << " bytes to:" << destAddress <<
                   " in network:" << networkAddress);

  if(m_port.writePort(buff, totalSize) >= 0)
  {
    m_bytesTransmitted += totalSize;
    return true;
  }

  ROS_WARN("Xbee: Writing Message to port failed");
  return false;
}

unsigned char XbeeInterface::computeChecksum(const std::string &msg)
{
  return computeChecksum(reinterpret_cast<const unsigned char*>(msg.data()));
}

unsigned char XbeeInterface::computeChecksum(const unsigned char *msg)
{
  int len = (msg[1]<<8)|msg[2];
  int totalLen = len + 4;
  int sum = 0;

  for(int i = 3; i <= totalLen-2; i++)
  {
    sum += msg[i]&0xFF;
  }
  return 0xFF-(sum&0xFF);
}

void XbeeInterface::diagInfoRequest(const ros::TimerEvent& /*time*/)
{
  std::map<std::string, std::string>::const_iterator mapIt;
  for(mapIt = m_diagCommands.begin(); mapIt != m_diagCommands.end(); mapIt++)
  {
    sendCommandPacket(mapIt->first);
  }
}

void XbeeInterface::diagUpdate(const ros::TimerEvent& /*time*/)
{
  m_port.diag("AT Response Failures", std::to_string(m_atResponseFailures));
  m_port.diag("Transmit bandwidth B/s", std::to_string(m_bytesTransmitted));
  m_port.diag("Receive bandwidth B/s", std::to_string(m_bytesReceived));
  m_bytesTransmitted = 0;
  m_bytesReceived = 0;

  std::map<std::string, std::string>::const_iterator mapIt;
  for(mapIt = m_diagCommands.begin(); mapIt != m_diagCommands.end(); mapIt++)
  {
    if(mapIt->first == "TR")
    {
      m_port.diag("TR - transmission errors", mapIt->second);
    } else if(mapIt->first == "ER")
    {
      m_port.diag("ER - RF receive packet errors", mapIt->second);
    } else
    {
      m_port.diag(mapIt->first, mapIt->second);
    }
  }
}

std::string XbeeInterface::stringToHex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();

    std::string output;
    output.reserve(2 * len);
    for (size_t i = 0; i < len; ++i)
    {
        const unsigned char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}

std::string XbeeInterface::hexToString(const std::string& input)
{
  static const char* const lut = "0123456789ABCDEF";
  size_t len = input.length();

  std::string output;
  output.reserve(len / 2);
  for (size_t i = 0; i < len; i += 2)
  {
      char a = input[i];
      const char* p = std::lower_bound(lut, lut + 16, a);
      if (*p != a)
      {
        ROS_ERROR("Xbee: hexToString Invalid hex digit:%x", a);
        m_port.diag_error("Xbee: hexToString Invalid hex digit:" + a);
      }

      char b = input[i + 1];
      const char* q = std::lower_bound(lut, lut + 16, b);
      if (*q != b)
      {
        ROS_ERROR("Xbee: hexToString Invalid hex digit:%x", b);
        m_port.diag_error("Xbee:  hexToString Invalid hex digit:" + b);
      }
      output.push_back(((p - lut) << 4) | (q - lut));
  }
  return output;
}

std::string XbeeInterface::hexCharToString(const std::vector<signed char>& input)
{
  static const char* const lut = "0123456789ABCDEF";
  int length = 2*input.size();
  std::string toReturn(length, 0x00); //allocate space in string
  
  for(size_t i  = 0; i < input.size(); i++) 
  {
    toReturn[2*i+1] = lut[(input[i]&0x0F)];
    toReturn[2*i] = lut[((input[i]>>4)&0x0F)];
    //ROS_ERROR_STREAM("Converted:" << (int)input[i] << " to:" << toReturn.substr(2*i,2));
  }

  return toReturn;
}

std::vector<char> XbeeInterface::stringToHexChar(const std::string& input)
{
  std::vector<char> toReturn(input.size()/2);
  if(input.size()%2 == 0)
  {
    for(size_t i  = 0; i < input.size()/2; i++) 
    {
      std::istringstream buffer(input.substr(i*2,2));
      int p;
      buffer >> std::hex >> p;
      toReturn[i] = (signed char)p;
    }
  } else
  {
    ROS_ERROR_STREAM("Xbee: stringToHexChar input string odd length, can't convert to hex");
  }
  return toReturn;
}

void XbeeInterface::printMessage(const std::string &message)
{
  printMessage(message.c_str(), message.size());
}

void XbeeInterface::printMessage(const char* message, int size)
{
  for (int i = 0; i < size; i++)
  {
    	printf("%x ", message[i]&0xFF );
  }
  printf("\n");
}
