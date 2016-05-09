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
 * @file SerialCommon.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date July 22, 2013
 * @copyright 2013 Georgia Institute of Technology
 * @brief SerialCommon class implementation
 *
 ***********************************************/
#include <autorally_core/SerialCommon.h>

#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

SerialCommon::SerialCommon() :
  m_fd(-1),
  m_settingError("")
{}

SerialCommon::SerialCommon(const std::string& portHandle,
                           const std::string& hardwareID,
                           const std::string& portPath) :
  m_fd(-1),
  m_settingError("")
{
  init(portHandle, hardwareID, portPath);
}

SerialCommon::~SerialCommon()
{
  if(m_fd > 0)
  {
    std::cout << "shutting down " << m_fd << " " << close(m_fd) << std::endl;
    m_fd = 0;
  }
}

void SerialCommon::init(const std::string& otherInfo,
                        const std::string& hardwareID,
                        const std::string& portPath)
{
  //init in Diagnostics
  std::cout << portPath << " initializing" << std::endl;
  Diagnostics::init(otherInfo, hardwareID, portPath);
}


bool SerialCommon::connect(const std::string& port,
                           const int baud,
                           const std::string parity,
                           const int stopBits,
                           const int dataBits,
                           const bool hardwareFlow,
                           const bool softwareFlow)
{
  std::cout << port << " connecting" << std::endl;
  if (m_fd != -1 )
  {
    ROS_ERROR("Already connected to %s", port.c_str());
    return false;
  }

  //m_fd = open(port.c_str(), O_WRONLY | O_NOCTTY | O_NDELAY);
  //ioctl(m_fd, USBDEVFS_RESET, 0);
	//close(m_fd);
  //std::cout << port << " reset" << std::endl;

  m_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  
  //std::cout << port << " open attempted" << std::endl;

  if (m_fd == -1 )
  {
    perror("Unable to open serial port");
    ROS_ERROR("Unable to open %s", port.c_str());
    diag_error("Unable to open " + port);
    return false;
  } else
  {
    struct termios port_settings;    // structure to store the port settings in
    memset(&port_settings, 0, sizeof(port_settings));
    //Get the current options for the port...
    if(tcgetattr (m_fd, &port_settings) != 0)
    {
      ROS_ERROR("SerialSensorInterface: error %d from tcgetattr", errno);
      diag_error("SerialSensorInterface: error from tcgetattr");
      return false;
    }
    //tcgetattr(m_fd, &m_old_port_settings);
    //fcntl(m_fd, F_SETFL, 0);
    //fcntl(m_fd, F_SETFL, FNDELAY);
    //std::cout << port << " got attributes" << std::endl;
    speed_t b;
    switch(baud)
    {
      case 4800:
        b = B4800;
        break;
      case 9600:
        b = B9600;
        break;
      case 19200:
        b = B19200;
        break;
      case 38400:
        b = B38400;
        break;
      case 57600:
        b = B57600;
        break;
      case 115200:
        b = B115200;
        break;
      case 230400:
        b = B230400;
        break;
      default:
        ROS_ERROR("Unsupported baud:%d", baud);
        m_settingError = "Unsupported baud";
        return false;
    }

    // set baud rates
    if(cfsetispeed(&port_settings, b) != 0 ||
       cfsetospeed(&port_settings, b) != 0)
    {
      ROS_ERROR("Could not set baud:%d", baud);
      m_settingError = "Could not set baud";
    }

    // set 8N1
    if(parity == "none")
    {
      port_settings.c_cflag &= ~PARENB; // no parity bit
      port_settings.c_cflag &= ~PARODD; // even parity
    } else if(parity == "even")
    {
      port_settings.c_cflag |= PARENB; // enable parity
      port_settings.c_cflag &= ~PARODD; // even parity
    } else if(parity == "odd")
    {
      port_settings.c_cflag |= PARENB; // enable parity
      port_settings.c_cflag |= PARODD; // odd parity
    } else
    {
      ROS_ERROR("Unsupported parity:%s", parity.c_str());
      m_settingError = "Unsupported parity:"+parity;
      return false;
    }


    if(stopBits == 1)
    {
      port_settings.c_cflag &= ~CSTOPB; // only one stop bit
    } else if(stopBits == 2)
    {
      port_settings.c_cflag |= CSTOPB; // two stop bits
    } else
    {
      ROS_ERROR("Unsupported stopBits:%d", stopBits);
      m_settingError = "Unsupported stopBits";
      return false;
    }

    //port_settings.c_cflag |= ~CSIZE;  // clear data bit number
    switch(dataBits)
    {
      case 5:
        port_settings.c_cflag |= CS5;     // set 5 data bits
        break;
      case 6:
        port_settings.c_cflag |= CS6;     // set 6 data bits
        break;
      case 7:
        port_settings.c_cflag |= CS7;     // set 7 data bits
        break;
      case 8:
        port_settings.c_cflag |= CS8;     // set 8 data bits
        break;
      default:
        ROS_ERROR("Unsupported dataBits:%d", dataBits);
        m_settingError = "Unsupported dataBits";
        return false;
    }

    // hardware flow control
    if(hardwareFlow)
    {
      port_settings.c_cflag |= CRTSCTS;
    } else
    {
      port_settings.c_cflag &= ~CRTSCTS;
    }

    // software flow control
    if(softwareFlow)
    {
      port_settings.c_iflag |= (IXON | IXOFF | IXANY);
    } else
    {
      port_settings.c_iflag &= ~(IXON | IXOFF | ~IXANY); //this part is important
    }

    // enable reading and ignore control lines
    port_settings.c_cflag |= CREAD | CLOCAL;
    //set raw input mode (not canonical)
    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    port_settings.c_oflag &= ~OPOST; //  disable pre-processing of input data
                                         
    port_settings.c_cc[VMIN]  = 1; // wait for at least 1 character (read doesn't block)
    port_settings.c_cc[VTIME] = 0; // 0.5 seconds read timeout

    //std::cout << baud << std::endl;

    //std::cout << port << " setting attributes" << std::endl;
    // apply the settings to the port
    tcflush(m_fd, TCIFLUSH);
    
    if(tcsetattr(m_fd, TCSANOW, &port_settings) == 0)
    {
      usleep(200000); //  wait for connection to be negotiated
                      //  found this length through experimentation
      ROS_INFO("%s port configuration complete", port.c_str());
      return true;
    }
  }
  ROS_ERROR("Could not configure %s", port.c_str());
  m_settingError = "Could not set serial port attributes";
  return false;
}

int SerialCommon::writePort(const std::string data) const
{
  int n;
  n = write(m_fd, (char*)data.c_str(), data.length());
  //std::cout<<(char*)data.c_str();
  if (n < 0)
  {
    //ROS_INFO("%s","write() failed!");
    return -1;
  }
  return n;
}

int SerialCommon::writePort(const unsigned char* data, unsigned int length) const
{
  int n;
  n=write(m_fd, data, length);
  if(n < 0)
  {
    //ROS_INFO("%s","write() failed!");
    return -1;
  }
  return n;
}
