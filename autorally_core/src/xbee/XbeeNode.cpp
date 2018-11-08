/**********************************************
 * @file XbeeNode.cpp
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date May 17, 2013
 * @copyright 2012 Georgia Institute of Technology
 * @brief
 *
 * @details This file contains the XbeeNode class implementation
 ***********************************************/

#include "XbeeNode.h"
#include <boost/bind.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "xbeeNode");
  ros::NodeHandle nh;

  std::string xbeePort;

  if(!nh.getParam("xbeeNode/port", xbeePort))
  {
    ROS_ERROR("Could not get xbeeNode parameters");
    return -1;
  }

  XbeeNode coordinator(nh, xbeePort);

  ros::spin();
  return 0;
}

XbeeNode::XbeeNode(ros::NodeHandle &nh, const std::string& port):
  m_xbee(nh, port),
  m_nh(nh),
  m_lastrunstop(0),
  m_lastTargetedrunstop(0),
  m_coordinatorAddress("")
{
  std::string nName = ros::this_node::getName();
  int transmitPositionRate;
  if(!nh.getParam(nName+"/transmitPositionRate", transmitPositionRate))
  {
    ROS_ERROR("Could not get all xbee parameters for %s", nName.c_str());
  }

  //until a runstop is received from rf, it will publish runstops
  //with this name
  m_runstop.sender = "XbeeNode";
  m_runstop.motionEnabled = false;

  //fill out space for incoming RTCM3 message sequences
  for(unsigned char i = 'A'; i <= 'Z'; i++)
  {
    m_correctionPackets[i] = Rtcm3Packets();
  }

  m_runstopPublisher = nh.advertise<autorally_msgs::runstop>
                                      ("runstop", 1);
  m_gpsRTCM3Publisher = nh.advertise<std_msgs::ByteMultiArray>
                                      ("gpsBaseRTCM3", 3);

  m_xbee.registerReceiveMessageCallback(boost::bind(&XbeeNode::processXbeeMessage, this, _1, _2, _3, _4) );

  m_xbeeHeartbeatTimer = nh.createTimer(ros::Duration(0.5),
                                        &XbeeNode::xbeeHeartbeatState,
                                        this);
  m_hiTimer = m_nh.createTimer(ros::Duration(1.0),
                               &XbeeNode::sendHi,
                               this);

  if(transmitPositionRate > 0)
  {
    m_transmitPositionTimer = m_nh.createTimer(ros::Duration(1.0/transmitPositionRate),
                               &XbeeNode::transmitPosition,
                               this);
    m_poseSubscriber = nh.subscribe("pose_estimate", 1,
                                  &XbeeNode::odomCallback,
                                  this);
  }
}

XbeeNode::~XbeeNode()
{}

void XbeeNode::sendHi(const ros::TimerEvent& /*time*/)
{
  if(m_coordinatorAddress.size() == 0 || m_coordinatorAddress == "--")
  {

    //dont send until we read our own node identifier
    if(m_xbee.getNodeIdentifier() != "-")
    {
      std::string sendData = "HI " + m_xbee.getNodeIdentifier();
      //std::cout << "SENDING:" << sendData << std::endl;

      m_xbee.sendTransmitPacket(sendData);
    }
  } else
  {
    //once we know the address of the coordinator, start sending periodic info to it
    m_hiTimer.stop();
    m_stateTimer = m_nh.createTimer(ros::Duration(1.0),
                                &XbeeNode::sendState,
                                this);
  }
}

void XbeeNode::sendState(const ros::TimerEvent& /*time*/)
{
  std::string sendData = "ST " + m_xbee.getNodeIdentifier();
  m_xbee.sendTransmitPacket(sendData);
}

void XbeeNode::xbeeHeartbeatState(const ros::TimerEvent& /*time*/)
{
  if( (ros::Time::now()-m_lastrunstop).toSec() > 1.0)
  {
    m_runstop.sender = "XbeeNode(No runstop data from RF)";
    m_runstop.motionEnabled = false;
    m_runstop.header.stamp = ros::Time::now();
    m_runstopPublisher.publish(m_runstop);
  }
}

void XbeeNode::processXbeeMessage(const std::string& sender,
                                  const std::string& /*networkAddress*/,
                                  const std::string& data,
                                  const bool broadcast)
{
  std::stringstream ss(data);
  std::string msg;
  ss >> msg;
  msg = data.substr(0,2);
  //std::cout << msg << std::endl;
  //ROS_INFO_STREAM("Xbee NODE receive:" << data << " -" << msg << "-" << m_coordinatorAddress);
  if(msg == "AK")
  {
    ss >> m_coordinatorAddress;
  } else if (msg == "OD")
  {
    //ROS_ERROR("XbeeNode: Received pose estimate");
    if(data.length() == 72)
    {
      processXbeeOdom(data, sender);
    }
    else
      ROS_ERROR("XbeeNode: Received incorrect length(%d) odom message \"%s\"", (int)data.length(),data.c_str());
  } else if(msg == "RS")
  {
    //std::cout << "RS:" << ss << std::endl;
    m_lastrunstop = ros::Time::now();
    if(!broadcast)
    {
      m_lastTargetedrunstop = m_lastrunstop;
    }

    //if I'm receiving targeted xbee runstop messages, ignore broadcast xbee
    //runstop messages
    std::string motionEnabled = "0";
    double timeDiff = (m_lastrunstop-m_lastTargetedrunstop).toSec();
    if( timeDiff <= 1.0 && !broadcast)
    {
      ss >> m_runstop.sender >> motionEnabled;
      try
      {
        m_runstop.motionEnabled = boost::lexical_cast<int>(motionEnabled);
      }
      catch(boost::bad_lexical_cast &e)
      {
        ROS_ERROR_STREAM("XbeeNode: bad motionEnabled lexical cast from:" << motionEnabled);
        m_runstop.motionEnabled = 0;
      }
      m_runstop.header.stamp = m_lastTargetedrunstop;
      m_runstopPublisher.publish(m_runstop);
    } else if(broadcast)
    {
      ss >> m_runstop.sender >> motionEnabled;
      try
      {
        m_runstop.motionEnabled = boost::lexical_cast<bool>(motionEnabled);
      }
      catch(boost::bad_lexical_cast &e)
      {
        ROS_ERROR_STREAM("XbeeNode: bad broadcast motionEnabled lexical cast from:" << motionEnabled);
        m_runstop.motionEnabled = 0;
      }
m_runstop.header.stamp = m_lastrunstop;
      m_runstopPublisher.publish(m_runstop);
    } else
    {
      ROS_ERROR("XbeeNode: something wrong with runstop received over xbee");
    }

  } else if(msg == "GC")
  {
    try
    {
      char seq = data[2];
      int seqLen = boost::lexical_cast<int>(data[3]);
      size_t packetNum = boost::lexical_cast<int>(data[4]);

      ROS_DEBUG_STREAM("Xbee sequence: " << seq << " seq length: " << seqLen <<
                       " packet num: " << packetNum);

      //just received first packet of new RTCM3 message
      if(ros::Time::now()-m_correctionPackets[seq].time > ros::Duration(2.0))
      {
        m_correctionPackets[seq].reset(seqLen);
      }

      //check to make sure there is space in the packet vector (+1 since packets #s start at 1)
      if(packetNum < m_correctionPackets[seq].packets.size())
      {
        m_correctionPackets[seq].packets[packetNum] = data.substr(5);
        ++m_correctionPackets[seq].count;
      } else
      {
        ROS_ERROR_STREAM("Xbee RTCM packet number " << packetNum << " larger than packet vector length " <<
                         m_correctionPackets[seq].packets.size());
      }

      if(m_correctionPackets[seq].count == m_correctionPackets[seq].packets.size()-1)
      {
        ROS_DEBUG_STREAM("Xbee publishing RTCM3 msg seq " << seq);

        std::string gpsString;
        //actual RTCM payload data packets start at 1
        for(size_t i = 1; i < m_correctionPackets[seq].packets.size(); ++i)
        {
          gpsString += m_correctionPackets[seq].packets[i];
        }
        //clear message data immediately so it can't be accidentally used again
        m_correctionPackets[seq].packets.clear();
        m_correctionPackets[seq].count = 0;

        //allocate new message, fill it in
        std_msgs::ByteMultiArrayPtr m_gpsCorrection(new std_msgs::ByteMultiArray);

        m_gpsCorrection->layout.dim.push_back(std_msgs::MultiArrayDimension());
        m_gpsCorrection->layout.dim.front().label = m_msgLabel;

        for(size_t i = 0; i < gpsString.size(); i++)
        {
          m_gpsCorrection->data.push_back(gpsString[i]);
        }

        //publish correction into ros
        m_gpsCorrection->layout.dim.front().size = m_gpsCorrection->data.size();
        m_gpsCorrection->layout.dim.front().stride = m_gpsCorrection->data.size();
        m_gpsRTCM3Publisher.publish(m_gpsCorrection);
        m_xbee.m_port.tick("RTCM3 correction");

      }
    } catch(boost::bad_lexical_cast &e)
    {
      ROS_ERROR_STREAM("XbeeNode: caught bad lexical cast for GPS RTCM3 msgnum:" << data[4]);
    }
  } else if(msg == "HI" || msg == "ST")
  {} //These will be received but are only needed by coordinator
  else
  {
    ROS_ERROR("XbeeNode: Received unknown message \"%s\"", msg.c_str());
  }
}
void XbeeNode::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  m_odometry = *odom;
}

void XbeeNode::transmitPosition(const ros::TimerEvent& /*time*/)
{
  if(m_odometry.header.stamp > m_lastXbeeOdomTransmit)
  {
    char temp[16];
    std::string data;
    data += "OD ";
    sprintf(temp, "%5i", scaleAndClip(m_odometry.pose.pose.position.x, 500, 0.01));
    data += temp;
    sprintf(temp, "%5i", scaleAndClip(m_odometry.pose.pose.position.y, 500, 0.01));
    data += temp;
    sprintf(temp, "%5i", scaleAndClip(m_odometry.pose.pose.position.z, 500, 0.01));
    data += temp;
    sprintf(temp, "%4i", scaleAndClip(m_odometry.twist.twist.linear.x, 50, 0.01));
    data += temp;
    sprintf(temp, "%4i", scaleAndClip(m_odometry.twist.twist.linear.y, 50, 0.01));
    data += temp;
    sprintf(temp, "%4i", scaleAndClip(m_odometry.twist.twist.linear.z, 50, 0.01));
    data += temp;

    sprintf(temp, "%8i", scaleAndClip(m_odometry.pose.pose.orientation.x, 1, 0.0000001));
    data += temp;
    sprintf(temp, "%8i", scaleAndClip(m_odometry.pose.pose.orientation.y, 1, 0.0000001));
    data += temp;
    sprintf(temp, "%8i", scaleAndClip(m_odometry.pose.pose.orientation.z, 1, 0.0000001));
    data += temp;
    sprintf(temp, "%8i", scaleAndClip(m_odometry.pose.pose.orientation.w, 1, 0.0000001));
    data += temp;
    double timestamp = m_odometry.header.stamp.toSec();
    double truncatedTime =  timestamp - std::floor(timestamp / 10) * 10;
    sprintf(temp, "%10i", scaleAndClip(truncatedTime, 10, 0.00000001));
    data += temp;

    /*ROS_WARN("[%f %f %f][%f %f %f][%f %f %f %f][%f]",
             m_odometry.pose.pose.position.x,
             m_odometry.pose.pose.position.y,
             m_odometry.pose.pose.position.z,
             m_odometry.twist.twist.linear.x,
             m_odometry.twist.twist.linear.y,
             m_odometry.twist.twist.linear.z,
             m_odometry.pose.pose.orientation.x,
             m_odometry.pose.pose.orientation.y,
             m_odometry.pose.pose.orientation.z,
             m_odometry.pose.pose.orientation.w,
             m_odometry.header.stamp.toSec());*/

    //ROS_WARN_STREAM("Sending position:" << data);
    m_xbee.sendTransmitPacket(data);
    m_xbee.m_port.tick("pose_estimate broadcast");
    m_lastXbeeOdomTransmit = m_odometry.header.stamp;
  }
}


int XbeeNode::scaleAndClip(double number, double range, double resolution)
{
  double scaled = (number + range) / resolution;
  if (scaled >= ((2.0*range)/resolution))
    scaled = 2.0*range / resolution;
  if (scaled < 0)
    scaled = 0;
  return (int)scaled;
}

double XbeeNode::unscaleAndClip(int number, double range, double resolution)
{
  return ((double)number * resolution) - range;
}

void XbeeNode::processXbeeOdom(const std::string& message, const std::string& sender)
{
  //ROS_WARN_STREAM("Received:" << message << " from:" << sender);
  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
  ros::Time curTime = ros::Time::now(); 

  odom->pose.pose.position.x = unscaleAndClip(atoi(message.substr(3,5).c_str()), 500, 0.01);
  odom->pose.pose.position.y = unscaleAndClip(atoi(message.substr(8,5).c_str()), 500, 0.01);
  odom->pose.pose.position.z = unscaleAndClip(atoi(message.substr(13,5).c_str()), 500, 0.01);
  odom->twist.twist.linear.x = unscaleAndClip(atoi(message.substr(18,4).c_str()), 50, 0.01);
  odom->twist.twist.linear.y = unscaleAndClip(atoi(message.substr(22,4).c_str()), 50, 0.01);
  odom->twist.twist.linear.z = unscaleAndClip(atoi(message.substr(26,4).c_str()), 50, 0.01);
  odom->pose.pose.orientation.x = unscaleAndClip(atoi(message.substr(30,8).c_str()), 1, 0.0000001);
  odom->pose.pose.orientation.y = unscaleAndClip(atoi(message.substr(38,8).c_str()), 1, 0.0000001);
  odom->pose.pose.orientation.z = unscaleAndClip(atoi(message.substr(46,8).c_str()), 1, 0.0000001);
  odom->pose.pose.orientation.w = unscaleAndClip(atoi(message.substr(54,8).c_str()), 1, 0.0000001);
  
  // catch if the timestamp is acutally from the last tenths place
  // TODO magic number
  if(static_cast<int>(curTime.toSec()) % 10 < 3 && unscaleAndClip(atoi(message.substr(62,10).c_str()), 10, 0.00000001) > 7) {
  odom->header.stamp = ros::Time(unscaleAndClip(atoi(message.substr(62,10).c_str()), 10, 0.00000001) + (std::floor(curTime.toSec() / 10) * 10 - 10));
  } else {
    odom->header.stamp = ros::Time(unscaleAndClip(atoi(message.substr(62,10).c_str()), 10, 0.00000001) + std::floor(curTime.toSec() / 10) * 10);
  }
  
  std_msgs::Float64 timeDiff;
  timeDiff.data = curTime.toSec() - odom->header.stamp.toSec();
  
  /*ROS_WARN("[%f %f %f][%f %f %f][%f %f %f %f][%f]",
           odom->pose.pose.position.x,
           odom->pose.pose.position.y,
           odom->pose.pose.position.z,
           odom->twist.twist.linear.x,
           odom->twist.twist.linear.y,
           odom->twist.twist.linear.z,
           odom->pose.pose.orientation.x,
           odom->pose.pose.orientation.y,
           odom->pose.pose.orientation.z,
           odom->pose.pose.orientation.w,
           odom->header.stamp.toSec());*/


  if(!m_recOdomPublishers[sender])
  {
    ros::NodeHandle nh;
    m_recOdomPublishers[sender] = nh.advertise<nav_msgs::Odometry>
                                      ("/pose_estimate_"+sender, 1);
                                      
    m_timeDiffPublishers[sender] = nh.advertise<std_msgs::Float64>
                                      ("/pose_estimate_"+sender+"_time_diff", 1);

  }
  odom->child_frame_id = sender;
  odom->header.frame_id = "odom";
  m_xbee.m_port.tick("/pose_estimate_"+sender);
  m_recOdomPublishers[sender].publish(odom);
  m_timeDiffPublishers[sender].publish(timeDiff);
}
