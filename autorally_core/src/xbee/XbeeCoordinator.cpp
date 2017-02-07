#include "XbeeCoordinator.h"
#include <boost/bind.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xbeeCoordinator");
	ros::NodeHandle nh;

	std::string xbeePort;
	if(!nh.getParam("xbeeCoordinator/port", xbeePort))
	{
		ROS_ERROR("Could not get xbeeCoordinator port name, will exit");
		return -1;
	}

	XbeeCoordinator coordinator(nh, xbeePort);

	ros::spin();
	return 0;
}

XbeeCoordinator::XbeeCoordinator(ros::NodeHandle &nh, const std::string& port):
  m_xbee(nh, port),
  m_rtkCount('A')
{
  m_xbee.registerReceiveMessageCallback(boost::bind(&XbeeCoordinator::processXbeeMessage, this, _1, _2, _3, _4) );

  m_runstopSubscriber = nh.subscribe("runstop", 1,
                                       &XbeeCoordinator::runstopCallback,
                                       this);
  m_baseStationRTKSubscriber = nh.subscribe("gpsBaseRTCM3", 1,
                                       &XbeeCoordinator::gpsCorrectionsCallback,
                                       this);
}

XbeeCoordinator::~XbeeCoordinator()
{}

void XbeeCoordinator::processXbeeMessage(const std::string& sender,
                                         const std::string& /*networkAddress*/,
                                         const std::string& data,
                                         const bool /*broadcast*/)
{
  std::string msg = data.substr(0,2);
  ROS_DEBUG_STREAM("Xbee: rf message from" << sender << ":" << data);
  if(msg == "HI")
  {
    ROS_INFO_STREAM("New Xbee in system: " << sender);
    RobotState toAdd(sender, data.substr(2));
    m_robotInfos[sender] = toAdd;
    //give 5 extra seconds to complete startup before the node is stale
    m_robotInfos[sender].lastHeartbeat = ros::Time::now()+ros::Duration(5);
    std::string sendData = "AK " + m_xbee.getAddress();
    m_xbee.sendTransmitPacket(sendData, sender);
  } else if(msg == "ST")
  {
    m_robotInfos[sender].lastHeartbeat = ros::Time::now();
    m_robotInfos[sender].name = data.substr(2);
    m_xbee.m_port.diag_ok("Xbee node: " + m_robotInfos[sender].name);
  } else if(msg == "AK")
  {
    ROS_ERROR("XbeeCoordinator: Received AK message, there may be another\
              coordinator with address %s", sender.c_str());
  } else if (msg == "OD")
  {
    //if(data.length() == 62)
    //{
    //  processXbeeOdom(data, sender);
    //}
    //else
    //  ROS_ERROR("XbeeNode: Received incorrect length(%d) odom message \"%s\"", (int)data.length(),data.c_str());
  } else
  {
    ROS_ERROR("XbeeCoordinator: Received unknown message %s", msg.c_str());
  }
}

void XbeeCoordinator::runstopCallback(const autorally_msgs::runstopConstPtr& msg)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1);
  ss << (int)msg->motionEnabled;

  std::string sendData = "RS " + msg->sender + " " + ss.str();
  m_xbee.sendTransmitPacket(sendData);
  //ROS_INFO(sendData.c_str());  

  //ros::Time now = ros::Time::now();
  //std::map<std::string, RobotState>::const_iterator mapIt;
  //if state has not been received from a node in > 2 seconds, send a
  //runstop = 0.0 to that node (overriding the broadcast runstop),
  //regardless of what overall system runstop is
  //for(mapIt = m_robotInfos.begin(); mapIt != m_robotInfos.end(); mapIt++)
  //{
    //if we received a hearbeat in the last 2 sec from xbee
    //if( (ros::Time::now()-mapIt->second.lastHeartbeat) < ros::Duration(2.0) )
    //{
      //ROS_WARN("No recent heartbeat from %s", mapIt->first.c_str());
      //sendData = "RS " + msg->sender + " 0.00";
      //m_xbee.sendTransmitPacket(sendData, mapIt->second.address);
    //}
  //}

}

void XbeeCoordinator::gpsCorrectionsCallback(const std_msgs::ByteMultiArray::ConstPtr& correction)
{
  //xbee packet has max len of 72 bytes
  //size_t bytesSent = 0;
  std::string correctionSubstr;
  size_t payloadSize = 67;
  int numMsgs = 1 + (correction->data.size()-1)/payloadSize;
  int msgNum = 1;

  //each msg gets a unique character to identify it
  //this system will break down if more than 27 GPS RTCM msgs are being sent per second
  if(m_rtkCount == 'Z')
  {
    m_rtkCount = 'A';
  } else
  {
    ++m_rtkCount;
  }

  //send a header packet with identifier, msg count, label from the MultiByteArray
  //std::string msgHeader = "GC" + boost::lexical_cast<std::string>(m_rtkCount) + boost::lexical_cast<std::string>(numMsgs);
  //ROS_INFO_STREAM("Sending gps correction len " << correction->data.size() << 
  //                " in " << numMsgs << " xbee packets");
  //ROS_INFO_STREAM(msgHeader + boost::lexical_cast<std::string>(msgNum) +
  //                            correction->layout.dim.front().label);
  if(correction->layout.dim.front().label.size() < payloadSize)
  {
    //if(m_xbee.sendTransmitPacket(msgHeader +
    //                           boost::lexical_cast<std::string>(msgNum) +
    //                           correction->layout.dim.front().label) )
    //{
      //msgNum++;
      std::vector<unsigned char> msgBody;
      
      while(msgNum <= numMsgs)
      {
        //msg payload is 67 bytes of data appended
        //correctionSubstr.clear();
        msgBody.clear();
        msgBody.push_back('G');
        msgBody.push_back('C');
        msgBody.push_back(m_rtkCount);
        msgBody.push_back(boost::lexical_cast<char>(numMsgs));
        msgBody.push_back(boost::lexical_cast<char>(msgNum));
        
        for(int i = (msgNum-1)*payloadSize; i < std::min<int>((msgNum)*payloadSize, correction->data.size()); i++)
        {
          msgBody.push_back(correction->data[i]);
        }

        ROS_INFO_STREAM("GC" << m_rtkCount << std::to_string(numMsgs) << 
                        std::to_string(msgNum) << " positions " << (msgNum-1)*payloadSize << ":" <<
                        std::min<int>((msgNum)*payloadSize, correction->data.size()));

        if(!m_xbee.sendTransmitPacket(msgBody))
        {
          ROS_ERROR_STREAM("XbeeCoordinator: transmit of gps correction packet " << msgNum << "/" <<
                           numMsgs << " failed");
        } else
        {
          msgNum++;
        }
        
      }
    //} else
    //{
    //  ROS_ERROR("XbeeCoordinator gps correction header transmit failed");
    //}
  } else
  {
    ROS_WARN_STREAM("Xbee: ByteMultiArray label " << correction->layout.dim.front().label << " will be clipped to " <<
                    payloadSize << " bytes");
  }
}

double XbeeCoordinator::unscaleAndClip(int number, double range, double resolution)
{
  return ((double)number * resolution) - range;
}

void XbeeCoordinator::processXbeeOdom(const std::string& message, const std::string& sender)
{
  //ROS_WARN_STREAM("Received:" << message << " from:" << sender);
  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);

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
  /*ROS_WARN("[%f %f %f][%f %f %f][%f %f %f %f]", 
           odom->pose.pose.position.x,
           odom->pose.pose.position.y,
           odom->pose.pose.position.z,
           odom->twist.twist.linear.x,
           odom->twist.twist.linear.y,
           odom->twist.twist.linear.z,
           odom->pose.pose.orientation.x,
           odom->pose.pose.orientation.y,
           odom->pose.pose.orientation.z,
           odom->pose.pose.orientation.w);
  */

  if(!m_recOdomPublishers[sender])
  {
    ros::NodeHandle nh;
    m_recOdomPublishers[sender] = nh.advertise<nav_msgs::Odometry>
                                      ("/pose_estimate_"+sender, 1);

  }
  odom->header.stamp = ros::Time::now();
  odom->child_frame_id = sender;
  m_xbee.m_port.tick("/pose_estimate_"+sender);
  m_recOdomPublishers[sender].publish(odom);
}
