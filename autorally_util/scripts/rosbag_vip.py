#! /usr/bin/env python
import argparse

from matplotlib import rc

import numpy as np
import rosbag
import rospy
import cv2

from mppi_msgs.msg import mppiStatistics
from autorally_msgs.msg import pathIntegralStats # Lap Stats
from autorally_msgs.msg import runstop # runstop
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script will run through a bag file" +
                                     " and create graphs of the free energy, velocities, " +
                                     "and nominal states selected per lap and for the entire" +
                                     " bag. These are always saved to the output directory")
    parser.add_argument("rosbag_path", type=str, help="Path to rosbag")

    args = parser.parse_args()
    print("Rosbag path: ", args.rosbag_path)


    state_topics = ["/pose_estimate", "/ground_truth/state", "/particle_filter/pose_estimate"]
    image_topics = ["/right_camera/image_raw"]
    control_topics = ["/joystick/chassisCommand"]
    bag = rosbag.Bag(args.rosbag_path)

    topics = bag.get_type_and_topic_info()[1].keys() # list of all topics in the bag file
    print("topics\n", topics)

    t0 = bag.get_start_time()
    tf = bag.get_end_time()

    for topic, msg, t in bag.read_messages(start_time=rospy.Time(t0),
                                                end_time=rospy.Time(tf),
                                                topics=state_topics + image_topics + control_topics):
        print("Topic: ", topic)
        t_float = msg.header.stamp.to_sec()
        if topic in state_topics:
            print("Got Odometry\n", msg)
            print("X location: ", msg.pose.pose.position.x)
            # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        elif topic in control_topics:
            print("got control topic\n", msg)
        elif topic in image_topics:
            #print("msg: ", msg)
            #print("got image: ", len(msg.data[:]))
            img = np.frombuffer(msg.data, dtype='uint8').reshape(msg.height, msg.width, 3)

            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.imshow('image', img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()



