#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
        arr = np.load("../params/maps/gazebo_costmap_01_08_2019.npz")
        rospy.init_node('track_boundaries')
        inner_X, inner_Y, outer_X, outer_Y = arr['X_in'], arr['Y_in'], arr['X_out'], arr['Y_out']
        pub1 = rospy.Publisher("inner_boundary", Path, queue_size = 1,latch=True)
        pub2 = rospy.Publisher("outer_bounday", Path, queue_size = 1,latch=True)
        path1 = Path()
        path2 = Path()
        for i in range(inner_X.size):
            loc = PoseStamped()
            loc.pose.position.x = inner_X[i]
            loc.pose.position.y = inner_Y[i]
            loc.pose.position.z = 0
            loc.pose.orientation.w = 0
            loc.pose.orientation.x = 0
            loc.pose.orientation.y = 0
            loc.pose.orientation.z = 0
            loc.header.frame_id = "base_link"
            path1.poses.append(loc)
        for i in range(outer_X.size):
            loc = PoseStamped()
            loc.pose.position.x = outer_X[i]
            loc.pose.position.y = outer_Y[i]
            loc.pose.position.z = 0
            loc.pose.orientation.w = 0
            loc.pose.orientation.x = 0
            loc.pose.orientation.y = 0
            loc.pose.orientation.z = 0
            loc.header.frame_id = "base_link"
            path2.poses.append(loc)
        path1.header.frame_id = "base_link"
        path2.header.frame_id = "base_link"
        pub1.publish(path1)
        pub2.publish(path2)
        rospy.spin()
        
