#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dump relevant contents of rosbag file into numpy arrays
"""

import rosbag
import rospy
import numpy as np
import cv2 as cv


fpath = './large-files/'
casename = 'platformA_2015-08-13-12-02-41_split1'

# set up folders before running - casename folder and images subfolder
outpath = "./large-files/" + casename + "/"
posefile = outpath + "pose.npz"
camfile = outpath + "cam_info.npz"

# Open bag
bag = rosbag.Bag(fpath + casename + ".bag")

# Print out topics if needed
topics = bag.get_type_and_topic_info()[1].keys()
print(topics)
# %% Coord transform between chassis and camera frame
numsg = 5
qC = np.array([])
# for tf_static, need to hardcode for live
for topic, msg, t in bag.read_messages(topics=['/tf_static']):
    numsg -= 1
    if numsg < 1:
        break

    qC = np.array([[msg.transforms[1].transform.translation.x,
                    msg.transforms[1].transform.translation.y,
                    msg.transforms[1].transform.translation.z]])

    # %% Camera Intrinsic Matrix
numsg = 5
for topic, msg, t in bag.read_messages(topics=['/left_camera/camera_info']):

    K = np.reshape(msg.K, (3, 3))
    height = msg.height
    width = msg.width
    print("camera info: \n" + str(K) + "\n" + str(qC) + "\n" + str(
        height) + "x" + str(width))
    numsg -= 1
    if numsg < 1:
        break

# %% Pose Information and Images

state_topics = ["/ground_truth/state"]
image_topics = ["/left_camera/image_raw"]

t0 = bag.get_start_time()
tf = bag.get_end_time()

# iterate through bag to get poses
x = y = z = t1 = t2 = np.array([])
quat = np.empty((4, 1))

for topic, msg, t in bag.read_messages(start_time=rospy.Time(t0),
                                       end_time=rospy.Time(tf),
                                       topics=state_topics + image_topics):
    if topic in state_topics:
        t2 = np.append(t2, rospy.Time.to_sec(t))
        x = np.append(x, msg.pose.pose.position.x)
        y = np.append(y, msg.pose.pose.position.y)
        z = np.append(z, msg.pose.pose.position.z)
        new_quat = np.array([[msg.pose.pose.orientation.x],
                             [msg.pose.pose.orientation.y],
                             [msg.pose.pose.orientation.z],
                             [msg.pose.pose.orientation.w]])
        quat = np.hstack((quat, new_quat))

    elif topic in image_topics:
        img = np.frombuffer(msg.data, dtype='uint8').reshape(msg.height,
                                                             msg.width, 3)
        tstr = str(rospy.Time.to_sec(t))
        outfile = outpath + "images/" + tstr + ".png"
        #        cv.imwrite(outfile,cv.cvtColor(img, cv.COLOR_RGB2BGR))
        cv.imwrite(outfile, img)

quat = np.delete(quat, 0, 1)

pos = np.vstack((x, y, z))

bag.close()

# %% Save output


np.savez(posefile, pos=pos, quat=quat, tt=t2)
np.savez(camfile, K=K, qC=qC, height=height, width=width)

cam_info = {"K": K, "qC": qC, "height": 1024, "width": 1280}
