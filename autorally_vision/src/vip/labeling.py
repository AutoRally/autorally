#!/usr/bin/env python

# TODO: Pull bag selection, timestep into command line arg

from collections import Counter

import back_projection as backProp
import roughness
import rosbag
import rospy
import numpy as np
import cv2 as cv

npz_exists = False


def main(isTrack, dt):
    # get bagfile

    bagpath = '/home/todd/autorally/'
    track_bag = "alpha_autorally0_2020-07-23-16-27-57_0.bag"
    sim_bag = "2020-10-15-11-16-39.bag"
    if isTrack:
        fname = bagpath + track_bag
    else:
        fname = bagpath + sim_bag
    bag = rosbag.Bag(fname)

    # setup output file
    outpath = '/home/todd/autorally/labeled_images/'
    if isTrack:
        outpath = outpath + track_bag[:-4] + '/'
    else:
        outpath = outpath + sim_bag[:-4] + '/'
    posefile = outpath + "pose.npz"
    camfile = outpath + "cam_info.npz"

    # First pass: gather metric labels for time slots
    labels = Counter()
    times, metrics, disturbance = roughness.get_labels(bag, dt)
    for i in range(len(times)):
        labels[times[i]] = metrics[i]

    # Second pass: generate posefile and camfile, from rosbag_dump.py
    # TODO: Functionize this setup

    if not npz_exists:
        # only do once
        # %% Coord transform between chassis and camera frame

        # Print out topics if needed
        print(str(bag.get_type_and_topic_info()[1].keys()))
        if isTrack:
            for topic, msg, t in bag.read_messages(topics=['/tf_static']):
                qC = np.array([[msg.transforms[1].transform.translation.x,
                                msg.transforms[1].transform.translation.y,
                                msg.transforms[1].transform.translation.z]])

                K = np.reshape(msg.K, (3, 3))
                height = msg.height
                width = msg.width

        # %% Pose Information and Images
        state_topics = ["/ground_truth/state"]
        image_topics = ["/left_camera/image_raw"]

        # iterate through bag to get poses
        x = y = z = t1 = t2 = np.array([])
        quat = np.empty((4, 1))

        for topic, msg, t in bag.read_messages(
                topics=state_topics + image_topics):
            if topic in state_topics:
                t2 = np.append(t2, msg.header.stamp.to_time())
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
                                                                     msg.width,
                                                                     3)
                tstr = str(msg.header.stamp.to_time())
                outfile = outpath + "images/" + tstr + ".png"
                #        cv.imwrite(outfile,cv.cvtColor(img, cv.COLOR_RGB2BGR))
                cv.imwrite(outfile, img)

        quat = np.delete(quat, 0, 1)
        pos = np.vstack((x, y, z))
        # Save output
        np.savez(posefile, pos=pos, quat=quat, tt=t2)
        # np.savez(camfile, K=K, qC=qC, height=height, width=width)
        # TODO: reimplement cam_info and camfile properly
        cam_info = {"K": K, "qC": qC, "height": 1024, "width": 1280}

    # Third pass: get back prop
    pose = np.load(posefile)
    # cam_info = np.load(camfile)
    for t_des in times:
        x, y = backProp.back_projection(pose, None, t_des, False)


if __name__ == '__main__':
    main(True, 1)
