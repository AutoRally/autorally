#!/usr/bin/env python

# TODO: Pull bag selection, timestep into command line arg

from collections import Counter

import back_projection as backProp
import roughness
import rosbag
import rospy
import numpy as np
import cv2 as cv
import statistics
import os
from pathlib import Path

npz_exists = False


def main(isTrack, dt):
    # get bagfile

    bagpath = './large-files/'
    track_bag = "alpha_autorally0_2020-07-23-16-27-57_0.bag"
    sim_bag = "2020-10-15-11-16-39.bag"
    if isTrack:
        fname = bagpath + track_bag
    else:
        fname = bagpath + sim_bag
    bag = rosbag.Bag(fname)

    state_topics = ["/ground_truth/state",
                    "/particle_filter/pose_estimate"]
    image_topics = ["/left_camera/image_color/compressed",
                    "/left_camera/image_raw/compressed"]

    # setup output file
    outpath = './large-files/'
    if isTrack:
        outpath = outpath + track_bag[:-4] + '/'
    else:
        outpath = outpath + sim_bag[:-4] + '/'
    if not os.path.exists(outpath):
        Path(outpath).mkdir()
        Path(outpath + "/images/").mkdir()

    posefile = outpath + "pose.npz"
    camfile = outpath + "cam_info.npz"

    Path(posefile).touch()
    Path(camfile).touch()

    max_num_images = 6000
    start_num_image = 200
    num_images_norm = 0
    num_images_processed = 0

    # First pass: gather metric labels for time slots
    # labels is indexed by time stamps, metrics are 0-int indexed
    labels = Counter()
    times, metrics, disturbance = roughness.get_labels(bag, dt)
    for i in range(len(times)):
        labels[times[i]] = metrics[i]

    maxMetric = max(metrics)
    minMetric = min(metrics)
    stdDevMetric = statistics.stdev(metrics)
    avgMetric = statistics.mean(metrics)
    # Second pass: generate posefile and camfile, from rosbag_dump.py
    # TODO: Functionize this setup
    numsg = 3
    if not isTrack:
        for topic, msg, t in bag.read_messages(topics=['/tf_static']):
            numsg -= 1
            if numsg < 1:
                break
            qC = np.array([[msg.transforms[1].transform.translation.x,
                            msg.transforms[1].transform.translation.y,
                            msg.transforms[1].transform.translation.z]])
            numsg = 2
            for topic, msg, t in bag.read_messages(
                    topics=['/left_camera/camera_info']):
                numsg -= 1
                if numsg < 1:
                    break
                K = np.reshape(msg.K, (3, 3))
                height = msg.height
                width = msg.width
                np.savez(camfile, K=K, qC=qC, height=height, width=width)
    if not npz_exists:
        # only do once
        # %% Coord transform between chassis and camera frame

        # Print out topics if needed
        # print(str(bag.get_type_and_topic_info()[1].keys()))

        # %% Pose Information and Images

        # iterate through bag to get poses
        x = y = z = t1 = t2 = np.array([])
        quat = np.empty((4, 1))
        print("parsing poses")
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
        quat = np.delete(quat, 0, 1)
        pos = np.vstack((x, y, z))
        # Save output
        np.savez(posefile, pos=pos, quat=quat, tt=t2)
        print("posefile saved with quaternion: " + str(quat))
    pose = np.load(posefile)
    for topic, msg, t in bag.read_messages(
            topics=state_topics + image_topics):
        if topic in image_topics:
            np_arr = np.fromstring(msg.data, np.uint8)
            img = cv.imdecode(np_arr, cv.IMREAD_COLOR)

            # img = np.frombuffer(msg.data, dtype='uint8').reshape(msg.height,
            #                                                      msg.width,
            #                                                      3)
            tstr = str(msg.header.stamp.to_time())
            outfile = outpath + "images/" + tstr + ".png"
            num_images_norm = num_images_norm + 1
            if num_images_norm < start_num_image:
                continue
            else:
                # image wizardry time
                # cv.imwrite(outfile, img)
                # get prop and draw
                x, y, label = backProp.back_projection(pose, None,
                                                       msg.header.stamp.to_time(),
                                                       not isTrack,
                                                       times, metrics)
                # print("projected into: " + str(x) + ", " + str(y))
                # making pretty pictures
                for i in range(len(x)):
                    lw = 3  # width of line
                    # TODO: change color based on label
                    color = np.array([0, 0, 0])
                    if label[i] < avgMetric - stdDevMetric:
                        # low disturbance, blue
                        color = np.array([255, 0, 0])
                    elif label[i] < avgMetric + stdDevMetric:
                        # mid, green
                        color = np.array([0, 255, 0])
                    else:
                        # high, red
                        color = np.array([0, 0, 255])
                    img[(x[i] - lw):(x[i] + lw), (y[i] - lw):(y[i] + lw),
                    :] = color
                cv.imwrite(outfile, img)
                print("edited image " + outfile)

            if num_images_norm - start_num_image > max_num_images:
                break


if __name__ == '__main__':
    main(True, 1)
