#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project points onto focal plane
"""
import numpy as np
import rosbag
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# import quat_utils

import cv2 as cv

# fpath = '/Users/thomas.king/Downloads/'
fpath = '/home/thomas/Downloads/'
# fname = '2020-09-14-14-35-43_0.bag'
fname = '2020-10-15-11-16-39.bag'


# %% Open bag and extract pose and images


def back_projection(pose, cam_info, t_des, use_cam_info):
    # TODO: add metric to npz
    tt, car_pos, car_quat = pose["tt"], pose["pos"], pose["quat"]
    if use_cam_info:
        K, qC, height, width = cam_info["K"], cam_info["qC"], cam_info[
            "height"], cam_info["width"]
    else:
        # Override camera values
        K = np.array([[762.72493376, 0., 640.5],
                      [0., 762.72493376, 512.5],
                      [0., 0., 1.]])

        qC = np.array([[0.127, 0., 0.2159]])
        height = 1024
        width = 1280
    # Frame Definitions:
    #   I: inertial, world frame
    #   B: body, car frame
    #   C: camera frame (3D coord, not image plane)

    tix = np.argmin(np.abs(tt - t_des))
    t1_quat = car_quat[:, tix]

    # Get vehicle pose at t1
    q_I_t1 = car_pos[:, tix]

    # Get vehicle pose at future times, t2 > t1
    q_I_t2 = car_pos[:, (tix + 1):]
    # TODO: labeling goes here

    # Get vector from t1 to t2 in body frame
    R_B_I = quat2mat(qinv(t1_quat))  # get quaternion to t1
    p_I_t2 = np.transpose(q_I_t2.T - q_I_t1.T)  # pos vec in I frame
    p_B_t2 = np.dot(R_B_I, p_I_t2)  # rotate to B frame

    # Rotation from B to C: y points down, z points forward, x to the right
    R_C_B = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])

    # Shift by camera offset and rotate to camera frame
    p_C_t2 = np.dot(R_C_B, (p_B_t2.T - qC).T)

    # Find image coordinates from camera frame
    lam_uv = np.dot(K, p_C_t2)  # get raw projection
    uv = lam_uv / lam_uv[2,
                  :]  # divide by 3rd term to account for distance param

    # filter out points outside of FOV
    inside_h = (uv[0, :] < height) & (uv[0, :] > 0)  # inside height
    inside_w = (uv[1, :] < width) & (uv[1, :] > 0)  # inside width
    infront = p_C_t2[2, :] > 0  # in front of camera
    keep_ix = inside_h & inside_w & infront
    uv_fov = uv[:2, keep_ix]  # don't need 3rd param anymore, should be all 1's

    # round and cast to integer indices
    uv_ix = np.round(uv_fov).astype(int)

    rx = uv_ix[1, :]
    cx = uv_ix[0, :]

    return rx, cx


def quat2mat(q):
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    return [[qw ** 2 + qx ** 2 - qy ** 2 - qz ** 2, 2 * qx * qy - 2 * qw * qz,
             2 * qx * qz + 2 * qw * qy],
            [2 * qx * qy + 2 * qw * qz, qw ** 2 - qx ** 2 + qy ** 2 - qz ** 2,
             2 * qy * qz - 2 * qw * qx],
            [2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx,
             qw ** 2 - qx ** 2 - qy ** 2 + qz ** 2]]


def qinv(q):
    return np.hstack((-q[0:3], q[3]))

# %%  Example
# cam_info = {"K":K,"qC":qC,"height":1024,"width":1280}
# rx,cx = back_projection(t2,t2_pose, t2_quat, cam_info, t1)
#
# img_copy = img.copy()   # np.fromBuffer points img to memory buffer of original
#                        # data, which is immutable, so need to make copy
## Color pixels
# for i in range(len(rx)):
#    lw = 3 # width of line
#    img_copy[(rx[i]-lw):(rx[i]+lw),(cx[i]-lw):(cx[i]+lw),:] = np.array([255,0,0])
# fig = plt.figure()
# plt.imshow(img_copy)
# plt.title("Right Camera POV at t = " + str(t0))
