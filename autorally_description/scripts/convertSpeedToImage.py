#!/usr/bin/env python

import cv2
import numpy as np
from math import sqrt, atan, degrees

def main():
    # creates a blank image and splits the color channels
    img = cv2.imread('HighResTrack.png',cv2.IMREAD_COLOR)

    b_channel, g_channel, r_channel = cv2.split(img)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255

    height, width, depth = img.shape

    flag = 0
    for i in range(0, height):
        for j in range(0, (width)):
            """
            if img[i, j][0] == 0 and img[i,j][1] == 0 and img[i,j][2] == 0:
                if flag == 0:
                    flag = 1
                if flag == 2:
                    flag = 3
            elif (img[i, j][0] != 0 or img[i,j][1] != 0 or img[i,j][2] != 0) and flag > 0:
                if flag == 3:
                    flag = 0
                elif img[i,j][2] != 150:
                     alpha_channel[i,j] = 0
                    flag = 2
            flag = 0
            """
            if img[i, j][0] != 0 or img[i,j][1] != 0 or img[i,j][2] != 0:
                alpha_channel[i,j] = 0

    # writes out the image
    img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
    cv2.imwrite("../urdf/textures/blended_texture_ccrf.png", img_RGBA)

if __name__ == '__main__':
    main()
