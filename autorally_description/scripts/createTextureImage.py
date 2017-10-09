import cv2
import numpy as np
from math import sqrt, tan

# tuple for the size of the field X, Y
ground_plane_size = 30

# pixel width and height
image_size = 300


def create_circle(centerX, centerY, radiusIn, radiusOut, orientationStart, orientationEnd, alpha_channel):
    #print "X range = ", centerX - convert_distance_to_pixel(radiusOut), "to", centerX + convert_distance_to_pixel(radiusOut)
    #print "Y range = ", centerY - convert_distance_to_pixel(radiusOut), "to", centerY + convert_distance_to_pixel(radiusOut)
    for i in range(centerX - convert_distance_to_pixel(radiusOut), centerX + convert_distance_to_pixel(radiusOut)):
        for j in range(centerY - convert_distance_to_pixel(radiusOut), centerY + convert_distance_to_pixel(radiusOut)):
            distance = convert_pixel_to_distance(centerX, centerY, i, j)
            if distance <= radiusOut and distance >= radiusIn:
                alpha_channel[i, j] = 0

    return alpha_channel

def check_angle(calcualted_theta, orientationStart, orientationEnd):
    return

def convert_pixel_to_distance(pixel1X, pixel1Y, pixel2X, pixel2Y):
    xDist = (pixel1X - pixel2X) * (float(ground_plane_size) / image_size)
    yDist = (pixel1Y - pixel2Y) * (float(ground_plane_size) / image_size)
    return sqrt(xDist**2 + yDist**2)

def convert_distance_to_pixel(distance):
    return int(round(distance * (float(image_size) / ground_plane_size)))

def create_line(centerX, centerY, width, length, alpha_channel):
    half_width = float(width) / 2
    for i in range(centerY - convert_distance_to_pixel(half_width), centerY + convert_distance_to_pixel(half_width)):
        for j in range(centerX, centerX + convert_distance_to_pixel(length)):
            alpha_channel[i, j] = 0
    return alpha_channel



def main():
    blank_image = np.zeros((image_size,image_size,3))
    b_channel, g_channel, r_channel = cv2.split(blank_image)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255


    #alpha_channel = create_circle(0, 0, 1, 4, 0, 360, alpha_channel)
    #alpha_channel = create_circle(image_size / 2, 3 * image_size / 4, 1, 4, 0, 360, alpha_channel)
    #alpha_channel = create_line(5, 5, 1, 1, alpha_channel)
    #alpha_channel = create_line(image_size / 4, image_size / 2, 5, 10, alpha_channel)

    img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
    cv2.imwrite("../urdf/test.png", img_RGBA)

if __name__ == '__main__':
    main()
