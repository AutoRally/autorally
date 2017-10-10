import cv2
import numpy as np
from math import sqrt, atan, degrees

# tuple for the size of the field X, Y
ground_plane_size = 30

# pixel width and height
image_size = 1000


def create_circle(centerX, centerY, radiusIn, radiusOut, orientationStart, orientationEnd, alpha_channel):
    for i in range(centerX - convert_distance_to_pixel(radiusOut), centerX + convert_distance_to_pixel(radiusOut)):
        for j in range(centerY - convert_distance_to_pixel(radiusOut), centerY + convert_distance_to_pixel(radiusOut)):
            distance = convert_pixel_to_distance(centerX, centerY, i, j)
            theta = -1
            if i - centerX != 0:
                theta = degrees(atan((float(j) - centerY)/(i - centerX)))

            if i >= centerX and j <= centerY:
                theta += 360
            elif i <= centerX and j >= centerY:
                theta += 180
            elif i <= centerX and j <= centerY:
                theta += 180

            if distance <= radiusOut and distance >= radiusIn and theta >= orientationStart and theta <= orientationEnd:
                alpha_channel[i, j] = 0

    return alpha_channel

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


    alpha_channel = create_circle(image_size / 2, convert_distance_to_pixel(9), 4, 8, 180, 360, alpha_channel)
    alpha_channel = create_circle(image_size / 2, convert_distance_to_pixel(21), 4, 8, 0, 180, alpha_channel)
    alpha_channel = create_line(convert_distance_to_pixel(8.5), convert_distance_to_pixel(9), 4, 13, alpha_channel)
    alpha_channel = create_line(convert_distance_to_pixel(8.5), convert_distance_to_pixel(21), 4, 13, alpha_channel)

    img_RGBA = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
    cv2.imwrite("../urdf/blended_texture.png", img_RGBA)

if __name__ == '__main__':
    main()
