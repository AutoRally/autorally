import cv2 as cv
import numpy as np


def WaterShedSegmentation(cv_image, grey_image, output_image_index):
    # https://docs.opencv.org/master/d3/db4/tutorial_py_watershed.html

    ret, thresh = cv.threshold(grey_image, 0, 255,
                               cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
    # noise removal
    kernel = np.ones((3, 3), np.uint8)
    opening = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel, iterations=2)
    # sure background area
    sure_bg = cv.dilate(opening, kernel, iterations=3)
    # Finding sure foreground area
    dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 0)
    ret, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255,
                                0)
    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    # fg_file = str(output_image_index) + "-foreground.png"
    # cv.imwrite(fg_file, sure_fg)
    unknown = cv.subtract(sure_bg, sure_fg)
    # Marker labelling
    ret, markers = cv.connectedComponents(sure_fg)
    # Add one to all labels so that sure background is not 0, but 1
    markers = markers + 1
    # Now, mark the region of unknown with zero
    markers[unknown == 255] = 0
    # Apply watershed
    markers = cv.watershed(cv_image, markers)
    # marker_file = str(output_image_index) + "-markers.png"
    # cv.imwrite(marker_file, markers)
    # image was read only for some reason
    cv_image.setflags(write=1)
    cv_image[markers == -1] = [0, 0, 255]
    result_image = cv_image
    return result_image

def WaterShedSegmentation(cv_image, grey_image, output_image_index):