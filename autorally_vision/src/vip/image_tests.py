"""
Pull image from rosbag and call processing
"""

import rosbag
import numpy as np
from cv_bridge import CvBridge
# from algos import WaterShedSegmentation

bridge = CvBridge()
import cv2 as cv

fpath = '/home/todd/autorally/'
fname = 'alpha_autorally0_2020-07-23-16-27-57_0.bag'

# Open bag
bag = rosbag.Bag(fpath + fname)
output_image_index = 0
# Print out topics if needed
topics = bag.get_type_and_topic_info()[1].keys()
print(topics)
for topic, msg, t in bag.read_messages(topics=topics):
    print(msg)
"""
for topic, msg, t in bag.read_messages(topics=['/left_camera/image_color/compressed']):
    # Convert image message to OpenCV format
    cv_image = bridge.compressed_imgmsg_to_cv2(msg)
    grey_image = cv.cvtColor(cv_image, cv.COLOR_RGB2GRAY)

    # pick an algo
    result_image = WaterShedSegmentation(cv_image, grey_image, output_image_index)

    # display image
    image_file = str(output_image_index) + ".png"
    print("writing to file: ", image_file)
    output_image_index = output_image_index + 1
    cv.imwrite(image_file, result_image)
"""
bag.close()
