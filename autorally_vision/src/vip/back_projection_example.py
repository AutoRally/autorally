

''' Back Projection Example '''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import cv2 as cv
import back_projection as bp


fpath = '/home/thomas/Downloads/'
casename = '2020-10-15-11-16-39'

outpath = "/home/thomas/Documents/Traversability/" + casename + "/"
posefile = outpath + "pose.npz"
camfile = outpath + "cam_info.npz"


# Select projection time
t1 = 23.174
#t1 = 24.712
#t1 = 31.845

# Load files
pose = np.load(posefile)
cam_info = np.load(camfile)
fname = outpath + "images/" + str(t1) + ".png"
img = cv.imread(fname)

# check for missing image name
if img is None:
    raise Exception("Cannot find image by name: \n" + fname)

# Run back projection for selected time t1
rx,cx = bp.back_projection(pose, cam_info, t1)


# Color back projected pixels on image
for i in range(len(rx)):
    lw = 3 # width of line
    img[(rx[i]-lw):(rx[i]+lw),(cx[i]-lw):(cx[i]+lw),:] = np.array([255,0,0])
fig = plt.figure()
plt.imshow(img)
plt.title("Camera POV at t = " + str(t1))