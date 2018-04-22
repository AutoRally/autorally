import numpy as np
from PIL import Image

dict_file = "/home/gwilliams76/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/gazebo/gazebo_track.cfg"

with open(dict_file,'r') as inf:
    config_dict = eval(inf.read())

data = Image.open("/home/gwilliams76/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/gazebo/gazebo_map.png")

#Rotate the image so that the origin is in the top left corner.
data = data.rotate(config_dict["imageRotation"])

#Cast image to numpy array
data = np.array(data, dtype = np.float32)

data[:,:,0] = (data[:,:,0] + config_dict["rOffset"])/config_dict["rNormalizer"]
data[:,:,1] = (data[:,:,1] + config_dict["gOffset"])/config_dict["gNormalizer"]
data[:,:,2] = (data[:,:,2] + config_dict["bOffset"])/config_dict["bNormalizer"]
data[:,:,3] = (data[:,:,3] + config_dict["aOffset"])/config_dict["aNormalizer"]

#Save data to numpy array, each channel is saved individually as an array in row major order.
track_dict = {"xBounds":np.array(config_dict["xBounds"], dtype = np.float32), 
			  "yBounds":np.array(config_dict["yBounds"], dtype = np.float32),
			  "pixelsPerMeter":np.array(config_dict["pixelsPerMeter"], dtype=np.float32),
			  "channel0":data[:,:,0].flatten(),
			  "channel1":data[:,:,1].flatten(),
			  "channel2":data[:,:,2].flatten(),
              "channel3":data[:,:,3].flatten()}

np.savez("gazebo_map.npz", **track_dict)
