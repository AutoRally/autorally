import numpy as np
from PIL import Image
import argparse

def gen_costmap(input_img, config_file, output_name):
	dict_file = config_file

	with open(dict_file,'r') as inf:
	    config_dict = eval(inf.read())

	data = Image.open(input_img)

	#Rotate the image so that the origin is in the top left corner.
	data = data.rotate(config_dict["imageRotation"])

	#Cast image to numpy array
	data = np.array(data, dtype = np.float32)

	data[:,:,0] = (data[:,:,0] + config_dict["rOffset"])/config_dict["rNormalizer"]
	data[:,:,1] = (data[:,:,1] + config_dict["gOffset"])/config_dict["gNormalizer"]
	data[:,:,2] = (data[:,:,2] + config_dict["bOffset"])/config_dict["bNormalizer"]
	data[:,:,3] = (data[:,:,3] + config_dict["aOffset"])/config_dict["aNormalizer"]

	costmap = np.copy(data)
	for i in range(4):
		costmap[:,:,config_dict["channelMap"][i]] = data[:,:,i]

	if (config_dict["flip"]):
		for i in range(4):
			costmap[:,:,i] = np.flipud(costmap[:,:,i])

	#Save data to numpy array, each channel is saved individually as an array in row major order.
	track_dict = {"xBounds":np.array(config_dict["xBounds"], dtype = np.float32), 
				  "yBounds":np.array(config_dict["yBounds"], dtype = np.float32),
				  "pixelsPerMeter":np.array(config_dict["pixelsPerMeter"], dtype=np.float32),
				  "channel0":costmap[:,:,0].flatten(),
				  "channel1":costmap[:,:,1].flatten(),
				  "channel2":costmap[:,:,2].flatten(),
	              "channel3":costmap[:,:,3].flatten()}

	np.savez(output_name, **track_dict)


if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--input", type = str, help = "Image to generate costmap with")
	parser.add_argument("-c", "--config", type = str, help = "Costmap configuration file")
	parser.add_argument("-o", "--output", type = str, help = "File to save map to", default = "map.npz")
	args = vars(parser.parse_args())
	gen_costmap(args["input"], args["config"], args["output"])
