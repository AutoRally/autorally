import numpy as np
from PIL import Image
import argparse

def gen_costmap(costmap, image_name, output_name):
	
  cmap_file = open(costmap, "r")
  cmap = cmap_file.read()
  cmap = cmap.split(" ")
  channel0 = np.array(cmap[5:-1], dtype = np.float32)
  channel1 = np.zeros_like(channel0)
  channel2 = np.zeros_like(channel0)
  channel3 = np.zeros_like(channel0)

  #Save data to numpy array, each channel is saved individually as an array in row major order.
  track_dict = {"xBounds":np.array([cmap[0], cmap[1]], dtype = np.float32), 
				  "yBounds":np.array([cmap[2], cmap[3]], dtype = np.float32),
				  "pixelsPerMeter":np.array([cmap[4]], dtype=np.float32),
				  "channel0":channel0,
				  "channel1":channel1,
				  "channel2":channel2,
	        "channel3":channel3}

  w = int((float(cmap[3]) - float(cmap[2]))*float(cmap[4]))
  h = int((float(cmap[1]) - float(cmap[0]))*float(cmap[4]))
  
  img = channel0.reshape((w,h))
  img = img[::-1,:]

  img = np.array(img*255.0, dtype=np.uint8)
  img = Image.fromarray(img)
  img.save(image_name)

  np.savez(output_name, **track_dict)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("-i", "--input", type = str, help = "Costmap in old .txt format")
  parser.add_argument("-d", "--display", type = str, help = "Name of image to save costmap as", default="display_image.jpg")
  parser.add_argument("-o", "--output", type = str, help = "File to save map to", default = "map.npz")
  args = vars(parser.parse_args())
  gen_costmap(args["input"], args["display"], args["output"])
