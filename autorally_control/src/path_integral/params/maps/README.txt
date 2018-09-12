# Data Format

This folder contains contains costmaps used by MPPI in order to drive within the track boundaries. Each costmap contains the same data, but for a different track. The data arrays contained in the numpy archives are the following:

xBounds, yBounds - These are the bounds of the costmap, described in meters from the origin. Together they define a rectangular area that the track is entirely contained in.

pixelsPerMeter - The number of pixels in a meter. The size of the costmap in pixels is then: pixelsPerMeter^2 * (x_max - x_min) * (y_max - y_min).

channel0 - This channel contains the actual values describing the track surface. It is stored in row-major order, with each row containing (x_max - x_min)*pixelsPerMeter values. Values of zero indicate the track centerline, and values of 1.0 indicate the track boundary. Anything between 0 and 1 is a location on the track, and anything above 1.0 defines a region outside the boundaries.

channel1, channel2, channel3 - These channels are currently all zero. They can be modified in order to include additional data about the track that can be used by classes inheriting from costs.cu.

# Maps

gazebo_costmap_09_08_2018.npz - Map of the gazebo simulation environment. Origin is the same as the spawn point for the car.

marietta_costmap_09_08_2018.npz - Map of the marietta track, located just south of Georgia Tech. Origin is on the side of the track closest to Marietta street, and is marked by the tops of two metal stakes buried in the ground.

marietta_costmap_09_08_2018.npz - Map of the marietta track, located just south of Georgia Tech. Origin is on the side of the track furthest to Marietta street. This map is now deprecated, as the origin marker has been lost and the track boundaries have been moved after construction clean-up. 

ccrf_costmap_09_29_2019.npz - Map of the track located at Georgia Tech's Cobb County Research Facility. Origin is on the in the pit lane by the car-port, and is marked by four metal stakes buried in the ground.
