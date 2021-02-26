"""
Calculates roughness metric based on IMU Z axis disturbance
"""
import numpy as np
import rosbag
import rospy
import matplotlib.pyplot as plt

# bagfile directory
fpath = '/home/todd/autorally/'

# bagfile
fname = fpath + 'alpha_autorally0_2020-07-23-16-27-57_0.bag'  # from track not sim

bag = rosbag.Bag(fname)

topics = bag.get_type_and_topic_info()[1].keys()

# Demo:
disturbance_data = np.array([])
roughness_data = np.array([])
time = np.array([])
t0 = 0
dt = 1.0  # timestep analysed, to be tied into back projection time stamp
running_squared_data = np.array([])  # measurements used in calculation of metric
num_data_points_cap = 12000
num_data_points = num_data_points_cap
for topic, msg, t in bag.read_messages(topics=['/imu/imu']):
    if num_data_points == num_data_points_cap:
        t0 = msg.header.stamp.to_time()
    num_data_points = num_data_points - 1
    if num_data_points < 0:
        break
    imu_data = msg
    current_time = msg.header.stamp.to_time() - t0
    vertical_acceleration = imu_data.linear_acceleration.z + 9.81
    running_squared_data = np.append(running_squared_data, vertical_acceleration * vertical_acceleration)
    disturbance_data = np.append(disturbance_data, vertical_acceleration)
    time = np.append(time, current_time)
    if current_time > dt:
        # calculate metric
        roughness_metric = (np.average(running_squared_data, returned=True))[0]
        # print("roughness_metric: " + str(roughness_metric))
        roughness_data = np.append(roughness_data, roughness_metric)
        # take off LRU
        running_squared_data = np.delete(running_squared_data, 0)
# Extend roughness data to be able to plot it
while len(disturbance_data) > len(roughness_data):
    roughness_data = np.append(roughness_data, 0.0)

# make pretty graphs
fig, ax = plt.subplots()
ax.set_xlabel('time')
ax.set_ylabel('z-axis disturbance and metric')
ax.set_title('IMU-based roughness metric with timestep of 1s')
disturbance, = ax.plot(time, disturbance_data, label="Disturbance")
metric, = ax.plot(time, roughness_data, label="Roughness Metric")
# ax.plot(time, disturbance_data, time, roughness_data)
ax.legend()
plt.show()
bag.close()
