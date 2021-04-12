#!/user/bin/env python
"""
Calculates roughness metric based on IMU Z axis disturbance
"""
import numpy as np
import rosbag
import rospy
# import matplotlib.pyplot as plt

"""

Returns (np.array, np.array, np.array) 
    = (timeStamp, metric at timeStamp, raw measurement at timeStamp)
"""


def get_labels(bagfile, timestep):
    disturbance_data = np.array([])
    roughness_data = np.array([])
    time_relative = np.array([])
    time_ros = np.array([])
    t0 = 0
    dt = timestep
    running_squared_data = np.array([])  # used in calculation of metric
    first_entry = True
    for topic, msg, t in bagfile.read_messages(topics=['/imu/imu']):
        if first_entry:
            t0 = msg.header.stamp.to_time()
            first_entry = False
        imu_data = msg
        current_time = msg.header.stamp.to_time() - t0
        vertical_acceleration = imu_data.linear_acceleration.z + 9.81
        running_squared_data = \
            np.append(running_squared_data,
                      vertical_acceleration * vertical_acceleration)
        disturbance_data = np.append(disturbance_data, vertical_acceleration)
        time_relative = np.append(time_relative, current_time)
        time_ros = np.append(time_ros, msg.header.stamp.to_time())
        # if dt time has not passed yet we do not have enough data
        if current_time > dt:
            # calculate metric
            roughness_metric = \
                (np.average(running_squared_data, returned=True))[0]
            # print("roughness_metric: " + str(roughness_metric))
            roughness_data = np.append(roughness_data, roughness_metric)
            # take off LRU
            running_squared_data = np.delete(running_squared_data, 0)
        else:
            roughness_data = np.append(roughness_data, 0)
    # Extend roughness data to be able to plot it
    # while len(disturbance_data) > len(roughness_data):
    #     roughness_data = np.append(roughness_data, 0.0)
    return time_ros, roughness_data, disturbance_data


if __name__ == '__main__':
    # bagfile directory
    fpath = '/home/todd/autorally/'

    # bagfile
    fname = fpath + 'alpha_autorally0_2020-07-23-16-27-57_0.bag'  # trackbag

    bag = rosbag.Bag(fname)

    topics = bag.get_type_and_topic_info()[1].keys()

    times, roughness, disturbance = get_labels(bag, 1)
    # make pretty graphs
    fig, ax = plt.subplots()
    ax.set_xlabel('time')
    ax.set_ylabel('z-axis disturbance and metric')
    ax.set_title('IMU-based roughness metric with timestep of 1s')
    disturbance, = ax.plot(times, disturbance, label="Disturbance")
    metric, = ax.plot(times, roughness, label="Roughness Metric")
    # ax.plot(time, disturbance_data, time, roughness_data)
    ax.legend()
    plt.show()
    bag.close()
