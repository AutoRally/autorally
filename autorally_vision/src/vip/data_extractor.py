import argparse
import matplotlib.pyplot as plt
import numpy as np
import pickle
import copy
import rosbag
import yaml
import rospy
import os
import glob
import yaml

from scipy import interpolate
from math import atan2, atan, asin, tan


# Converts a quaternion to 1-2-3 Euler Angles
def convert_quat_to_euler(quat):
    q0 = quat.w
    q1 = quat.x
    q2 = quat.y
    q3 = quat.z
    # Using the 1-2-3 euler angle convention
    roll = atan2(2 * q2 * q3 + 2 * q0 * q1,
                 q3 ** 2 - q2 ** 2 - q1 ** 2 + q0 ** 2)
    pitch = -asin(2 * q1 * q3 - 2 * q0 * q2)
    yaw = atan2(2 * q1 * q2 + 2 * q0 * q3,
                q1 ** 2 + q0 ** 2 - q3 ** 2 - q2 ** 2)
    return roll, pitch, yaw


# Converts angular velocity to euler angle derivatives
def convert_angular_to_ederiv(ang_vel, r, p, y):
    conversion_mat = 1 / np.cos(p) * np.array(
        [[np.cos(p), np.sin(r) * np.sin(p), np.cos(r) * np.sin(p)],
         [0, np.cos(r) * np.cos(p), -np.sin(r) * np.cos(p)],
         [0, np.sin(r), np.cos(r)]])
    ang_vel_np = np.array([ang_vel[0], ang_vel[1], ang_vel[2]])
    derivs = np.dot(conversion_mat, ang_vel_np)
    return derivs[0], derivs[1], derivs[2]


def convert_yaw_range(yaw):
    heading = np.zeros_like(yaw)
    last_heading = yaw[0]
    heading[0] = last_heading
    heading_multiplier = 0
    for t in range(1, len(yaw)):
        if last_heading > 3.0 and yaw[t] < -3.0:
            heading_multiplier += 1
        elif last_heading < -3.0 and yaw[t] > 3.0:
            heading_multiplier -= 1
        last_heading = yaw[t]
        heading[t] = yaw[t] + heading_multiplier * 2 * np.pi
    return heading


def interpolate_varying_distances_single(times, values, target_time):
    result = interpolate_varying_distances_multiple(times, values,
                                                    [target_time])
    return result[0]


def interpolate_varying_distances_multiple(times, values, target_times):
    result = []
    current_index = 0
    end = target_times[-1]
    for target_time in target_times:
        while current_index < len(times) and times[current_index] < end and \
                times[current_index] < target_time:
            current_index += 1
        if current_index == 0:
            result.append(values[current_index])
        elif current_index < len(times):
            # print "current index: ", current_index
            # print "target time: ", [target_time]
            # print "time to interp: ", times[current_index-1:current_index+1], " diff: ", target_time - times[current_index-1], " dt ", times[current_index] - times[current_index-1]
            # print "value to interp: ", values[current_index-1:current_index+1]
            new_value = interpolate_state(
                times[current_index - 1:current_index + 1],
                values[current_index - 1:current_index + 1], target_time)
            # print "interped value: ", new_value
            result.append(new_value)
            # print "last time: ", last_time
            # print "times ", times[current_index-1] - last_time, " ", times[current_index] - last_time, " alpha ", alpha
            # result.append(values[current_index-1] * (1-alpha) + values[current_index]*alpha)
            # print "result size ", len(result)
            # print "current index: ", current_index
            # print "target time ", target_time
    return result


def calculate_rate_of_change(times, values, target_times):
    result = []
    current_index = 0
    end = target_times[-1]
    for target_time in target_times:
        while current_index < len(times) and times[current_index] < end and \
                times[current_index] < target_time:
            current_index += 1
        if current_index == 0:
            result.append(values[current_index])
        elif current_index < min(len(times), len(values)):
            # print "len of values: ", len(values)
            # print "current index: ", current_index
            # print "target time: ", [target_time]
            # print "time to interp: ", times[current_index-1:current_index+1], " diff: ", target_time - times[current_index-1], " dt ", times[current_index] - times[current_index-1]
            # print "value to interp: ", values[current_index-1:current_index+1]
            new_value = compute_diff(times[current_index - 1:current_index + 1],
                                     values[
                                     current_index - 1:current_index + 1])
            # print "interped value: ", new_value
            result.append(new_value)
            # print "last time: ", last_time
            # print "times ", times[current_index-1] - last_time, " ", times[current_index] - last_time, " alpha ", alpha
            # result.append(values[current_index-1] * (1-alpha) + values[current_index]*alpha)
            # print "result size ", len(result)
            # print "current index: ", current_index
            # print "target time ", target_time
    return result


def interpolate_state(times, values, target_time):
    # print "times ", times
    alpha = (target_time - times[0]) / (times[1] - times[0])
    # print "alpha: ", alpha
    assert alpha >= 0 or alpha <= 1
    return values[0] * (1 - alpha) + values[1] * (alpha)


def compute_diff(times, values):
    return (values[1] - values[0]) / (times[1] - times[0])


def get_all_bag_data(args):
    result = {}

    # check if we are only using a single bag or loading a collection of them
    if args['dataset'][-3:-1] == "bag":
        result[args['dataset']] = get_bag_data(args)
        return result
    else:
        base_path = args['dataset'] + "/"
        bags = [os.path.basename(x) for x in glob.glob(base_path + "*.bag")]

        # if does not exist create default one
        if not os.path.exists(base_path + "dataset_config.yaml"):
            print("cannot find config file ", base_path + "dataset_config.yaml",
                  " creating default")
            config_file = {}

            config_file['dt'] = 0.02
            config_file['smooth'] = True
            for bag in bags:
                config_file[bag] = {}
                config_file[bag]['start_time'] = 0
                config_file[bag]['duration'] = -1
                config_file[bag]['dataset'] = base_path + bag

            yaml.dump(config_file, open(base_path + "dataset_config.yaml", 'w'),
                      default_flow_style=False)
        else:
            print("loading config file with dataset")

        config_file = yaml.load(open(base_path + "dataset_config.yaml", "r"))

        for bag in bags:
            dataset_file_name = bag
            dataset_file_name += "_smooth_" + str(config_file['smooth'])
            for key in config_file[bag].keys():
                if key != "dataset":
                    dataset_file_name += "_" + key + "_" + str(
                        config_file[bag][key])
            dataset_file_name += ".pkl"

            print("dataset_file_name", dataset_file_name)

            if os.path.exists(base_path + dataset_file_name) and not args[
                'reload_data']:
                print("loading previsouly extracted dataset file: ",
                      dataset_file_name)

                result[bag] = pickle.load(open(base_path + dataset_file_name))
            else:
                print("extracting from the bag file: ", bag)

                local_params = config_file[bag]
                local_params['dt'] = config_file['dt']

                result[bag] = get_bag_data(local_params,
                                           smooth_data=config_file['smooth'])

                pickle.dump(result[bag],
                            open(base_path + dataset_file_name, 'wb'))
    return result


def interpolate_data(Y, T, spline_pwr=3, der=0):
    knots = np.linspace(T[0], T[-1], len(T) / 3)[1:-1]
    spline_params = interpolate.splrep(np.asarray(T), np.asarray(Y),
                                       k=spline_pwr, t=knots)
    return interpolate.splev(T, spline_params, der=der)


# extracts the topics out of the bag file
def get_bag_data(args, smooth_data=False):
    bag_file = rosbag.Bag(args['dataset'], 'r')
    # Postion and velocities
    x = []
    y = []
    x_dot = []
    y_dot = []
    vx = []
    vy = []
    # Euler Angles and derivatives
    roll = []
    pitch = []
    yaw = []
    angV_x = []
    angV_y = []
    angV_z = []
    # Commands
    throttle_cmd = []
    steering_cmd = []
    frontBrake_cmd = []
    backBrake_cmd = []
    # Time Stamps
    state_ts = []
    control_ts = []

    info_dict = yaml.load(bag_file._get_yaml_info())

    dt = args['dt']

    if info_dict['duration'] < args['duration']:
        print("using in invalid end time, longer than entire bag")
        args['duration'] = info_dict['duration']

    start_time = rospy.Time(info_dict['start'] + args['start_time'])
    end_time = rospy.Time(start_time.to_sec() + args['duration'])
    if args['duration'] == -1:
        end_time = rospy.Time(info_dict['end'])
    print("search in bag file from ", start_time.to_sec(), " to ",
          end_time.to_sec())

    # Fill state arrays
    for topic, msg, t in bag_file.read_messages(
            topics=['/particle_filter/pose_estimate'], start_time=start_time,
            end_time=end_time):
        state_ts.append(
            msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0)
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)
        x_dot.append(msg.twist.twist.linear.x)
        y_dot.append(msg.twist.twist.linear.y)
        r, p, ya = convert_quat_to_euler(msg.pose.pose.orientation)
        roll.append(r)
        pitch.append(p)
        yaw.append(ya)
        rot_mat = np.array(
            [[np.cos(ya), np.sin(ya)], [-np.sin(ya), np.cos(ya)]])
        vel_wf = np.array([x_dot[-1], y_dot[-1]])
        vel_bf = np.dot(rot_mat, vel_wf)
        vx.append(vel_bf[0])
        vy.append(vel_bf[1])
        # Get angular velocity *Not* equal to euler angle derivatives
        angV_x.append(msg.twist.twist.angular.x)
        angV_y.append(msg.twist.twist.angular.y)
        angV_z.append(msg.twist.twist.angular.z)

    # Fill control command arrays
    for topic, msg, t, in bag_file.read_messages(topics=['/chassisState'],
                                                 start_time=rospy.Time(
                                                         start_time.to_sec() - 1),
                                                 end_time=end_time):
        control_ts.append(
            msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0)
        throttle_cmd.append(msg.throttle)
        steering_cmd.append(msg.steering)
        frontBrake_cmd.append(msg.frontBrake)
        backBrake_cmd.append(msg.frontBrake)

    desired_indexes = int((state_ts[-1] - state_ts[0]) / dt)

    new_state_ts = np.linspace(state_ts[0], state_ts[0] + dt * desired_indexes,
                               desired_indexes)
    new_control_ts = np.linspace(state_ts[0],
                                 state_ts[0] + dt * desired_indexes,
                                 desired_indexes)
    print("state_ts ", len(new_state_ts), " control_ts ", len(new_control_ts))

    # calculate the rates directly
    ax = calculate_rate_of_change(state_ts, vx, new_state_ts)
    ay = calculate_rate_of_change(state_ts, vy, new_state_ts)
    a_yaw = calculate_rate_of_change(state_ts, angV_z, new_state_ts)

    x = interpolate_varying_distances(state_ts, x, new_state_ts)
    y = interpolate_varying_distances(state_ts, y, new_state_ts)
    vx = interpolate_varying_distances(state_ts, vx, new_state_ts)
    vy = interpolate_varying_distances(state_ts, vy, new_state_ts)
    roll = interpolate_varying_distances(state_ts, roll, new_state_ts)
    pitch = interpolate_varying_distances(state_ts, pitch, new_state_ts)
    yaw = interpolate_varying_distances(state_ts, yaw, new_state_ts)
    angV_x = interpolate_varying_distances(state_ts, angV_x, new_state_ts)
    angV_y = interpolate_varying_distances(state_ts, angV_y, new_state_ts)
    angV_z = interpolate_varying_distances(state_ts, angV_z, new_state_ts)

    throttle_cmd = interpolate_varying_distances(control_ts, throttle_cmd,
                                                 new_control_ts)
    steering_cmd = interpolate_varying_distances(control_ts, steering_cmd,
                                                 new_control_ts)
    # print "len x: ", len(x)
    # print "len steering ", len(steering_cmd)

    if smooth_data:
        x = interpolate_data(x, new_state_ts)
        y = interpolate_data(y, new_state_ts)
        vx = interpolate_data(vx, new_state_ts)
        vy = interpolate_data(vy, new_state_ts)
        roll = interpolate_data(roll, new_state_ts)
        pitch = interpolate_data(pitch, new_state_ts)
        yaw = interpolate_data(yaw, new_state_ts)
        angV_x = interpolate_data(angV_x, new_state_ts)
        angV_y = interpolate_data(angV_y, new_state_ts)
        angV_z = interpolate_data(angV_z, new_state_ts)

        ax = interpolate_data(vx, new_state_ts, der=1)
        ay = interpolate_data(vy, new_state_ts, der=1)
        a_yaw = interpolate_data(angV_z, new_state_ts, der=1)

    yaw = convert_yaw_range(yaw)

    return ({"x": x, "y": y, "vx": vx, "vy": vy,
             "roll": roll, "pitch": pitch, "yaw": yaw,
             "angV_x": angV_x, "angV_y": angV_y, "angV_z": angV_z,
             "throttle_cmd": throttle_cmd, "steering_cmd": steering_cmd},
            {"ax": ax, "ay": ay, "a_yaw": a_yaw},
            {"state_ts": new_state_ts, "control_ts": new_control_ts, 'dt': dt})
