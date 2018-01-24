#!/usr/bin/env python

import rospy
import numpy as np

import os

#Import message types
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from autorally_msgs.msg import chassisState, runstop
from autorally_msgs.msg import pathIntegralStats

#Slope, Offset, X min, X max
gazebo_line = [1.0, 5.0, -13, -9]
marietta_line = [1.0, -13, -3, 0]
ccrf_line = [1.0, -26.4, -2, 2.25]

def convert_quat_to_euler(quat):
	q0 = quat.w
	q1 = quat.x
	q2 = quat.y
	q3 = quat.z
	#Using the 1-2-3 euler angle convention
	roll = np.arctan2(2*q2*q3 + 2*q0*q1, q3**2 - q2**2 - q1**2 + q0**2)
	pitch = -np.arcsin(2*q1*q3 - 2*q0*q2)
	#Yaw is heading
	yaw = np.arctan2(2*q1*q2 + 2*q0*q3, q1**2 + q0**2 - q3**2 - q2**2)
	return roll, pitch, yaw

def get_launch_params(prefix):
	param_dict = {}
	param_dict["hz"] = rospy.get_param(prefix + "/hz")
	param_dict["num_timesteps"] = rospy.get_param(prefix + "/num_timesteps")
	param_dict["tag"] = rospy.get_param("/stat_tracker/tag")
	param_dict["gamma"] = rospy.get_param(prefix + "/gamma")
	param_dict["num_iters"] = rospy.get_param(prefix + "/num_iters")
	param_dict["init_steering"] = rospy.get_param(prefix + "/init_steering")
	param_dict["init_throttle"] = rospy.get_param(prefix + "/init_throttle")
	param_dict["steering_var"] = rospy.get_param(prefix + "/steering_std")
	param_dict["throttle_var"] = rospy.get_param(prefix + "/throttle_std")
	param_dict["max_throttle"] = rospy.get_param(prefix + "/max_throttle")
	param_dict["desired_speed"] = rospy.get_param(prefix + "/desired_speed")
	param_dict["speed_coefficient"] = rospy.get_param(prefix + "/speed_coefficient")
	param_dict["track_coefficient"] = rospy.get_param(prefix + "/track_coefficient")
	param_dict["max_slip_angle"] = rospy.get_param(prefix + "/max_slip_angle")
	param_dict["slip_penalty"] = rospy.get_param(prefix + "/slip_penalty")
	param_dict["track_slop"] = rospy.get_param(prefix + "/track_slop")
	param_dict["crash_coeff"] = rospy.get_param(prefix + "/crash_coeff")
	param_dict["map_path"] = rospy.get_param(prefix + "/map_path")
	return param_dict

class Lap:

	def __init__(self, line, params, prefix):
		self.line = line
		self.last_eval = 0
		self.start_time = None
		self.end_time = None
		self.lap_time = 0
		self.max_speed = 0
		self.max_slip = 0
		self.lap_number = 1
		self.params = params
		self.prefix = prefix
		self.pub = rospy.Publisher('lap_stats', pathIntegralStats, queue_size = 1)
		rospy.Subscriber("/pose_estimate", Odometry, self.process_pose)

	def reset_lap(self):
		self.last_eval = 0
		self.start_time = rospy.get_rostime()
		self.end_time = None
		self.lap_time = 0
		self.max_speed = 0
		self.max_slip = 0
		self.lap_number += 1

	def publish_msg(self):
		msg = pathIntegralStats()
		msg.header.stamp = rospy.get_rostime()
		msg.header.frame_id = "mppi_stats"
		msg.tag = self.params["tag"]
		msg.params.hz = self.params["hz"]
		msg.params.num_timesteps = self.params["num_timesteps"]
		msg.params.num_iters = self.params["num_iters"]
		msg.params.gamma = self.params["gamma"]
		msg.params.init_steering = self.params["init_steering"]
		msg.params.init_throttle = self.params["init_throttle"]
		msg.params.steering_var = self.params["steering_var"]
		msg.params.throttle_var = self.params["throttle_var"]
		msg.params.max_throttle = self.params["max_throttle"]
		msg.params.speed_coefficient = self.params["speed_coefficient"]
		msg.params.track_coefficient = self.params["track_coefficient"]
		msg.params.max_slip_angle = self.params["max_slip_angle"]
		msg.params.track_slop = self.params["track_slop"]
		msg.params.crash_coeff = self.params["crash_coeff"]
		msg.params.map_path = self.params["map_path"]
		msg.params.desired_speed = self.params["desired_speed"]
		msg.stats.lap_number = self.lap_number
		msg.stats.lap_time = self.lap_time
		msg.stats.max_speed = self.max_speed
		msg.stats.max_slip = self.max_slip
		self.pub.publish(msg)

	def process_pose(self, pose_msg):
		#Get the pose from the message
		x = pose_msg.pose.pose.position.x
		y = pose_msg.pose.pose.position.y
		z = pose_msg.pose.pose.position.z
		x_dot = pose_msg.twist.twist.linear.x
		y_dot = pose_msg.twist.twist.linear.y
		r,p,ya = convert_quat_to_euler(pose_msg.pose.pose.orientation)
		vel_wf = np.array([x_dot, y_dot])
		rot_mat = np.array([[np.cos(ya), np.sin(ya)], [-np.sin(ya), np.cos(ya)]])
		vel_bf = np.dot(rot_mat, vel_wf)
		v_x = vel_bf[0]
		v_y = vel_bf[1]
		#Process the pose to get statistics
		total_v = (v_x**2 + v_y**2)**.5
		if (total_v > self.max_speed):
			self.max_speed = total_v
		slip = 0
		if (v_x > 0.1):
			slip = -np.arctan(v_y/np.abs(v_x))
		if (slip > self.max_slip):
			self.max_slip = slip
		line_eval = (y > self.line[0]*x + self.line[1])
		#Check if we've completed the last
		if ((self.last_eval is not 0) and (line_eval is not self.last_eval) and (x > self.line[2]) and (x < self.line[3])):
			if (self.start_time is None):
				self.start_time = rospy.get_rostime()
			else:
				self.end_time = rospy.get_rostime()
				self.lap_time = self.end_time.to_sec() - self.start_time.to_sec()
				#Read the launch params
				self.params = get_launch_params(self.prefix)
				self.publish_msg()
				self.reset_lap()
		self.last_eval = line_eval

if __name__ == "__main__":
	rospy.init_node("stat_tracker", anonymous = True)
	prefix =  rospy.get_param("/stat_tracker/controller_type")
	param_dict = None
	start = rospy.get_rostime()
	curr = rospy.get_rostime()
	while (param_dict is None):
		curr = rospy.get_rostime()
		if (curr.to_sec() - start.to_sec() > 30.0):
			print curr.to_sec() - start.to_sec()
			break
		try:
			param_dict = get_launch_params(prefix)
		except KeyError:
			pass

	line = None
	if ("gazebo" in param_dict["map_path"]):
		line = gazebo_line
	elif ("marietta" in param_dict["map_path"]):
		line = marietta_line
	elif ("ccrf" in param_dict["map_path"]):
		line = ccrf_line
	elif ("SDF_Track" in param_dict["map_path"]):
		line = ccrf_line

	else:
		rospy.signal_shutdown("No start line for the given map.")
	current_lap = Lap(line, param_dict, prefix)	
	rospy.spin()

