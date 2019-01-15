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

Gates = [([12.5, 17.5], [-7.5, -7.5]),
([-6,-4], [-1, -6]),
([-11.5, -7.5], [-20, -17]),
([7.5, 12], [-37, -33]),
([26, 27], [-26.5, -31]),
([24, 28.5],[-19, -16]),
([7.5, 14],[-20, -20.5]),
([-4, 0],[-9.5,-13])]

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

	def __init__(self, params, prefix):
		#Start with gate 3
		self.gate_idx = 1
		self.lap_time = 0
		self.start_time = None
		self.end_time = None
		self.last_eval = None

		self.max_speed = 0
		self.max_slip = 0
		self.lap_number = 1
		self.params = params
		self.prefix = prefix
		self.pub = rospy.Publisher('lap_stats', pathIntegralStats, queue_size = 1)
		rospy.Subscriber("/mppi_controller/subscribedPose", Odometry, self.process_pose)

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
		v_x = pose_msg.twist.twist.linear.x
		v_y = pose_msg.twist.twist.linear.y
		#Process the pose to get statistics
		total_v = (v_x**2 + v_y**2)**.5
		if (total_v > self.max_speed):
			self.max_speed = total_v
		slip = 0
		if (v_x > 0.1):
			slip = -np.arctan(v_y/np.abs(v_x))
		if (slip > self.max_slip):
			self.max_slip = slip
		self.track_progress(x,y)

	def track_progress(self, x, y):
		gate = Gates[self.gate_idx]
		slope = (gate[1][1] - gate[1][0])/(gate[0][1] - gate[0][0])
		offset = gate[1][1] - gate[0][1]*slope
		d1 = np.sqrt((x - gate[0][0])**2 + (y - gate[1][0])**2)
		d2 = np.sqrt((x - gate[0][1])**2 + (y - gate[1][1])**2)
		#print np.max([d1, d2])
		if (np.max([d1, d2]) < 4.5): #Point is close to the gate, run the test
			eval = (y < x*slope + offset)
			if (self.last_eval != eval) and (self.last_eval is not None): #Switch occured
				if self.start_time is None:
					print "Starting new lap, gate idx: " + str(self.gate_idx)
					self.start_time = rospy.get_rostime()
				elif (rospy.get_rostime().to_sec() - self.start_time.to_sec() > 5.0):
					print "Ending lap, gate idx: " + str(self.gate_idx)
					self.end_time = rospy.get_rostime()
					self.lap_time = self.end_time.to_sec() - self.start_time.to_sec()
					self.params = get_launch_params(self.prefix)
					self.publish_msg()
					self.reset_lap()
			else:
				self.last_eval = eval

	def reset_lap(self):
		self.last_eval = None
		self.start_time = None
		self.end_time = None
		self.lap_time = None
		self.max_speed = 0
		self.max_slip = 0
		self.lap_number += 1
		self.gate_idx = (self.gate_idx + 1) % len(Gates)
		print "Resetting lap, new gate idx: " + str(self.gate_idx)


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

	current_lap = Lap(param_dict, prefix)	
	rospy.spin()
	