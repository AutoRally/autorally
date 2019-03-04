#!/usr/bin/env python

import rospy
import numpy as np

import os

#Import message types
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from autorally_msgs.msg import chassisState, runstop
from autorally_msgs.msg import lapStatsStamped

map_dict = {
	"CCRF":[([12.5, 17.5], [-7.5, -7.5]),
		([-6,-4], [-1, -6]),
		([-11.5, -7.5], [-20, -17]),
		([7.5, 12], [-37, -33]),
		([26, 27], [-26.5, -31]),
		([24, 28.5],[-19, -16]),
		([7.5, 14],[-20, -20.5]),
		([-4, 0],[-9.5,-13])],
	"Marietta":[],
	"Gazebo":[]
}

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

class Lap:

	def __init__(self, gates, rl_mode, pose_estimate, starting_gate = 0):
		self.gate_idx = starting_gate
		self.gates = gates
		self.reinforcement_learning_mode = rl_mode
		self.lap_time = 0
		self.start_time = None
		self.end_time = None
		self.last_eval = None
		self.max_speed = 0
		self.max_slip = 0
		self.lap_number = 1
		self.pub = rospy.Publisher("~lap_stats", lapStatsStamped, queue_size = 1)
		rospy.Subscriber(pose_estimate, Odometry, self.process_pose)

	def publish_msg(self, timestamp):
		msg = lapStatsStamped()
		msg.header.stamp = timestamp
		msg.header.frame_id = "recorder"
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
		timestamp = pose_msg.header.stamp
		r,p,ya = convert_quat_to_euler(pose_msg.pose.pose.orientation)
		v_x = np.cos(ya)*pose_msg.twist.twist.linear.x + np.sin(ya)*pose_msg.twist.twist.linear.y
		v_y = -np.sin(ya)*pose_msg.twist.twist.linear.x + np.cos(ya)*pose_msg.twist.twist.linear.y
		#Process the pose to get statistics
		total_v = (v_x**2 + v_y**2)**.5
		if (total_v > self.max_speed):
			self.max_speed = total_v
		slip = 0
		if (v_x > 0.1):
			slip = -np.arctan(v_y/np.abs(v_x))
		if (slip > self.max_slip):
			self.max_slip = slip
		self.track_progress(x,y, timestamp)

	def track_progress(self, x, y, timestamp):
		gate = self.gates[self.gate_idx]
		slope = (gate[1][1] - gate[1][0])/(gate[0][1] - gate[0][0])
		offset = gate[1][1] - gate[0][1]*slope
		d1 = np.sqrt((x - gate[0][0])**2 + (y - gate[1][0])**2)
		d2 = np.sqrt((x - gate[0][1])**2 + (y - gate[1][1])**2)
		#print np.max([d1, d2])
		if (np.max([d1, d2]) < 4.5): #Point is close to the gate, run the test
			eval = (y < x*slope + offset)
			if (self.last_eval != eval) and ((self.last_eval is not None) or (not self.reinforcement_learning_mode)): #Switch occured
				if self.start_time is None:
					print "Starting new lap, gate idx: " + str(self.gate_idx)
					self.start_time = timestamp
				elif (timestamp.to_sec() - self.start_time.to_sec() > 5.0):
					print "Ending lap, gate idx: " + str(self.gate_idx)
					self.end_time = timestamp
					self.lap_time = self.end_time.to_sec() - self.start_time.to_sec()
					self.publish_msg(timestamp)
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
		if (self.reinforcement_learning_mode):
			self.gate_idx = (self.gate_idx + 1) % len(Gates)
		print "Resetting lap, new gate idx: " + str(self.gate_idx)


if __name__ == "__main__":
	rospy.init_node("stat_tracker", anonymous = True)
	track_name = rospy.get_param("~track_name")
	rl_mode = rospy.get_param("~rl_mode")
	pose_estimate = rospy.get_param("~pose_estimate")
	starting_gate = rospy.get_param("~starting_gate")
	current_lap = Lap(map_dict[track_name],  rl_mode, pose_estimate, starting_gate = starting_gate)	
	rospy.spin()
	