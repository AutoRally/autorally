#!/usr/bin/env python

"""autorally_controller.py

Control the wheels of a vehicle with Ackermann steering.

Subscribed Topics:
  chassisCommand (autorally_msgs/chassisCOmmand)
    Actuator command to control all actuators, valid values [-1, 1]
  runstop (autorally_msgs/runstop)
    Whether the vehicle can move or not

Published Topics:
  <left steering controller name>/command (std_msgs/Float64)
    Command for the left steering controller.
  <right steering controller name>/command (std_msgs/Float64)
    Command for the right steering controller.
  <left front axle controller name>/command (std_msgs/Float64)
    Command for the left front axle controller.
  <right front axle controller name>/command (std_msgs/Float64)
    Command for the right front axle controller.
  <left rear axle controller name>/command (std_msgs/Float64)
    Command for the left rear axle controller.
  <right rear axle controller name>/command (std_msgs/Float64)
    Command for the right rear axle controller.
  <shock absorber controller name>/command (std_msgs/Float64)
    One of these topics exists for each shock absorber. They are latched
    topics.

Services Called:
  controller_manager/list_controllers (controller_manager_msgs/
                                       ListControllers)
    List the states of the controllers.

Parameters:
  ~left_front_wheel/steering_link_name (string, default: left_steering_link)
  ~right_front_wheel/steering_link_name (string,
                                         default: right_steering_link)
    Names of links that have origins coincident with the origins of the
    left and right steering joints, respectively. The steering links are
    used to compute the distance between the steering joints, as well as
    the vehicle's wheelbase.

  ~left_front_wheel/steering_controller_name (string, default:
                                              left_steering_controller)
  ~right_front_wheel/steering_controller_name (string, default:
                                               right_steering_controller)
Steering controller names.

  ~left_rear_wheel/link_name (string, default: left_wheel)
  ~right_rear_wheel/link_name (string, default: right_wheel)
    Names of links that have origins coincident with the centers of the
    left and right wheels, respectively. The rear wheel links are used to
    compute the vehicle's wheelbase.

  ~left_front_wheel/axle_controller_name (string)
  ~right_front_wheel/axle_controller_name
  ~left_rear_wheel/axle_controller_name
  ~right_rear_wheel/axle_controller_name
    Axle controller names. If no controller name is specified for an axle,
    that axle will not have a controller. This allows the control of
    front-wheel, rear-wheel, and four-wheel drive vehicles.

  ~left_front_wheel/diameter (float, default: 1.0)
  ~right_front_wheel/diameter
  ~left_rear_wheel/diameter
  ~right_rear_wheel/diameter
    Wheel diameters. Each diameter must be greater than zero. Unit: meter.

  ~shock_absorbers (sequence of mappings, default: empty)
    Zero or more shock absorbers.

    Key-Value Pairs:

    controller_name (string)
      Controller name.
    equilibrium_position (float, default: 0.0)
      Equilibrium position. Unit: meter.

  ~cmd_timeout (float, default: 0.5)
    If ~cmd_timeout is greater than zero and this node does not receive a
    command for more than ~cmd_timeout seconds, vehicle motion is paused
    until a command is received. If ~cmd_timeout is less than or equal to
    zero, the command timeout is disabled.
  ~publishing_frequency (float, default: 30.0)
    Joint command publishing frequency. It must be greater than zero.
    Unit: hertz.

Required tf Transforms:
  <~left_front_wheel/steering_link_name> to <~right_rear_wheel/link_name>
    Specifies the position of the left front wheel's steering link in the
    right rear wheel's frame.
  <~right_front_wheel/steering_link_name> to <~right_rear_wheel/link_name>
    Specifies the position of the right front wheel's steering link in the
    right rear wheel's frame.
  <~left_rear_wheel/link_name> to <~right_rear_wheel/link_name>
    Specifies the position of the left rear wheel in the right rear
    wheel's frame.

Copyright (c) 2013-2014 Wunderkammer Laboratory

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import math
from math import pi
import numpy
import threading
import operator

import rospy
import tf

from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState

from autorally_msgs.msg import chassisCommand
from autorally_msgs.msg import chassisState
from autorally_msgs.msg import runstop
from autorally_msgs.msg import wheelSpeeds

class AutoRallyCtrlr(object):
  """AutoRally controller

  An object of class AutoRallyCtrlr is a node that controls the wheels of a
  vehicle with Ackermann steering (AutoRally platform).
  """

  def __init__(self, namespace='controller_manager'):
      """Initialize this _AutoRallyCtrlr."""

      rospy.init_node("autorally_controller")

      # Parameters

      # Wheels
      (left_steer_link_name, left_steer_ctrlr_name,
       left_front_axle_ctrlr_name, self._left_front_inv_circ) = \
       self._get_front_wheel_params("left")
      (right_steer_link_name, right_steer_ctrlr_name,
       right_front_axle_ctrlr_name, self._right_front_inv_circ) = \
       self._get_front_wheel_params("right")
      (left_rear_link_name, left_rear_axle_ctrlr_name,
       self._left_rear_inv_circ) = \
       self._get_rear_wheel_params("left")
      (self._right_rear_link_name, right_rear_axle_ctrlr_name,
       self._right_rear_inv_circ) = \
       self._get_rear_wheel_params("right")

      list_ctrlrs = rospy.ServiceProxy(namespace + '/list_controllers',
                                       ListControllers)
      list_ctrlrs.wait_for_service()

      # Shock absorbers
      shock_param_list = rospy.get_param("~shock_absorbers", [])
      self._shock_pubs = []
      try:
          for shock_params in shock_param_list:
              try:
                  ctrlr_name = shock_params["controller_name"]
                  try:
                      eq_pos = shock_params["equilibrium_position"]
                  except:
                      eq_pos = self._DEF_EQ_POS
                  eq_pos = float(eq_pos)
              except:
                  rospy.logwarn("An invalid parameter was specified for a "
                                "shock absorber. The shock absorber will "
                                "not be used.")
                  continue

              pub = rospy.Publisher(ctrlr_name + "/command", Float64,
                                    latch=True, queue_size=1)
              _wait_for_ctrlr(list_ctrlrs, ctrlr_name)
              pub.publish(eq_pos)
              self._shock_pubs.append(pub)
      except:
          rospy.logwarn("The specified list of shock absorbers is invalid. "
                        "No shock absorbers will be used.")

      # Command timeout
      try:
          self._cmd_timeout = float(rospy.get_param("~cmd_timeout",
                                                    self._DEF_CMD_TIMEOUT))
      except:
          rospy.logwarn("The specified command timeout value is invalid. "
                        "The default timeout value will be used instead.")
          self._cmd_timeout = self._DEF_CMD_TIMEOUT

      try:
          self._vehicle_prefix = rospy.get_param("~vehicle_prefix")
          if self._vehicle_prefix != "":
              self._vehicle_prefix = "/" + self._vehicle_prefix
      except:
          rospy.logwarn("The specified namespace value is invalid. "
                        "The default timeout value will be used instead.")
          self._vehicle_prefix = ""

      # Publishing frequency
      try:
          pub_freq = float(rospy.get_param("~publishing_frequency",
                                           self._DEF_PUB_FREQ))
          if pub_freq <= 0.0:
              raise ValueError()
      except:
          rospy.logwarn("The specified publishing frequency is invalid. "
                        "The default frequency will be used instead.")
          pub_freq = self._DEF_PUB_FREQ
      self._sleep_timer = rospy.Rate(pub_freq)

      # _last_cmd_time is the time at which the most recent Ackermann
      # driving command was received.
      self._last_cmd_time = rospy.get_time()

      self._last_steer_ang = 0.0  # Last steering angle
      self._theta_left = 0.0      # Left steering joint angle
      self._theta_right = 0.0     # Right steering joint angle

      self._last_speed = 0.0
      self._last_accel_limit = 0.0  # Last acceleration limit
      # Axle angular velocities
      self._left_front_ang_vel = 0.0
      self._right_front_ang_vel = 0.0
      self._left_rear_ang_vel = 0.0
      self._right_rear_ang_vel = 0.0

      # _joint_dist_div_2 is the distance between the steering joints,
      # divided by two.
      tfl = tf.TransformListener()
      ls_pos = self._get_link_pos(tfl, left_steer_link_name)
      rs_pos = self._get_link_pos(tfl, right_steer_link_name)
      self._joint_dist_div_2 = numpy.linalg.norm(ls_pos - rs_pos) / 2
      lrw_pos = self._get_link_pos(tfl, left_rear_link_name)
      rrw_pos = numpy.array([0.0] * 3)
      front_cent_pos = (ls_pos + rs_pos) / 2     # Front center position
      rear_cent_pos = (lrw_pos + rrw_pos) / 2    # Rear center position
      self._wheelbase = numpy.linalg.norm(front_cent_pos - rear_cent_pos)
      self._inv_wheelbase = 1 / self._wheelbase  # Inverse of _wheelbase
      self._wheelbase_sqr = self._wheelbase ** 2

      #self.lastCmdTime = rospy.get_time()
      self.chassisCmds = dict()
      self.chassisCmdLock = threading.Lock()

      #load chassis commander priorities
      self.commandPriorities = rospy.get_param("~chassisCommandProirities", [])
      self.commandPriorities = sorted(self.commandPriorities.items(), key=operator.itemgetter(1))
      rospy.loginfo("AutoRallyGazeboController: Loaded %d chassis commanders", len(self.commandPriorities))

      #runstop information
      self.runstops = dict()
      self.runstopLock = threading.Lock()

      self.front_axle_max_effort = 2.5
      self.front_axle_brake_effort = 2.5
      self.rear_axle_max_effort = 8
      self.rear_axle_brake_effort = 4

      #self.rear_axle_reverse_percent = 0.25 # percent of max_effort applied when reversing
      #self.rear_axle_reverse_effort = self.rear_axle_max_effort*self.rear_axle_reverse_percent

      # Publishers and subscribers
      self._left_steer_cmd_pub = \
          _create_cmd_pub(list_ctrlrs, left_steer_ctrlr_name)
      self._right_steer_cmd_pub = \
          _create_cmd_pub(list_ctrlrs, right_steer_ctrlr_name)

      self._left_front_axle_cmd_pub = \
          _create_axle_cmd_pub(list_ctrlrs, left_front_axle_ctrlr_name)
      self._right_front_axle_cmd_pub = \
          _create_axle_cmd_pub(list_ctrlrs, right_front_axle_ctrlr_name)
      self._left_rear_axle_cmd_pub = \
          _create_axle_cmd_pub(list_ctrlrs, left_rear_axle_ctrlr_name)
      self._right_rear_axle_cmd_pub = \
          _create_axle_cmd_pub(list_ctrlrs, right_rear_axle_ctrlr_name)



      self.left_front_name, self.left_front_dia = self.getJointStateWheelParams('left', 'front')
      self.right_front_name, self.right_front_dia = self.getJointStateWheelParams('right', 'front')
      self.left_rear_name, self.left_rear_dia = self.getJointStateWheelParams('left', 'rear')
      self.right_rear_name, self.right_rear_dia = self.getJointStateWheelParams('right', 'rear')

      self.wheelSpeedFront = 0.0;

      #don't set up callback until params are initialized
      self.wheelSpeedsPub = rospy.Publisher(self._vehicle_prefix + '/wheelSpeeds', wheelSpeeds, queue_size=1)
      #self.sub = rospy.Subscriber('/autorally_platform/gazebo/link_states', ModelStates, self.callback)

      self.chassisStatePub = rospy.Publisher(self._vehicle_prefix + "/chassisState", chassisState, queue_size=1)

      self.chassisCmdSub = dict()
      for cmd, priority in self.commandPriorities:
          self.chassisCmdSub[cmd] = \
              rospy.Subscriber(self._vehicle_prefix+"/"+cmd+"/chassisCommand", chassisCommand,
                               self.chassisCmdCb, queue_size=1)

      self.runstopSub = rospy.Subscriber(self._vehicle_prefix + "/runstop", runstop, self.runstopCb, queue_size=5)

      self.wheelSpeedSub = rospy.Subscriber(self._vehicle_prefix + '/joint_states', JointState, self.wheelSpeedsCb)

  def spin(self):
    """Control the vehicle."""

    last_time = rospy.get_time()

    while not rospy.is_shutdown():
      t = rospy.get_time()
      delta_t = t - last_time
      last_time = t

      frontBrake = 0.0;
      speed = 0.0;
      chassisSt = chassisState()
      chassisSt.runstopMotionEnabled = self.getrunstop();
      if (self._cmd_timeout > 0.0 and
        t - self._last_cmd_time > self._cmd_timeout):
        # Too much time has elapsed since the last command. Stop the
        # vehicle.
        steer_ang_changed, center_y = \
          self._ctrl_steering(self._last_steer_ang, 0.0, 0.001)
        self._ctrl_axles(0.0, 0.0, 0.0, steer_ang_changed, center_y)
      elif delta_t > 0.0:
        with self.chassisCmdLock:
          foundSteering = False
          foundThrottle = False
          foundFrontBrake = False
          steer_ang = 0.0
          steer_ang_vel = 0.0
          foundSteering = False
          accel = 0.0

          if not chassisSt.runstopMotionEnabled:
            chassisSt.throttle = 0.0;
            chassisSt.throttleCommander = 'runstop';
            foundThrottle = True

          for cmd,priority in self.commandPriorities:
            #rospy.logwarn("looking for chassis commander %s with priority %d", cmd, priority)
            if cmd in self.chassisCmds:
              if abs(self.chassisCmds[cmd].steering) <= 1.0 and \
                 (rospy.Time.now()-self.chassisCmds[cmd].header.stamp) < \
                    rospy.Duration.from_sec(0.2) and\
                 not foundSteering:
                #rospy.loginfo("%s in control of steering", cmd);
                steer_ang = -math.radians(25)*self.chassisCmds[cmd].steering
                steer_ang_vel = 0.0
                chassisSt.steering = self.chassisCmds[cmd].steering
                chassisSt.steeringCommander = self.chassisCmds[cmd].sender
                foundSteering = True

              if abs(self.chassisCmds[cmd].throttle) <= 1.0 and \
                 (rospy.Time.now()-self.chassisCmds[cmd].header.stamp) < \
                    rospy.Duration.from_sec(0.2) and\
                 not foundThrottle:

                #rospy.loginfo("%s in control of throttle", cmd);
                if self.chassisCmds[cmd].throttle >= 0.0:
                  speed = self.rear_axle_max_effort*self.chassisCmds[cmd].throttle
                else:
                  speed = self.rear_axle_brake_effort*self.chassisCmds[cmd].throttle

                accel = 0.0
                chassisSt.throttle = self.chassisCmds[cmd].throttle
                chassisSt.throttleCommander = self.chassisCmds[cmd].sender
                foundThrottle = True


              if self.chassisCmds[cmd].frontBrake >= 0.0 and \
                 self.chassisCmds[cmd].frontBrake <= 1.0 and \
                 (rospy.Time.now()-self.chassisCmds[cmd].header.stamp) < \
                    rospy.Duration.from_sec(0.2) and\
                 not foundFrontBrake:

                #the brake acts to slow any movement
                frontBrake = numpy.sign(self.wheelSpeedFront)*(-self.front_axle_brake_effort*self.chassisCmds[cmd].frontBrake)
                chassisSt.frontBrake = self.chassisCmds[cmd].frontBrake
                chassisSt.frontBrakeCommander = self.chassisCmds[cmd].sender
                foundFrontBrake = True

              else:
                frontBrake = 0

          steer_ang_changed, center_y = self._ctrl_steering(steer_ang, steer_ang_vel, delta_t)
          self._ctrl_axles(speed, accel, delta_t, steer_ang_changed, center_y)

        # Publish the steering and axle joint commands.
        chassisSt.header.stamp = rospy.Time.now()
        self.chassisStatePub.publish(chassisSt)

        self._left_steer_cmd_pub.publish(self._theta_left)
        self._right_steer_cmd_pub.publish(self._theta_right)
        if self._left_front_axle_cmd_pub:
          self._left_front_axle_cmd_pub.publish(frontBrake)
        if self._right_front_axle_cmd_pub:
          self._right_front_axle_cmd_pub.publish(frontBrake)
        if self._left_rear_axle_cmd_pub:
          self._left_rear_axle_cmd_pub.publish(speed)
        if self._right_rear_axle_cmd_pub:
          self._right_rear_axle_cmd_pub.publish(speed)

        try:
          self._sleep_timer.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
          continue #rospy.loginfo()

  def chassisCmdCb(self, chassisCommand):
    """Chassis command callback

      :Parameters:
          chassisCommand : autorally_msgs.msg.chassisCommand
                         Position to set the control actuators
    """
    with self.chassisCmdLock:
      #self.lastCmdTime = rospy.get_time()
      self.chassisCmds[chassisCommand.sender] = chassisCommand
      self._last_cmd_time = rospy.get_time()

  def runstopCb(self, runstop):
    with self.runstopLock:
      self.runstops[runstop.sender] = runstop

  def getrunstop(self):
    with self.runstopLock:
      runstop = True
      for item in self.runstops:
        runstop &= self.runstops[item].motionEnabled
      return runstop

  def _get_front_wheel_params(self, side):
    # Get front wheel parameters. Return a tuple containing the steering
    # link name, steering controller name, axle controller name (or None),
    # and inverse of the circumference.

    prefix = "~" + side + "_front_wheel/"
    steer_link_name = rospy.get_param(prefix + "steering_link_name",
                                      side + "_steering_link")
    steer_ctrlr_name = rospy.get_param(prefix + "steering_controller_name",
                                       side + "_steering_controller")
    axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
    return steer_link_name, steer_ctrlr_name, axle_ctrlr_name, inv_circ

  def _get_rear_wheel_params(self, side):
    # Get rear wheel parameters. Return a tuple containing the link name,
    # axle controller name, and inverse of the circumference.

    prefix = "~" + side + "_rear_wheel/"
    link_name = rospy.get_param(prefix + "link_name", side + "_wheel")
    axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
    return link_name, axle_ctrlr_name, inv_circ

  def _get_common_wheel_params(self, prefix):
    # Get parameters used by the front and rear wheels. Return a tuple
    # containing the axle controller name (or None) and the inverse of the
    # circumference.

    axle_ctrlr_name = rospy.get_param(prefix + "axle_controller_name",
                                      None)

    try:
      dia = float(rospy.get_param(prefix + "diameter",
                                    self._DEF_WHEEL_DIA))
      if dia <= 0.0:
          raise ValueError()
    except:
      rospy.logwarn("The specified wheel diameter is invalid. "
                    "The default diameter will be used instead.")
      dia = self._DEF_WHEEL_DIA

    return axle_ctrlr_name, 1 / (pi * dia)

  def _get_link_pos(self, tfl, link):
    # Return the position of the specified link, relative to the right
    # rear wheel link.

    while True:
      try:
        trans, not_used = \
          tfl.lookupTransform(self._right_rear_link_name, link,
                              rospy.Time(0))
        return numpy.array(trans)
      except:
        pass

  def _ctrl_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
    # Control the steering joints.

    # Compute theta, the virtual front wheel's desired steering angle.
    if steer_ang_vel_limit > 0.0:
      # Limit the steering velocity.
      ang_vel = (steer_ang - self._last_steer_ang) / delta_t
      ang_vel = max(-steer_ang_vel_limit,
                     min(ang_vel, steer_ang_vel_limit))
      theta = self._last_steer_ang + ang_vel * delta_t
    else:
      theta = steer_ang

    # Compute the desired steering angles for the left and right front
    # wheels.
    center_y = self._wheelbase * math.tan((pi / 2) - theta)
    steer_ang_changed = theta != self._last_steer_ang
    if steer_ang_changed:
      self._last_steer_ang = theta
      self._theta_left = \
          _get_steer_ang(math.atan(self._inv_wheelbase *
                                     (center_y - self._joint_dist_div_2)))
      self._theta_right = \
          _get_steer_ang(math.atan(self._inv_wheelbase *
                                   (center_y + self._joint_dist_div_2)))

    return steer_ang_changed, center_y

  def _ctrl_axles(self, speed, accel_limit, delta_t, steer_ang_changed,
                  center_y):
    # Control the axle joints.

    # Compute veh_speed, the vehicle's desired speed.
    if accel_limit > 0.0:
      # Limit the vehicle's acceleration.
      self._last_accel_limit = accel_limit
      accel = (speed - self._last_speed) / delta_t
      accel = max(-accel_limit, min(accel, accel_limit))
      veh_speed = self._last_speed + accel * delta_t
    else:
      self._last_accel_limit = accel_limit
      veh_speed = speed

    # Compute the desired angular velocities of the wheels.
    if veh_speed != self._last_speed or steer_ang_changed:
      self._last_speed = veh_speed
      left_dist = center_y - self._joint_dist_div_2
      right_dist = center_y + self._joint_dist_div_2

      # Front
      gain = (2 * pi) * veh_speed / abs(center_y)
      r = math.sqrt(left_dist ** 2 + self._wheelbase_sqr)
      self._left_front_ang_vel = gain * r * self._left_front_inv_circ
      r = math.sqrt(right_dist ** 2 + self._wheelbase_sqr)
      self._right_front_ang_vel = gain * r * self._right_front_inv_circ
      # Rear
      gain = (2 * pi) * veh_speed / center_y
      self._left_rear_ang_vel = \
        gain * left_dist * self._left_rear_inv_circ
      self._right_rear_ang_vel = \
        gain * right_dist * self._right_rear_inv_circ

  def getWheelSpeed(self, data, name, dia):
    try:
      idx = data.name.index(name)
      #return data.twist[idx].angular.y * (dia)/2.0 #if data is from linkState message
      return data.velocity[idx]*(dia/2.0)
    except IndexError:
      rospy.logerror('modelStates does not contain ' + name)
      return 0.0


  def wheelSpeedsCb(self, data):
    ws = wheelSpeeds()
    ws.header.stamp = rospy.Time.now()
    ws.header.frame_id = 'gazeboSim'

    #print data
    ws.lfSpeed = self.getWheelSpeed(data, self.left_front_name, self.left_front_dia)
    ws.rfSpeed = self.getWheelSpeed(data, self.right_front_name, self.right_front_dia)
    self.wheelSpeedFront = (ws.lfSpeed+ws.rfSpeed)/2.0

    #published speeds can't be negative so data mimics the physical platform
    ws.lfSpeed = abs(ws.lfSpeed)
    ws.rfSpeed = abs(ws.rfSpeed)
    ws.lbSpeed = abs(self.getWheelSpeed(data, self.left_rear_name, self.left_rear_dia))
    ws.rbSpeed = abs(self.getWheelSpeed(data, self.right_rear_name, self.right_rear_dia))

    if self.wheelSpeedsPub:
      self.wheelSpeedsPub.publish(ws)


  def getJointStateWheelParams(self, lr_prefix, fr_prefix):
    dia = rospy.get_param(self._vehicle_prefix + '/autorally_controller/' + lr_prefix + '_' + fr_prefix + '_wheel/diameter')
    name = lr_prefix + '_' + fr_prefix + '_axle'
    return name, dia

  def getLinkStateFrontWheelParams(self, lr_prefix):
    dia = rospy.get_param(self._vehicle_prefix + '/autorally_controller/' + lr_prefix + '_front_wheel/diameter')
    name = 'autorally_platform::' + rospy.get_param(self._vehicle_prefix + '/autorally_controller/' + lr_prefix + '_front_wheel/steering_link_name')
    return name, dia

  def getLinkStateRearWheelParams(self, lr_prefix):
    dia = rospy.get_param('/autorally_platform/' + self._vehicle_prefix + 'autorally_controller/' + lr_prefix + '_rear_wheel/diameter')
    name = 'autorally_platform::' + rospy.get_param(self._vehicle_prefix + '/autorally_controller/' + lr_prefix + '_rear_wheel/link_name')
    return name, dia

  _DEF_WHEEL_DIA = 0.19    # Default wheel diameter. Unit: meter.
  _DEF_EQ_POS = 0.0       # Default equilibrium position. Unit: meter.
  _DEF_CMD_TIMEOUT = 0.5  # Default command timeout. Unit: second.
  _DEF_PUB_FREQ = 50.0    # Default publishing frequency. Unit: hertz.
# end _AutoRallyCtrlr


def _wait_for_ctrlr(list_ctrlrs, ctrlr_name):
  # Wait for the specified controller to be in the "running" state.
  # Commands can be lost if they are published before their controller is
  # running, even if a latched publisher is used.

  while True:
    response = list_ctrlrs()
    for ctrlr in response.controller:
      if ctrlr.name == ctrlr_name:
        if ctrlr.state == "running":
          return
        rospy.sleep(0.1)
        break


def _create_axle_cmd_pub(list_ctrlrs, axle_ctrlr_name):
  # Create an axle command publisher.
  if not axle_ctrlr_name:
    return None
  return _create_cmd_pub(list_ctrlrs, axle_ctrlr_name)


def _create_cmd_pub(list_ctrlrs, ctrlr_name):
  # Create a command publisher.
  _wait_for_ctrlr(list_ctrlrs, ctrlr_name)
  return rospy.Publisher(ctrlr_name + "/command", Float64, queue_size=1)


def _get_steer_ang(phi):
  # Return the desired steering angle for a front wheel.
  if phi >= 0.0:
    return (pi / 2) - phi
  return (-pi / 2) - phi


# main
if __name__ == "__main__":
  ctrlr = AutoRallyCtrlr()
  ctrlr.spin()
