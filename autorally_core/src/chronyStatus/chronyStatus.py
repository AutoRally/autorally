#!/usr/bin/env python
# Software License Agreement (BSD License)
# Copyright (c) 2016, Georgia Institute of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
@package chronyStatus

Acquires and publishes to /diagnostics the current state of chrony time sychronization
and information about available timesyn sources. On startup the node verifies that the
installed version of chrony is at least chronyMinVersion.
"""

import os
import socket
import rospy
import commands
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from subprocess import check_output
import subprocess

"""
@return chrony version number, if available

Get current chrony version number by using chronyc interface
"""
def checkChronyVersion():
  try:
    versionText = check_output("chronyc -v", shell=True);
    lines = versionText.split(' ')
    if lines[2] == 'version':
      return lines[3]

  except subprocess.CalledProcessError as e:
    rospy.logerr('chronyStatus: subprocess error:' + e.output)
  except ValueError:
    rospy.logerr('chrony version check failed, version unkown')

"""
@param status the diganostic array to add information to

Queries and adds to diganostics the current tracking status of chronyd using chronyc
"""
def getTracking(status):
  try:
    trackingText = check_output("chronyc tracking", shell=True);

    for line in trackingText.split('\n'):
      if len(line):
        #split on first : to separate data field name from value because some values can have : in them
        info = line.split(':', 1)
        status.values.append(KeyValue(key=info[0], value=info[1]))
    
  except subprocess.CalledProcessError as e:
    rospy.logerr(e.output)
    status.values.append(KeyValue(key=e.output, value=chr(2)))

"""
@param status the diganostic array to add information to

Queries and adds to diagnostics the current sources information from chronyd using chronyc
"""
def getSources(status):
  try:
    sourcesText = check_output("chronyc sources", shell=True);
    lines = sourcesText.split('\n')
    status.level = 1
    status.message = "Not Synchronized"
    for line in lines[3:]:
      if len(line):
        tok = line.split()
        text = 'ModeState:' + tok[0] + ' Stratum:' + tok[2] + ' Poll:' + tok[3] + ' Reach:' + tok[4] +\
               ' LastRx:' + tok[5] + ' Last Sample:' + ''.join(tok[6:])
        status.values.append(KeyValue(key='source '+tok[1], value=text))    
        #M = tok[0][0]
        #S = tok[0][1]
        #all is good if we are synchronizing to a source
        if tok[0][1] == '*':
          status.level = 0
          status.message = "Synchronized to " + tok[1]
        #print M, S
    
  except subprocess.CalledProcessError as e:
    rospy.logerr(e.output)
    status.values.append(KeyValue(key=e.output, value=chr(2)))


if __name__ == '__main__':
  hostname = socket.gethostname()
  rospy.init_node('chronyStatus_'+hostname)
  pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1, latch=True)
  array = DiagnosticArray()
  status = DiagnosticStatus(name='chronyStatus',\
                            level=0,\
                            hardware_id=hostname)

  array.status = [status]
  rate = rospy.Rate(0.2) # query and publish chrony information once every 5 seconds
  
  chronyVersion = checkChronyVersion()
  #chronyMinVersion = 1.29
  
  #publish error and exit if chronyMinVersion is not satisfied
  #if chronyVersion < chronyMinVersion:
  #  rospy.logerr('ChronyStatus requires chrony version ' + str(chronyMinVersion) + \
  #               ' or greater, version ' + str(chronyVersion) + ' detected, exiting')

  #else:
  while not rospy.is_shutdown():

    status.values = []
    status.values.append(KeyValue(key='chrony version', value=chronyVersion) )

    getTracking(status)
    getSources(status)

    pub.publish(array)
    rate.sleep()
