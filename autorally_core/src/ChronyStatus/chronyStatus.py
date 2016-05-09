#!/usr/bin/env python
# Software License Agreement (BSD License)
# Copyright (c) 2013, Georgia Institute of Technology
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

Gets the current state of all chrony clients. The reported value is the
predicted offset between each client and the chrony server. This program can
be run on any computer connected into the system (offloaded to OCS to save CPU
on the main computer).
"""

import os
import socket
import sys
import roslib
import rospy
import commands
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from subprocess import call
from subprocess import check_output
import subprocess

"""
@param host the hostname for the chrony server
@return clients a list of all clients using host for clock syncronization
Contacts the chrony server in the system and parses out a list of all clients
currently using it as their clock synchronization source.
"""
def getClients(host):
  try:
    clientsText = check_output("chronyc -h " + host + " -m 'password muri123' clients", shell=True);
    
    #print clients.rfind('=')
    clientsText = clientsText[clientsText.rfind('=')+1:]
    lines = clientsText.split('\n')

    clients = []
    for l in lines:
      if len(l) and l.find('localhost'):
        #print l.split()[0].rstrip('.local')
        clients.append(l.split()[0].rstrip('.local'))
    return clients
  except subprocess.CalledProcessError as e:
    return 'subprocess error:' + e.output
  

"""
@param clients list of all clients currently in the system
@param status the diagnostic message to pupulate with offset information

Gets the offset of each client in the system and created the diagnostic message
"""
def getOffsets(clients, host, status):
  status.values = []
  for c in clients:
    clientName = c
    if c == socket.gethostname():
      c = 'localhost'
    try:
      #print 'pinging', c
      clientsOffset = check_output("chronyc -h " + c + " sourcestats", shell=True);
      
      #print clientsOffset
      clientsOffset = clientsOffset[clientsOffset.rfind('=')+1:].strip()
      data = clientsOffset.split('\n')
      
      for s in data:
        splitLine = s.split()
        #print 'looking for', host, 'in', splitLine
        if splitLine[0] == host:
          status.values.append(KeyValue(key=clientName, value=splitLine[6]))    
    except:
      print sys.exc_info()[0]
    

if __name__ == '__main__':
  rospy.init_node('chronyStatus')
  pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=2)
  array = DiagnosticArray()
  status = DiagnosticStatus(name='ChronyStatus',\
                            level=0,\
                            message='Master:'+os.environ.get('MASTER_HOSTNAME'))

  host = os.environ.get('MASTER_HOSTNAME')
  if socket.gethostname() == host:
    host = 'localhost'

  array.status = [status]

  rate = rospy.Rate(0.2)
  while not rospy.is_shutdown():

    clients = getClients(host)
    #print clients
    if len(clients) > 0 and 'subprocess error' not in clients:
      getOffsets(clients, host, status)
    elif 'subprocess error' in clients:
      status.values = []
      status.values.append(KeyValue(key=clients, value=chr(1)) )
    else:
      status.values = []
      status.values.append(KeyValue(key='No Clients', value=chr(1)) )

    pub.publish(array)
    rate.sleep()
