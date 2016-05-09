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

## @package systemStatus
#  Gets the wireless signal strength, power status, and CPU temperature. This information is published in the form of ROS messages.
#

#import roslib; roslib.load_manifest('autorally_msgs')
import rospy
import commands
import subprocess
import signal
import sys
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

running = True

def signal_handler(signal, frame):
        print('SystemStatus: SIGINT detected.')
        running = False

## WirelessStatus.
#
#  Uses iwconfig to retrieve wireless signal strength and then publishes it.
class WirelessStatus:
    ##  Initializes wirelessSignalStatusMSG
    def __init__(self):
        self.linkQuality = 0
        self.maxQuality = 0
    ##  Retrieves wireless signal strength and stores it in the MSG format.
    def getValues(self):
        outputString = commands.getoutput("sudo iwconfig | grep 'Link Quality='")
        linkQualityLocation = outputString.find("Link Quality=")
        if linkQualityLocation >= 0 and outputString[linkQualityLocation+13:linkQualityLocation+15].isdigit() and outputString[linkQualityLocation+16:linkQualityLocation+18].isdigit():
            self.linkQuality = int(outputString[linkQualityLocation+13:linkQualityLocation+15])
            self.maxQuality = int(outputString[linkQualityLocation+16:linkQualityLocation+18])

## PowerStatus.
#
#  Uses acpi to retrieve battery status and then publishes it.
class PowerStatus:
    ##  Initializes powerStatusMSG
    def __init__(self):
        self.percentage = 0
        self.valid = True
    ##  Retrieves battery status and stores it in the MSG format.
    def getValues(self):
        outputString = commands.getoutput("acpi")
        if outputString == "No support for device type: power_supply":
            self.valid = False
        else:
            self.valid = True
        percentageLocation = outputString.find("%")
        if percentageLocation >= 0:
            self.percentage = int(outputString[percentageLocation-2:pPubercentageLocation])

## M4ATXPowerStatus
#
#  Uses M4API to check compute box battery status and power supply diagnostics
class M4ATXPowerStatus:
    def __init__(self):
        self.percentage = 0
        self.VIN = 0
        self.V33 = 0
        self.V5  = 0
        self.V12 = 0
        self.temp = 0
        self.valid = True
    def getValues(self):
        if not self.valid:
            return
        try:
            output = subprocess.check_output("sudo m4ctl", shell=True)
        except subprocess.CalledProcessError, e:
            self.valid = False
            if e.returncode is 127:
                print "m4ctl gave error code 127"
            else :
                print "Unknown error code from m4ctl: ", e.returncode
        else:
            for line in output.split("\n"):
                tokens = line.split("\t")
                if len(tokens) is 2:
                    if tokens[0] == "VIN:":
                        self.valid = True
                        self.VIN = float(tokens[1])
                        # Battery max voltage 25, fully depleted 19, 6 volt span
                        self.percentage = min( 100, ( ( float(tokens[1]) - 19 ) / 6.0 ) * 100)
                    if tokens[0] == "33V:":
                        self.valid = True
                        self.V33 = float(tokens[1])
                    if tokens[0] == "5V:":
                        self.valid = True
                        self.V5 = float(tokens[1])
                    if tokens[0] == "12V:":
                        self.valid = True
                        self.V12 = float(tokens[1])
                    if tokens[0] == "TEMP:":
                        self.valid = True
                        self.temp = float(tokens[1])

## TempStatus.
#
#  Uses sensors to retrieve CPU temperature and then publishes it.
class TempStatus:
    ##  Initializes tempStatusMSG
    def __init__(self):
        self.cpuTemp = 0
        self.fanSpeed = 0
    ##  Retrieves CPU temperature and fan speed and stores it in the MSG format.
    def getValues(self):
        outputString = commands.getoutput("sensors | grep \"Core 0:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp1 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp1 = -1.0
        else:
            self.cpuTemp1 = 0.0
        outputString = commands.getoutput("sensors | grep \"Core 1:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp2 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp2 = -1.0
        else:
            self.cpuTemp2 = 0.0
        outputString = commands.getoutput("sensors | grep \"Core 2:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp3 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp3 = -1.0
        else:
            self.cpuTemp3 = 0.0
        outputString = commands.getoutput("sensors | grep \"Core 3:\"")
        cpuTempLocation = outputString.find("+")
        if cpuTempLocation >= 0:
            try:
                self.cpuTemp4 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
            except ValueError:
                self.cpuTemp4 = -1.0
        else:
            self.cpuTemp4 = 0.0
        self.cpuTemp = max(self.cpuTemp1, self.cpuTemp2, self.cpuTemp3, self.cpuTemp4)
        outputString = commands.getoutput("sensors | grep \"fan2:\"")
        try:
            self.fanSpeed = float(outputString[23:27])
        except ValueError:
            self.fanSpeed = -1.0

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('systemStatus')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    array = DiagnosticArray()
    status = DiagnosticStatus(name='SystemStatus',\
                              level=0,\
                              message='System Status')

    array.status = [status]

    if  not rospy.has_param('/systemStatus/cpuTempHigh') or\
        not rospy.has_param('/systemStatus/cpuTempCrit') or\
        not rospy.has_param('/systemStatus/batteryLow') or\
        not rospy.has_param('/systemStatus/batteryCrit') or\
        not rospy.has_param('/systemStatus/wirelessLow') or\
        not rospy.has_param('/systemStatus/wirelessCrit'):
        rospy.logerr("Could not get all SystemStatus parameters");

    wirelessStatusPublisher = WirelessStatus()
    powerStatusPublisher = PowerStatus()
    tempStatusPublisher = TempStatus()
    powerSupplyHandler = M4ATXPowerStatus()

    rate = rospy.Rate(0.5) #1Hz
    while not rospy.is_shutdown() and running:
        status.values = []

        wirelessStatusPublisher.getValues()
        powerStatusPublisher.getValues()
        tempStatusPublisher.getValues()
        powerSupplyHandler.getValues()

        if powerStatusPublisher.valid:
            status.values.append(KeyValue(key='Laptop Battery Level', value=str(powerStatusPublisher.percentage)))
            if powerStatusPublisher.percentage <= rospy.get_param('/systemStatus/batteryCrit'):
                status.values.append(KeyValue(key='BATTERY CRITICALLY LOW', value=chr(2)))
            elif powerStatusPublisher.percentage <= rospy.get_param('/systemStatus/batteryLow'):
                status.values.append(KeyValue(key='BATTERY LOW', value=chr(1)))

        if powerSupplyHandler.valid:
            status.values.append(KeyValue(key='Battery Level', value=str(powerSupplyHandler.percentage)))
            if powerSupplyHandler.percentage <= rospy.get_param('/systemStatus/batteryCrit'):
                status.values.append(KeyValue(key='BATTERY CRITICALLY LOW', value=chr(2)))
            elif powerSupplyHandler.percentage <= rospy.get_param('/systemStatus/batteryLow'):
                status.values.append(KeyValue(key='BATTERY LOW', value=chr(1)))
            status.values.append(KeyValue(key='VIN', value=str(powerSupplyHandler.VIN)))
            status.values.append(KeyValue(key='3.3v Rail', value=str(powerSupplyHandler.V33)))
            status.values.append(KeyValue(key='5v Rail', value=str(powerSupplyHandler.V5)))
            status.values.append(KeyValue(key='12v Rail', value=str(powerSupplyHandler.V12)))
            status.values.append(KeyValue(key='Power Supply Temp', value=str(powerSupplyHandler.temp)))
            
        status.values.append(KeyValue(key='CPU Temp', value=str(tempStatusPublisher.cpuTemp)))
        if tempStatusPublisher.cpuTemp >= rospy.get_param('/systemStatus/cpuTempCrit'):
            status.values.append(KeyValue(key='CPU CRITICALLY HOT', value=chr(2)))
        elif tempStatusPublisher.cpuTemp >= rospy.get_param('/systemStatus/cpuTempHigh'):
            status.values.append(KeyValue(key='CPU HOT', value=chr(1)))
        status.values.append(KeyValue(key='Fan Speed', value=str(tempStatusPublisher.fanSpeed)))

        # WiFi status is not currently published by our driver
        status.values.append(KeyValue(key='WiFi Quality', value=str(wirelessStatusPublisher.linkQuality)))
        status.values.append(KeyValue(key='WiFi Max Quality', value=str(wirelessStatusPublisher.maxQuality)))
        if wirelessStatusPublisher.linkQuality <= rospy.get_param('/systemStatus/wirelessCrit'):
            status.values.append(KeyValue(key='WiFi CRITICALLY WEAK', value=chr(2)))
        elif wirelessStatusPublisher.linkQuality <= rospy.get_param('/systemStatus/wirelessLow'):
            status.values.append(KeyValue(key='WiFi WEAK', value=chr(1)))
        pub.publish(array)
        rate.sleep()
    print("SystemStatus: Shutting down");
