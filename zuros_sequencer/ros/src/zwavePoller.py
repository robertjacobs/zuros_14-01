#!/usr/bin/env python
import sys
from socket import AF_INET, SOCK_DGRAM, socket, timeout
import json, urllib2, base64
import time

#ROS stuff
import rospy
import roslib
roslib.load_manifest('zuros_sensors')
from zuros_sensors.msg import Zwave

#polling processor made by UH
#https://github.com/uh-adapsys/UHCore/blob/master/Core/extensions.py
from include.extensions import PollingProcessor
                
from include.database import SensorsInDatabase

class SensorHandler(self):
	def __init__(self):
		#get the sensor definitions from the database
		self._sensors = SensorsInDatabase().GetAllSensors()
		#get the sensorTypes definitions from the database
		self._sensorTypes = SensorsInDatabase().GetAllSensorTypes()
	
	def Callback(data):
		sensor = None
		sensor['value'] = data.value
		sensor['communication_id'] = data.communication_id
		sensor['name'] = data.name
	
if __name__ == '__main__':
	import config
	
	handler = SensorHandler()
	
	rospy.init_node('zwavePoller', anonymous=False)
	rospy.Subscriber(config.message_name, Zwave, handler.Callback)
	rospy.spin()
