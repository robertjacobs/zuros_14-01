#!/usr/bin/env python

## @package set_parameters.py
# This module handles the initial setup of the config parameters configured in the parameter server.
# 	Run this script before running sensors.py because sensors.py relies on the parameters set by this script
# 
# Robert Jacobs 2014

import roslib; roslib.load_manifest('zuros_sensors')
import rospy
from time import sleep

if __name__ == "__main__":
	rospy.init_node("zuros_sensors_set_parameters")
	
	print "Hello, this is the set_parameters node."
	print "I will now write the parameters entered in this file to the parameter server."

	# Configure parameters
	sensor_config = {
		'sensor_config': 
		{
			"zwave" : 
			{
				"server_address" : "http://192.168.1.109:80/api/devices" 
			}
		}, 

		'locations': 
		{
			'ZAP': 
			{
				'sensors': 'ZWaveHomeController'
			}
		}
	}
	rospy.set_param("/zuros/zuros_sensors/config", sensor_config)

	sleep (1)
	print "I am done. Bye."
