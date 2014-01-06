#!/usr/bin/env python

import roslib; roslib.load_manifest('zuros_sensors')
import rospy
from time import sleep
if __name__ == "__main__":
	rospy.init_node("zuros_sensors_set_parameters")
	
	print "Hello, this is the set_parameters node."
	print "I will now write the parameters entered in this file to the parameter server."

	# Settings for the zwave sensor network
	sensor_config = {'sensor_config': {"zwave" : {"server_address" : "http://192.168.1.109:80/api/devices" }}, 'locations': {'ZAP': {'sensors': 'ZWaveHomeController'}}}
	rospy.set_param("/zuros/zuros_sensors/config", sensor_config)

	sleep (1)
	print "I am done. Bye."
