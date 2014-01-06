#!/usr/bin/env python

import roslib; roslib.load_manifest('zuros_sequencer')
import rospy
from time import sleep
if __name__ == "__main__":
	rospy.init_node("zuros_sequencer_set_parameters")
	
	print "Hello, this is the set_parameters node."
	print "I will now write the parameters entered in this file to the parameter server."

	# Settings for the zwave sensor network
	zwave_config = {'config' : {"database_config" : {"mysql_server" : "127.0.0.1", "mysql_user" : "zurosUser", "mysql_password" : "zuydrobotics", "mysql_db" : "Zuros", "mysql_sensor_table_name" : "Sensor", "mysql_sensor_types_table_name" : "SensorType"}, "message_config" : {"message_status" : "ZWAVE_STATUS", "message_sensors" : "ZWAVE_SENSORS"}}}

	rospy.set_param("/zuros/zuros_sequencer", zwave_config)

	sleep (1)
	print "I am done. Bye."
