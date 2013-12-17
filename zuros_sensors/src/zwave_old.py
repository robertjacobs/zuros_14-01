import sys
from socket import AF_INET, SOCK_DGRAM, socket, timeout
import json, urllib2, base64
import time

#ROS stuff
import rospy
from zuros_sensors.msg import Zwave

#polling processor made by UH
#https://github.com/uh-adapsys/UHCore/blob/master/Core/extensions.py
from include.extensions import PollingProcessor
                
from include.database import SensorsInDatabase
################################################################################
#
# ZWave thread
#
# Listens to the ZigBee gateway's UDP broadcast messages, transforms the
# channel values according to the specified sensor kind and puts them in
# the channel array.
#
################################################################################
class ZWaveHomeController(PollingProcessor):

	# Initialisation method
	def __init__ (self, ipAddress):
		super(ZWaveHomeController, self).__init__()
		
		#url of the ZWAVE controller
		self._baseUrl = ipAddress
		
		#get the sensor definitions from the database
		self._sensors = SensorsInDatabase().GetAllSensors()
		#get the sensorTypes definitions from the database
		self._sensorTypes = SensorsInDatabase().GetAllSensorTypes()
		
		#warnings and info regarding sensors gets placed inside these arrays so that terminal does not get flooded
		self._warned = []
		self._info = []

		#holds the added sensors (the ones which are also in the database)
		self._sensorsAdded = []

		self.pub = rospy.Publisher('ZWAVE', Zwave)

	def start(self):
		
		rospy.init_node('zwave_publisher')
		print "Started polling zwave sensors"
		
		try:
			# http://192.168.1.109/api/devices
			url = self._baseUrl
			request = urllib2.Request(url)
			base64string = base64.encodestring('%s:%s' % ('admin', 'admin')).replace('\n', '')
			request.add_header("Authorization", "Basic %s" % base64string)
			result = urllib2.urlopen(request)
			data = json.load(result) 
		except Exception as e:
			if id(self) + type(e) not in self._warned:
				print >> sys.stderr, "Error while receiving data from ZWaveHomeController: %s" % e
				self._warned.append(id(self) + type(e))
			return
		
		for device in data:
			#Format the communication ID to ZWAVE:[id]
			communication_id = "ZWAVE:" + str(device['id'])

			try:
				#find the sensor with the same communication id
				sensor = next(s for s in self._sensors if str(s['communication_id']) == str(communication_id))		
				# Only warn once, or we'll flood the console
				if(sensor not in self._sensorsAdded):
					print("[INFO] Found a known sensor: %s") % (sensor['name'])
					self._sensorsAdded.append(sensor)
				
			except StopIteration:
				# Only warn once, or we'll flood the console
				if communication_id not in self._warned:
					print "[WARNING] Sensor with name '%s' was not recognised because it is not defined in the database" % str(device['name'])
					self._warned.append(communication_id)
				continue	
		
		self._addPollingProcessor('zwave', self.pollZWaveSensors, None, 0.1)

	def stop(self):
		print "Stopped polling zwave sensors"
		self._removePollingProcessor('zwave')
		
	def pollZWaveSensors(self):
		try:
			# http://192.168.1.109/devices
			url = self._baseUrl
			request = urllib2.Request(url)
			base64string = base64.encodestring('%s:%s' % ('admin', 'admin')).replace('\n', '')
			request.add_header("Authorization", "Basic %s" % base64string)
			result = urllib2.urlopen(request)
			data = json.load(result)
		except Exception as e:
			if id(self) + type(e) not in self._warned:
				print >> sys.stderr, "Error while receiving data from ZWaveHomeController: %s" % e
				self._warned.append(id(self) + type(e))
				return

		for device in data:
			try:
				#Format the communication ID to ZWAVE:[id]
				communication_id = "ZWAVE:" + str(device['id'])
				
				#Check all the added sensors
				for sensor in self._sensorsAdded:
					#if the sensor communication id is the same as the id of the device in the ZWAVE controller, check its status.
					if(str(sensor['communication_id']) == str(communication_id)):
						#did the value of the device change?
						if(str(sensor['value']) != str(device['properties']['value'])):
							#sensortypes are being used to interpret the value. For example 0 can be 0 (analog) or 0 (as in off)
							for type in self._sensorTypes:
								#We will need to check which sensorType our sensor has in order to interpret the value
								if(str(sensor['sensorType']) == str(type['id'])):
									#Add the uninterpreted value into the value field
									sensor['value'] = device['properties']['value']
									#Set the last_interpreted value to the current one
									sensor['last_interpreted_value'] = sensor['interpreted_value']
									#set the last updated datetime
									sensor['lastUpdated'] = time.strftime('%Y-%m-%d %H:%M:%S')
									
									#check if we have an analog sensor or not
									if(type['onValue'] != None and type['offValue'] != None):
										if(str(device['properties']['value']) == "0"):
											#interpreted value - set to value set in database
											sensor['interpreted_value'] = type['offValue']
																
										elif (str(device['properties']['value']) == "1"):
											##interpreted value - set to value set in database
											sensor['interpreted_value'] = type['onValue']
									#we had an analog sensor, so we will write the raw value into the value
									else:
										sensor['interpreted_value'] = device['properties']['value']
							
							success = SensorsInDatabase().UpdateSensorValue(sensor)
							print "Updated sensor log for %(name)s to %(value)s" % {'name':sensor['name'],'value': sensor['interpreted_value'],}
					
							self.pub.publish(Zwave(name=sensor['name'],value=sensor['value'],communication_id=communication_id))
							
			except Exception as e:
				print >> sys.stderr, "Error while updating sensor information: %s" % e
				return			

if __name__ == '__main__':
	import config
	
	#array which holds the different sensors
	sensorList = []
	
	#find sensors in location named ZAP in config file
	for sensorType in config.locations_config['ZAP']['sensors']:
		sensor = None
		
		#is there a sensor with the name "ZWaveHomeController"?
		if sensorType == 'ZWaveHomeController':
			sensor = ZWaveHomeController(config.server_config['zwave_ip'])
	
		#if we found a sensor, add it to the list
		if sensor != None:
			sensorList.append(sensor)
	
	#for each sensor in the list, start the polling
	for sensor in sensorList:
		sensor.start()
	
	#wait for keyboard interrupt
	while True:
		try:
			sys.stdin.read()
		except KeyboardInterrupt:
			break
	
	#if there was a keyboard interrupt, stop all polling before shutdown
	for sensor in sensorList:
		sensor.stop()
