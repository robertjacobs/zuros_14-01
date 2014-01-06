import sys
import MySQLdb

#ROS stuff
import rospy

class SensorsInDatabase(object):
	def __init__(self):
		try:
			parameters = rospy.get_param("/zuros/zuros_sequencer/config/")
		except:
			rospy.loginfo(rospy.get_name() + " Parameters not set. Please run the folllowing console command: rosrun zuros_sequencer set_parameters.py")
			sys.exit("Parameters not set. Please run the folllowing console command: rosrun zuros_sequencer set_parameters.py")
	
		self._sensorTable = parameters['database_config']['mysql_sensor_table_name']
		self._sensorTypesTable = parameters['database_config']['mysql_sensor_types_table_name']
		self._sql = Database()

	def GetAllSensors(self):
		query = "SELECT * FROM %s" % self._sensorTable
		return self._sql.ExecuteAndReturn(query)
        
	def GetAllSensorTypes(self):
		query = "SELECT * FROM %s" % self._sensorTypesTable
		return self._sql.ExecuteAndReturn(query)
        
	def UpdateSensorValue(self, sensor):
		return self._sql.UpdateSensorValue(sensor)

class Database(object):
	def __init__(self, hostname=None, username=None, password=None, database=None):
		try:
			parameters = rospy.get_param("/zuros/zuros_sequencer/config/")
		except:
			rospy.loginfo(rospy.get_name() + " Parameters not set. Please run the folllowing console command: rosrun zuros_sequencer set_parameters.py")
			sys.exit("Parameters not set. Please run the folllowing console command: rosrun zuros_sequencer set_parameters.py")

		self._host = hostname or parameters['database_config']['mysql_server']
		self._pass = password or parameters['database_config']['mysql_password']
		self._user = username or parameters['database_config']['mysql_user']
		self._db_url = database or parameters['database_config']['mysql_db']
		self._db = MySQLdb.connect(self._host, self._user, self._pass, self._db_url)
                
	def OpenConnection(self):
		self._openConnection = True

	def Execute(self, query):
		cursor = self._db.cursor()
		cursor.execute(query)
		self._db.commit()
        
	def ExecuteAndReturn(self, query):
		cursor = self._db.cursor(MySQLdb.cursors.DictCursor)
		cursor.execute(query)
		return cursor.fetchall()
        
	def UpdateSensorValue(self, sensor):
		query = "UPDATE Sensor SET value='%s', interpreted_value='%s', last_interpreted_value='%s', lastUpdated='%s' WHERE id='%s' " % (
																																sensor['value'],
																																sensor['interpreted_value'],
																																sensor['last_interpreted_value'],
																																sensor['lastUpdated'],
																																sensor['id'])
		try:
			self.Execute(query)
			return True
		except Exception as e:
			print "[ERROR] - exception raised while updating sensor information in database: %s" % e
			return False
