import MySQLdb

class SensorsInDatabase(object):
	def __init__(self):
		from config import server_config
		self._sensorTable = server_config['mysql_sensor_table']
		self._sensorTypesTable = server_config['mysql_sensor_types_table']
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
		from config import server_config
		self._host = hostname or server_config['mysql_server'] 
		self._pass = password or server_config['mysql_password']
		self._user = username or server_config['mysql_user']
		self._db_url = database or server_config['mysql_db']
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
		query = "UPDATE Sensor SET value='%s', interpreted_value='%s', last_interpreted_value='%s', lastUpdated='%s' WHERE id='%s' " %  (
																																 sensor['value'], 
																															     sensor['interpreted_value'], 
																															     sensor['last_interpreted_value'], 
																															     sensor['lastUpdated'], 
																															     sensor['id']
																																)
		try:
			self.Execute(query)
			return True
		except Exception as e:
			print "[ERROR] - exception raised while updating sensor information in database: %s" % e
			return False
