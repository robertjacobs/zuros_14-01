server_config = {
  # The settings for the channel logging MySQL server / database / table
  'mysql_server':   '127.0.0.1',
  'mysql_user':     'zurosUser',
  'mysql_password': 'zuydrobotics',
  'mysql_db':       'Zuros',
  
  'mysql_sensor_table':    'Sensor',
  'mysql_sensor_types_table': 'SensorType',
  
  # Settings for the zwave sensor network
  'zwave_ip': 'http://192.168.1.109:80/api/devices'
}

""" Contains configuration information for each experiment site """
""" The 'sensors' element controlls which sensors classes are loaded when sensors.py is run """
""" The 'map' element is used to control the conversion between map coordinates and svg image coordinates """

locations_config = {
  	'ZAP': 
	{
		'sensors': ['ZWaveHomeController']
	}
}
