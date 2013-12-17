server_config = {
  # Settings for the zwave sensor network
  'zwave_ip': 'http://192.168.1.109:80/api/devices'
}

""" Contains configuration information for each experiment site """
""" The 'sensors' element controlls which sensors classes are loaded when sensors.py is run """

locations_config = {
  	'ZAP': 
	{
		'sensors': ['ZWaveHomeController']
	}
}
