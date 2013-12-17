server_config = {
  # The settings for the zwavePoller script
  'mysql_server'                 : '127.0.0.1',
  'mysql_user'                   : 'zurosUser',
  'mysql_password'               : 'zuydrobotics',
  'mysql_db'                     : 'Zuros',
  'mysql_sensor_table_name'      : 'Sensor',
  'mysql_sensor_types_table_name': 'SensorType'
}

message_config = {
  'message_status'               : 'ZWAVE_STATUS',
  'message_sensors'              : 'ZWAVE_SENSORS'
}
