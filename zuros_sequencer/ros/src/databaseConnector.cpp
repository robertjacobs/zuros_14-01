/*
 * databaseConnector.cpp
 *
 *  Created on: Dec 16, 2013
 *      Author: robot
 */

#include "database/databaseConnector.h"
#include "ros/ros.h"
#include "mysql/mysql.h" // MySQL Include File

MysqlConnector::MysqlConnector()
{
}

void MysqlConnector::Init(const char * server, const char * username, const char * password, const char * database)
{
	 mysql_init(&_connection);
	_server = server;
	_username = username;
	_password = password;
	_database = database;
}

void MysqlConnector::Query(const char * query)
{
}

void MysqlConnector::OpenConnection()
{
	mysql_real_connect(&_connection,_server,_username,_password,_database,0,NULL,0);

	//Check if we could make a connection to the database
	//if(_connection)
	//{
	//	ROS_INFO("Sequence connected to database");
	//}
	//else
	//{
	//	ROS_ERROR("Sequence could not connect to database");
	//}
}

void MysqlConnector::CloseConnection()
{

}






