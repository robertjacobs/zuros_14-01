/*
 * databaseConnector.h
 *
 *  Created on: Dec 16, 2013
 *      Author: robot
 */

#ifndef DATABASECONNECTOR_H_
#define DATABASECONNECTOR_H_

#include <mysql/mysql.h>
#include <stdio.h>

class MysqlConnector
{
	public:
		MysqlConnector();
		void Init(const char * server, const char * username, const char * password, const char * database);
		void OpenConnection();
		void CloseConnection();
		void Query(const char * query);
	private:
		MYSQL _connection;
		const char * _server;
		const char * _username;
		const char * _password;
		const char * _database;
};

#endif /* DATABASECONNECTOR_H_ */
