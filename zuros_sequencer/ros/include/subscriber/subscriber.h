/*
 * subscriber.h
 *
 *  Created on: Dec 16, 2013
 *      Author: robot
 */

#ifndef SUBSCRIBER_H_
#define SUBSCRIBER_H_

#include "ros/ros.h"
#include <zuros_sensors/Zwave.h>
#include "database/databaseConnector.h"

class ZwaveSubscriber
{
public:
		ZwaveSubscriber(ros::NodeHandle nh);
        void Init();
        void ReceiveCallback(const zuros_sensors::Zwave::ConstPtr& msg);

protected:
        ros::NodeHandle node_;
        ros::Subscriber string_subscriber_;
        MysqlConnector _mysqlConnector;
};

#endif /* SUBSCRIBER_H_ */
