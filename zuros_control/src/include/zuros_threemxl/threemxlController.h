/*
* 3mxlController.h
*
* Created on: Jan 13, 2014
* Author: Robert Jacobs
*/

#ifndef ZUROS_3MXL_CONTROLLER_H_
#define ZUROS_3MXL_CONTROLLER_H_

#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <threemxl/C3mxlROS.h>

class threemxlController
{
public:
	threemxlController(ros::NodeHandle nh);
	void init();
	void spin();
	void receiveCallback(const std_msgs::Int32::ConstPtr& msg);

protected:
	ros::NodeHandle node_;
	ros::Subscriber string_subscriber_;
	CDxlGeneric *motor_;
	LxSerial serial_port_; ///< Serial port interface
};


#endif /* ZUROS_3MXL_CONTROLLER_H_ */
