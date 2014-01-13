/*
* example_motor_controller.cpp
*
* Created on: Jan 13, 2014
* Author: Robert Jacobs
*/

#include <zuros_threemxl/threemxlController.h>
#include <sstream>
#include <threemxl/C3mxlROS.h>

threemxlController::threemxlController(ros::NodeHandle nh)
{
        node_ = nh;
}

void threemxlController::init()
{
	// Threemxl init
	CDxlConfig *config = new CDxlConfig();
	motor_ = new C3mxl();
	serial_port_.port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
	serial_port_.set_speed(LxSerial::S921600);
	motor_->setSerialPort(&serial_port_);

	motor_->setConfig(config->setID(107));
	motor_->init(false);
	motor_->set3MxlMode(SPEED_MODE);

	delete config;
}

void threemxlController::spin()
{
	ROS_INFO("SPIN");
	ros::Rate loop_rate(1); //herz;

  	while (ros::ok())
  	{
		motor_->setAcceleration(10);
		motor_->setSpeed(5);
    	loop_rate.sleep();
  	}	
  
	motor_->setAcceleration(1);
	motor_->setSpeed(0);
  	//motor_->setTorque(0);
}

void threemxlController::receiveCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("[%i]", msg->data);
}


        /**
        * The subscribe() call is how you tell ROS that you want to receive messages
        * on a given topic. This invokes a call to the ROS
        * master node, which keeps a registry of who is publishing and who
        * is subscribing. Messages are passed to a callback function, here
        * called receiveCallback. subscribe() returns a Subscriber object that you
        * must hold on to until you want to unsubscribe. When all copies of the Subscriber
        * object go out of scope, this callback will automatically be unsubscribed from
        * this topic.
        *
        * The second parameter to the subscribe() function is the size of the message
        * queue. If messages are arriving faster than they are being processed, this
        * is the number of messages that will be buffered up before beginning to throw
        * away the oldest ones.
        */
        //string_subscriber_ = node_.subscribe("zuros_threemxl", 10, &threemxlController::receiveCallback, this);
