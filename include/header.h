#pragma once
#ifndef HEADER_H
#define HEADER_H

#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Header.h>
#include <sensor_msgs/Joy.h>
#include <can_msgs/Frame.h>

#include <thread>

class ParamServer
{
public:
	ros::NodeHandle nh;

	std::string PROJECT_NAME;


	ParamServer()
	{
		nh.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "joy_control");


	}
};
#endif // HEADER_H