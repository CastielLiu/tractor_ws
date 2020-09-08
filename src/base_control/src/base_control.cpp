#include"base_control/steering_motor.h"
#include<driverless_msgs/ControlCmd.h>
#include<ros/ros.h>
#include<serial/serial.h>
#include<iostream>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<string>


#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#define __NAME__ "base_control_node"

class BaseControl
{
  public:
	BaseControl();
	~BaseControl();
	bool init();
	void run();
  private:
	void cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd);
	SteerMotor steerMotor_;
	std::string steerMotor_port_name_;
	
	ros::Subscriber sub_cmd_;
	ros::Publisher pub_msgs_;
	driverless_msgs::ControlCmd cmd_;
};

BaseControl::BaseControl()
{
	
}

BaseControl::~BaseControl()
{
	steerMotor_.stopReadSerial();
}


bool BaseControl::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("steerMotor_port_name",steerMotor_port_name_,"/dev/ttyUSB0");
	
	float road_wheel_angle_offset, road_wheel_angle_resolution;
	nh_private.param<float>("road_wheel_angle_offset",road_wheel_angle_offset,0.0);
	nh_private.param<float>("road_wheel_angle_resolution",road_wheel_angle_resolution,180.0/4096);
	steerMotor_.setRoadWheelAngleOffset(road_wheel_angle_offset);
	steerMotor_.setRoadWheelAngleResolution(road_wheel_angle_resolution);
	
	if(!steerMotor_.init(steerMotor_port_name_,115200))
	{
	    ROS_ERROR("[%s] init steering motor serial port failed.");
		return false;
	}
	steerMotor_.enable(); //?
	steerMotor_.startReadSerial();
	
	ROS_INFO("[%s] waiting for steering motor enable...",__NAME__);
	
	while(ros::ok() && !steerMotor_.is_enabled())
	{
	    ros::Duration(0.1).sleep();
	    steerMotor_.enable();
	}
	
	ROS_INFO("[%s] steering motor enabled.",__NAME__);
	
	std::string cmd_topic = nh_private.param<std::string>("cmd_topic","/cmd");
	sub_cmd_ = nh.subscribe(cmd_topic, 1, &BaseControl::cmd_callback, this);
	return true;
	
}

void BaseControl::cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& msg)
{
	if(!msg->driverless_mode) //exit auto drive
	{
		steerMotor_.disable();
		
	}
	else //enter auto drive, enable the steering
		steerMotor_.enable();
	
	
	if(msg->set_brake > 0)
	{
		//stop;
		ROS_INFO("stop!!!");
	}
	steerMotor_.run(msg->set_roadWheelAngle);
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"base_control_node");
	BaseControl base_control;
	if(base_control.init())
		ros::spin();
	return 0;
}
