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
		return false;
	steerMotor_.enable();
	steerMotor_.startReadSerial();
	
	sub_cmd_ = nh.subscribe("/cmd",1,&BaseControl::cmd_callback,this);
	
}

void BaseControl::cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& msg)
{
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
