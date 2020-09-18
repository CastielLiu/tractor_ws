#include"base_control/steering_motor.h"
#include<driverless_msgs/ControlCmd.h>
#include<driverless_msgs/BaseControlState.h>
#include<ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include<serial/serial.h>
#include<iostream>
#include<std_srvs/Empty.h>
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
    bool rebootMotor(std_srvs::Empty::Request& , std_srvs::Empty::Response&);
    bool clearMotorErrors(std_srvs::Empty::Request& , std_srvs::Empty::Response&);
	void cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd);
	void brakeSystem_callback(const std_msgs::UInt8::ConstPtr& state);
	void publishState_callback(const ros::TimerEvent&);
	SteerMotor steerMotor_;
	std::string steerMotor_port_name_;
	
	ros::Subscriber sub_cmd_;
	ros::Subscriber sub_brakeSystem_;
	ros::Publisher pub_brakeCmd_;
	ros::Publisher pub_state_;
	ros::ServiceServer clear_motor_error_service_;
	
	driverless_msgs::BaseControlState state_;
	driverless_msgs::ControlCmd cmd_;
	ros::Timer publishState_timer_;
	double lastBrakeValueTime_;
	uint8_t brake_value_;
};

BaseControl::BaseControl():
	lastBrakeValueTime_(0)
{
	
}

BaseControl::~BaseControl()
{
	steerMotor_.stop();
}


bool BaseControl::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("steering_port",steerMotor_port_name_,"/dev/ttyUSB0");
	
	float road_wheel_angle_offset, road_wheel_angle_resolution;
	nh_private.param<float>("road_wheel_angle_offset",road_wheel_angle_offset,0.0);
	nh_private.param<float>("road_wheel_angle_resolution",road_wheel_angle_resolution,180.0/4096);
	steerMotor_.setRoadWheelAngleOffset(road_wheel_angle_offset);
	steerMotor_.setRoadWheelAngleResolution(road_wheel_angle_resolution);
	
	if(!steerMotor_.init(steerMotor_port_name_,115200))
	{
	    ROS_ERROR("[%s] init steering motor failed.", __NAME__);
		return false;
	}
	
	publishState_timer_ = 
	    nh.createTimer(ros::Duration(0.1), &BaseControl::publishState_callback, this);
	
	std::string cmd_topic = nh_private.param<std::string>("cmd_topic","/cmd");
	sub_cmd_ = nh.subscribe(cmd_topic, 1, &BaseControl::cmd_callback, this);
	sub_brakeSystem_ = nh.subscribe("/brake_state", 1, &BaseControl::brakeSystem_callback, this);
	pub_brakeCmd_ = nh.advertise<std_msgs::UInt8>("/brake_cmd", 1);

	
	pub_state_ = nh.advertise<driverless_msgs::BaseControlState>("/base_control_state",1);
	clear_motor_error_service_ = nh.advertiseService("/clear_motor_error_flag", &BaseControl::clearMotorErrors, this);
	
    return true;
}

bool BaseControl::clearMotorErrors(std_srvs::Empty::Request& , std_srvs::Empty::Response&)
{
    steerMotor_.clearErrorFlag();
    return true;
}

bool BaseControl::rebootMotor(std_srvs::Empty::Request& , std_srvs::Empty::Response&)
{
    steerMotor_.reboot();
    return true;
}

void BaseControl::brakeSystem_callback(const std_msgs::UInt8::ConstPtr& state)
{
	brake_value_ = state->data;
	lastBrakeValueTime_ = ros::Time::now().toSec();
}

void BaseControl::publishState_callback(const ros::TimerEvent&)
{
	//转向电机状态反馈
  	state_.roadWheelAngle = steerMotor_.getRoadWheelAngle();
    state_.motorSpeed = steerMotor_.getMotorSpeed();
    state_.steerMotorEnabled = steerMotor_.is_enabled();
    state_.steerMotorError = steerMotor_.getErrorMsg();

	//制动系统状态反馈
	if(ros::Time::now().toSec() - lastBrakeValueTime_ > 0.5) //制动系统状态反馈超时
		state_.brakeError = state_.BRAKE_ERROR_OFFLINE;
	else
		state_.brakeError = state_.BRAKE_ERROR_NONE;
	state_.brake_val = brake_value_;

    pub_state_.publish(state_);
}

void BaseControl::cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& msg)
{
	if(!msg->driverless_mode) //exit auto drive
		steerMotor_.disable();
	else //enter auto drive, enable the steering
		steerMotor_.enable();
	steerMotor_.setRoadWheelAngle(msg->set_roadWheelAngle);
	
	//转发制动指令
	std_msgs::UInt8 brakeVal;
	brakeVal.data = msg->set_brake;
	pub_brakeCmd_.publish(brakeVal);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"base_control_node");
	BaseControl base_control;
	base_control.init();
	ros::spin();
	return 0;
}
