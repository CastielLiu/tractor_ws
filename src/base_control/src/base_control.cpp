#include"base_control/steering_motor.h"
#include<driverless_msgs/ControlCmd.h>
#include<driverless_msgs/BaseControlState.h>
#include<driverless_msgs/SteerMotorDebug.h>
#include<ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include<serial/serial.h>
#include<iostream>
#include<std_srvs/Empty.h>
#include<string>
#include <atomic>

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
    bool rebootSteerMotorService(std_srvs::Empty::Request  &req,
						   		std_srvs::Empty::Response &res);
    bool clearSteerErrorService(std_srvs::Empty::Request  &req,
						   		std_srvs::Empty::Response &res);
    bool resetBrakeService(		std_srvs::Empty::Request  &req,
						   		std_srvs::Empty::Response &res);
	void cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd);
	void steerDebug_callback(const driverless_msgs::SteerMotorDebug::ConstPtr& debug);
	void brakeSystem_callback(const std_msgs::UInt8::ConstPtr& state);
	void publishState_callback(const ros::TimerEvent&);
	SteerMotor steerMotor_;
	std::string steerMotor_port_name_;
	
	ros::Subscriber sub_cmd_;
	ros::Subscriber sub_brakeSystem_;
	ros::Subscriber sub_steer_debug_;

	ros::Publisher pub_brakeCmd_;
	ros::Publisher pub_state_;

	ros::ServiceServer srv_reboot_steer_;
    ros::ServiceServer srv_clear_steer_;
    ros::ServiceServer srv_resetBrake_;
	
	driverless_msgs::BaseControlState state_;
	driverless_msgs::ControlCmd cmd_;
	std::atomic<bool> driverless_mode_;
	ros::Timer publishState_timer_;
	double lastBrakeValueTime_;
	bool ignore_braker_error_;
	bool ignore_steer_error_;
	uint8_t brake_state_value_;
};

BaseControl::BaseControl():
	brake_state_value_(5),
	lastBrakeValueTime_(0),
	driverless_mode_(false)
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
	
	std::string cmd_topic = nh_private.param<std::string>("cmd_topic","/cmd");
	sub_cmd_ = nh.subscribe(cmd_topic, 1, &BaseControl::cmd_callback, this);
	sub_steer_debug_ = nh.subscribe("/steer_motor_debug", 1, &BaseControl::steerDebug_callback, this);

	sub_brakeSystem_ = nh.subscribe("/brake_state", 1, &BaseControl::brakeSystem_callback, this);
	pub_brakeCmd_ = nh.advertise<std_msgs::UInt8>("/brake_cmd", 1);

	publishState_timer_ = 
	    nh.createTimer(ros::Duration(0.1), &BaseControl::publishState_callback, this);
	pub_state_ = nh.advertise<driverless_msgs::BaseControlState>("/base_control_state",1);
	
	srv_reboot_steer_ = nh.advertiseService("reboot_motor",&BaseControl::rebootSteerMotorService, this);
	srv_clear_steer_  = nh.advertiseService("clear_motor_error_flag",&BaseControl::clearSteerErrorService, this);
	srv_resetBrake_   = nh.advertiseService("reset_braker",&BaseControl::resetBrakeService, this);

	float road_wheel_angle_offset = nh_private.param<float>("road_wheel_angle_offset",0.0);
 	float road_wheel_angle_resolution = nh_private.param<float>("road_wheel_angle_resolution",180.0/4096);
	steerMotor_.setRoadWheelAngleOffset(road_wheel_angle_offset);
	steerMotor_.setRoadWheelAngleResolution(road_wheel_angle_resolution);
	steerMotor_.setMaxRoadWheelAngle(nh_private.param<float>("max_road_wheel_angle",35));
	float max_steer_rotate_speed = nh_private.param<float>("max_steer_rotate_speed",100);
	float min_steer_rotate_speed = nh_private.param<float>("min_steer_rotate_speed",10);
	steerMotor_.setRotateSpeedRange(min_steer_rotate_speed, max_steer_rotate_speed);

	ignore_steer_error_ = nh_private.param<bool>("ignore_steer_error", false);
	ignore_braker_error_ = nh_private.param<bool>("ignore_braker_error", false);

	if(!steerMotor_.init(steerMotor_port_name_,115200))
	{
	    ROS_ERROR("[%s] init steering motor failed.", __NAME__);
		if(ignore_steer_error_) //debug mode
		{
			ROS_ERROR("[%s] sytem ignore init steering motor failed.", __NAME__);
			return true;
		}
		return false;
	}
    return true;
}

bool BaseControl::rebootSteerMotorService(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res)
{
	ROS_INFO("[%s] Request reboot the steer motor.", __NAME__);
    steerMotor_.reboot();
    return true;
}
bool BaseControl::clearSteerErrorService(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res)
{
	ROS_INFO("[%s] Request clear the steer motor error flag.", __NAME__);
    steerMotor_.clearErrorFlag();
    return true;
}
bool BaseControl::resetBrakeService(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res)
{
	ROS_INFO("[%s] Request reset the braker.", __NAME__);
    std_msgs::UInt8 brakeVal;
	brakeVal.data = 0;
	for(int i=0; i<20; ++i){
		pub_brakeCmd_.publish(brakeVal);
		ros::Duration(0.02).sleep();
	}
	return true;
}

void BaseControl::brakeSystem_callback(const std_msgs::UInt8::ConstPtr& state)
{
	brake_state_value_ = state->data;
	lastBrakeValueTime_ = ros::Time::now().toSec();
}

void BaseControl::publishState_callback(const ros::TimerEvent&)
{
	//转向电机状态反馈
  	state_.roadWheelAngle = steerMotor_.getRoadWheelAngle();
    state_.motorSpeed = steerMotor_.getMotorSpeed();
    state_.steerMotorEnabled = steerMotor_.is_enabled();

	if(ignore_steer_error_)
    	state_.steerMotorError = 0; //steerMotor_.getErrorMsg();
	else
		state_.steerMotorError = steerMotor_.getErrorMsg();

	//制动系统状态反馈
	if(ros::Time::now().toSec() - lastBrakeValueTime_ > 1.0 && //制动系统状态反馈超时
		!ignore_braker_error_)
		state_.brakeError = state_.BRAKE_ERROR_OFFLINE;
	else
		state_.brakeError = state_.BRAKE_ERROR_NONE;
	state_.brake_val = brake_state_value_;
    pub_state_.publish(state_);
}

void BaseControl::cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& msg)
{
	static int try_disable_motor_cnt = 0;
	if(!msg->driverless_mode && driverless_mode_) //上次处于自动驾驶模式，当前请求关闭
		try_disable_motor_cnt = 50;
	
	if(!msg->driverless_mode)
	{
		if(try_disable_motor_cnt > 0)
		{
			--try_disable_motor_cnt;
			steerMotor_.disable();
		}
	}
	else //enter auto drive, enable the steering
	{
		steerMotor_.enable();
		steerMotor_.setRoadWheelAngle(msg->set_roadWheelAngle);
	}

	driverless_mode_ = msg->driverless_mode;
	
	if(brake_state_value_ != msg->set_brake)
	{
		//转发制动指令
		std_msgs::UInt8 brakeVal;
		brakeVal.data = msg->set_brake;
		pub_brakeCmd_.publish(brakeVal);
	}
}

void BaseControl::steerDebug_callback(const driverless_msgs::SteerMotorDebug::ConstPtr& debug)
{
	if(driverless_mode_) //目前为自动驾驶模式，禁止使用转向电机调试电机
	{
		ROS_ERROR("[%s] Can not debug steer motor in driverless mode!", __NAME__);
		return;
	}

	if(debug->steer_enable)
	{
		steerMotor_.enable();
		steerMotor_.setSpeedAndAngle(debug->set_speed, debug->set_roadWheelAngle);
	}
	else
	{
		steerMotor_.disable();
	}

		
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"base_control_node");
	BaseControl base_control;
	if(base_control.init())
		ros::spin();
	std::cout << "[" << __NAME__ << "] exit!" << std::endl; 
	return 0;
}
