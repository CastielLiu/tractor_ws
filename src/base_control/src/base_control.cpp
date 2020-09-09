#include"base_control/steering_motor.h"
#include<driverless_msgs/ControlCmd.h>
#include<driverless_msgs/State.h>
#include<ros/ros.h>
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
    void requestMotorStatus();
    bool clearMotorErrors(std_srvs::Empty::Request& , std_srvs::Empty::Response&);
	void cmd_callback(const driverless_msgs::ControlCmd::ConstPtr& cmd);
	void requestMotorStatus_callback(const ros::TimerEvent&);
	SteerMotor steerMotor_;
	std::string steerMotor_port_name_;
	
	ros::Subscriber sub_cmd_;
	ros::Publisher pub_state_;
	ros::ServiceServer clear_motor_error_service_;
	
	driverless_msgs::State state_;
	driverless_msgs::ControlCmd cmd_;
	ros::Timer getSystemData_timer_;
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
	steerMotor_.startReadSerial();
	
	getSystemData_timer_ = 
	    nh.createTimer(ros::Duration(0.1), &BaseControl::requestMotorStatus_callback, this);
	
	ROS_INFO("[%s] waiting for steering motor enable...",__NAME__);
	
	while(ros::ok() && !steerMotor_.is_enabled())
	{
	    steerMotor_.enable();             //电机使能
	    ros::Duration(0.2).sleep();
	    steerMotor_.requestEnableStatus();//请求获取使能状态
	    ros::Duration(0.5).sleep();
	    ROS_INFO("[%s] waiting for steering motor enable...",__NAME__);
	}
	
	ROS_INFO("[%s] steering motor enabled.",__NAME__);
	
	std::string cmd_topic = nh_private.param<std::string>("cmd_topic","/cmd");
	sub_cmd_ = nh.subscribe(cmd_topic, 1, &BaseControl::cmd_callback, this);
	
	pub_state_ = nh.advertise<driverless_msgs::State>("/state",1);
	clear_motor_error_service_ = nh.advertiseService("/clear_motor_error_flag", &BaseControl::clearMotorErrors, this);
	
	ros::Rate loop_rate(30);
	while(ros::ok())
	{
	    ros::spinOnce();
	    loop_rate.sleep();
	}
	
	steerMotor_.stopReadSerial();
    return true;
}

bool BaseControl::clearMotorErrors(std_srvs::Empty::Request& , std_srvs::Empty::Response&)
{
    steerMotor_.clearErrorFlag();
    return true;
}

void BaseControl::requestMotorStatus()
{
    static int i = 0;
    static float cmd_inteval = 0.01;
    
    if(i%2 == 0)
    {
        steerMotor_.requestMotorSpeed();
        ros::Duration(cmd_inteval).sleep(); 
    }
    if(i%5 == 0)
    {
        steerMotor_.requestEnableStatus();
        ros::Duration(cmd_inteval).sleep();
    }
    if(i%50 == 0)
    {
        steerMotor_.requestErrorMsg();
        ros::Duration(cmd_inteval).sleep();
    }
    ++i;
    
    steerMotor_.requestAdcValue();
    
    state_.roadWheelAngle = steerMotor_.getRoadWheelAngle();
    state_.motorSpeed = steerMotor_.getMotorSpeed();
    state_.steerMotorEnabled = steerMotor_.is_enabled();
    state_.steerMotorError = steerMotor_.getErrorMsg();
    pub_state_.publish(state_);
    
    ros::Duration(cmd_inteval).sleep();
}

void BaseControl::requestMotorStatus_callback(const ros::TimerEvent&)
{
    this->requestMotorStatus();
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
	
	steerMotor_.setRoadWheelAngle(msg->set_roadWheelAngle);
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"base_control_node");
	BaseControl base_control;
	base_control.init();
	return 0;
}
