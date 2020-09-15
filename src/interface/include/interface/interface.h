#ifndef INTERFCE_H_
#define INTERFCE_H_

#include <can2serial/can2serial.h>
#include <std_msgs/Float32.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <driverless_msgs/PathTrackingInfo.h>
#include <driverless_msgs/BaseControlState.h>
#include <interface/RecordPath.h>
#include <interface/Driverless.h>
#include <interface/DriverlessStatus.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include "structs.h"


class Interface
{
  public:
	Interface();
	~Interface();
	bool init();
	void run();
	
  private:
	void readCanMsg();
	void odom_callback(const nav_msgs::Odometry::ConstPtr& gps);
	void msgReport_callback(const ros::TimerEvent& event);
	void heartbeat_callback(const ros::TimerEvent& event);

	void path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& );
	void baseControlState_callback(const driverless_msgs::BaseControlState::ConstPtr& msg);
	void driveSystemState_callback(const std_msgs::UInt8::ConstPtr& msg);
	bool driverlessStatusService(interface::DriverlessStatus::Request &req, interface::DriverlessStatus::Response &res);
	
  private:
	Can2serial * can2serial_;
	std::string can2serial_port_;
	int can_baudrate_;

	canMsgs_t info_;
	
	bool gps_odom_flag_, tracking_info_flag_;
	
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_pathtracking_info_;
	ros::Subscriber sub_steerMoter_state_;
	ros::Subscriber sub_system_state_;
	ros::Subscriber sub_brake_state_;


	ros::Timer msg_report_timer_;
	ros::Timer heartbeat_timer_;
	
	boost::shared_ptr<boost::thread> read_canMsg_thread_;
	
	ros::ServiceClient client_recordPath_;
	interface::RecordPath srv_record_path_;
	
	ros::ServiceClient client_driverless_;
	interface::Driverless srv_driverless_;
	
	ros::ServiceServer server_driverless_status_;
	ros::ServiceClient client_clearMotorError_;
	//清除转向电机错误代码
	

	
};


#endif
