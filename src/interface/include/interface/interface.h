#ifndef INTERFCE_H_
#define INTERFCE_H_

#include <can2serial/can2serial.h>
#include <std_msgs/Float32.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <driverless_msgs/PathTrackingInfo.h>
#include <interface/RecordPath.h>
#include <interface/Driverless.h>
#include <interface/Other.h>

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
	void timer_callback(const ros::TimerEvent& event);
	void path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& );
	bool otherService(interface::Other::Request  &req, interface::Other::Response &res);
	enum 
	{
		GPS_CAN_ID = 0x301,
		STATUS_CAN_ID = 0x302,
		RECORD_PATH_CAN_ID = 0x200,
		DRIVERLESS_CAN_ID = 0x201,
		RESPONSE_CAN_ID = 0x205,
	};
	struct Informathion
	{
		Informathion()
		{
			gps.ID = GPS_CAN_ID;
			status.ID = STATUS_CAN_ID;
		}
		CanMsg_t gps, status;
	};
	
  private:
	Can2serial * can2serial_;
	std::string can2serial_port_;
	int can_baudrate_;
	Informathion info_;
	
	bool odom_flag_, tracking_info_flag_;
	
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_pathtracking_info_;
	ros::Timer timer_;
	
	boost::shared_ptr<boost::thread> read_canMsg_thread_;
	ros::ServiceClient client_recordPath_;
	interface::RecordPath srv_record_path_;
	
	ros::ServiceClient client_driverless_;
	interface::Driverless srv_driverless_;
	
	ros::ServiceServer other_srv_nh_;
};


#endif
