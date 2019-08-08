#ifndef INTERFCE_H_
#define INTERFCE_H_

#include<can2serial/can2serial.h>
#include<std_msgs/Float32.h>
#include<string>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include<gps_msgs/Inspvax.h>
#include<driverless_msgs/PathTrackingInfo.h>
#include<interface/RecordPath.h>


class Interface
{
  public:
  	Interface();
  	~Interface();
  	bool init();
  	void run();
  	
  private:
  	void readCanMsg();
  	void gps_callback(const gps_msgs::Inspvax::ConstPtr& gps);
  	void roadWheelAngle_callback(const std_msgs::Float32 angle);
  	void timer_callback(const ros::TimerEvent& event);
  	void path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& );
  	enum 
	{
		GPS_CAN_ID = 0x500,
		STATUS_CAN_ID = 0x501,
		RECORD_PATH_CAN_ID = 0x502,
	};

	struct Informathion
	{
		CanMsg_t gps = {GPS_CAN_ID,8,Can2serial::STD_DATA_FRAME};
		CanMsg_t status = {STATUS_CAN_ID,8,Can2serial::STD_DATA_FRAME};
		CanMsg_t record_path={RECORD_PATH_CAN_ID,8,Can2serial::STD_DATA_FRAME};
		
		//CanMsg_t
	};

  private:
	Can2serial * can2serial_;
	std::string can2serial_port_;
	int can_baudrate_;
	Informathion info_;
	
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_roadWheelAngle_;
	ros::Subscriber sub_pathtracking_info_;
	ros::Timer timer_;
	
	boost::shared_ptr<boost::thread> read_canMsg_thread_;
	ros::ServiceClient client_recordPath_;
	interface::RecordPath srv_record_path_;
};


#endif
