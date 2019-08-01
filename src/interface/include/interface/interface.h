#ifndef INTERFCE_H_
#define INTERFCE_H_

#include<can2serial/can2serial.h>
#include<std_msgs/Float32.h>
#include<string>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include<gps_msgs/Inspvax.h>


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
  	
  	enum 
	{
		GPS_CAN_ID = 0x500,
	
	};

	struct Informathion
	{
		CanMsg_t gps = {GPS_CAN_ID,8,Can2serial::STD_DATA_FRAME};
		//CanMsg_t
	};

  	
  private:
	Can2serial * can2serial_;
	std::string can2serial_port_;
	int can_baudrate_;
	Informathion info_;
	
	ros::Subscriber sub_gps_;
	ros::Subscriber sub_roadWheelAngle_;
	ros::Timer timer_;
	
	boost::shared_ptr<boost::thread> read_canMsg_thread_;
	
	
};


#endif
