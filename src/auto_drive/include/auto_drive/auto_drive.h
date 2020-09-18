#ifndef AUTO_DRIVE_H_
#define AUTO_DRIVE_H_

#include"auto_drive/path_tracking.h"
#include"auto_drive/avoiding.h"
#include<nav_msgs/Odometry.h>
#include <interface/DriverlessStatus.h>
#include <interface/Driverless.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>

#include <driverless_msgs/State.h>
//#include <>
#include"ros/ros.h"

namespace fs = boost::filesystem;

class AutoDrive 
{
  public:
    AutoDrive();
    ~AutoDrive();
    bool init();
    void run();
    void autoDriveThread(float speed);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void state_callback(const driverless_msgs::State::ConstPtr& msg);
    void update_timer_callback(const ros::TimerEvent&);
    bool driverlessService(interface::Driverless::Request  &req,
									 interface::Driverless::Response &res);
	bool callDriverlessStatusService(uint8_t status);

  private: 
    float max_speed_;
	float vehicle_speed_;
	float roadwheel_angle_;
	float max_roadwheelAngle_;
    gpsMsg_t vehicle_point_;
    std::string path_file_dir_;
    path_t path_;

	ros::Timer timer_;
	ros::Publisher pub_cmd_;
    ros::Publisher pub_state_;
    ros::Subscriber sub_utm_;
    ros::Subscriber sub_state_;
    ros::Timer update_timer_;

    ros::ServiceServer srv_driverless_;
	ros::ServiceClient driverless_status_client_nh_;
    ros::NodeHandle nh_, nh_private_;

    boost::shared_ptr<boost::thread> auto_drive_thread_ptr_;
    driverless_msgs::ControlCmd cmd_;
    
	int status_;
    float avoid_offset_;

    PathTracking tracker_;
    Avoiding avoider_;

};





#endif
