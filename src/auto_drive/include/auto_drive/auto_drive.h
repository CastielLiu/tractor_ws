#ifndef AUTO_DRIVE_H_
#define AUTO_DRIVE_H_

#include"auto_drive/path_tracking.h"
#include"auto_drive/avoiding.h"
#include<nav_msgs/Odometry.h>
#include"ros/ros.h"

namespace fs = boost::filesystem;

class AutoDrive 
{
  public:
    AutoDrive();
    ~AutoDrive();
    bool init(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void run();
    void autoDriveThread(const fs::path& file, float speed);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void timer_callback(const ros::TimerEvent&);
    bool driverlessService(interface::Driverless::Request  &req,
									 interface::Driverless::Response &res);
	bool callOtherService(const std::string& data);

  private: 
    float max_speed_;
	float vehicle_speed_;
	float roadwheel_angle_;
	float max_roadwheelAngle_;
    gpsMsg_t current_point_;
    std::string path_file_dir_;
    std::vector<gpsMsg_t> path_points_;

	ros::Timer timer_;
	ros::Publisher pub_cmd_;
    ros::Subscriber sub_utm_;
    ros::ServiceServer srv_driverless_;
	ros::ServiceClient other_client_nh_;
    ros::NodeHandle nh_, nh_private_;

    boost::shared_ptr<boost::thread> auto_drive_thread_ptr_;
    driverless_msgs::ControlCmd cmd_;
    
	int status_;
    float avoid_offset_;

    PathTracking tracker_;
    Avoiding avoider_;

};





#endif