#ifndef AUTO_DRIVE_H_
#define AUTO_DRIVE_H_

#include"auto_drive/path_tracking.h"
#include"auto_drive/avoiding.h"
#include<nav_msgs/Odometry.h>
#include <interface/DriverlessStatus.h>
#include <interface/Driverless.h>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt8.h>
#include <driverless_msgs/BaseControlState.h>
#include "state_machine.h"
#include <shared_mutex>
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
    void base_ctrl_state_callback(const driverless_msgs::BaseControlState::ConstPtr& msg);
    void update_timer_callback(const ros::TimerEvent&);
    bool driverlessService(interface::Driverless::Request  &req,
						   interface::Driverless::Response &res);
    bool recordPathService(interface::RecordPath::Request  &req,
						   interface::RecordPath::Response &res);

	bool callDriverlessStatusService(uint8_t status);

private:
    const gpsMsg_t currentPose();
  private: 
    float max_speed_;
	float vehicle_speed_;
	float roadwheel_angle_;
	float max_roadwheelAngle_;
    gpsMsg_t pose_;
    std::shared_mutex pose_wr_mutex_;
    std::string path_file_dir_;
    path_t path_;

	ros::Timer timer_;
	ros::Publisher pub_cmd_;
    ros::Publisher pub_state_;
    ros::Subscriber sub_utm_;
    ros::Subscriber sub_state_;
    ros::Timer update_timer_;

    ros::ServiceServer srv_driverless_;
    ros::ServiceServer srv_recorder_;
    ros::NodeHandle nh_, nh_private_;

    std::shared_ptr<std::thread> auto_drive_thread_ptr_;
    std::mutex auto_drive_thread_mutex_;

    driverless_msgs::ControlCmd cmd_;
    
    StateMachine state_;

    float avoid_offset_;

    PathTracking tracker_;
    Avoiding avoider_;
    Recorder recorder_;

};





#endif
