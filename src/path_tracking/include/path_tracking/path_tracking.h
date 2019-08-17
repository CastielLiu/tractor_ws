#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_

#include<boost/filesystem.hpp>
#include<driverless_msgs/PathTrachingInfo.h>
#include<interface/Driverless.h>
#include<driverless_msgs/ControlCmd.h>
#include"path_tracking/function.h"
#include<std_msgs/Float32.h>
#include<std_msgs/Float32.h>
#include<boost/thread.hpp>
#include<std_msgs/UInt32.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<gps_msgs/Inspvax.h>
#include<gps_msgs/Utm.h>
#include<boost/bind.hpp>
#include<ros/ros.h>
#include<climits>
#include <thread>
#include<vector>
namespace fs = boost::filesystem;

class PathTracking
{
public:
	PathTracking();
	~PathTracking();
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private);
	void run();
	
	void timer_callback(const ros::TimerEvent&);
	
	void utm_callback(const gps_msgs::Utm::ConstPtr& msg);
	
	void avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg);

	void spinThread(){ros::spin();}

private:
	void publishInfo();
	bool driverlessService(interface::Driverless::Request  &req,
									 interface::Driverless::Response &res);
	void pathTrackingThread(const fs::path& file, float speed);
private:
	
	ros::Subscriber sub_utm_;
	
	ros::Subscriber sub_avoiding_from_lidar_;
	
	ros::Timer timer_;
	
	ros::Publisher pub_cmd_;
	
	ros::Publisher pub_info_;
	
	ros::ServiceServer srv_driverless_;
	
	boost::shared_ptr<boost::thread> rosSpin_thread_ptr_;
	
	std::string path_file_dir_;
	
	std::vector<gpsMsg_t> path_points_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	gpsMsg_t current_point_;
	gpsMsg_t target_point_;
	
	float min_foresight_distance_;
	float disThreshold_;
	
	float avoiding_offset_;
	
	driverless_msgs::ControlCmd cmd_;
	driverless_msgs::PathTrachingInfo info_;
	
	float max_speed_;
	float current_speed_;
	
	float current_roadwheelAngle_;
	
	float max_roadwheelAngle_;
	
	bool is_avoiding_;
	
	float lateral_err_;
	
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	float wheel_base_;
	
	bool is_gpsOk_;
	
	enum status_t
	{
		Idle = 0,
		Suspend = 1,
		VertexTracking = 2,
		CurveTracking = 3,
		
	}status_;
	
	boost::thread * thread_ptr_;
	
};


#endif

