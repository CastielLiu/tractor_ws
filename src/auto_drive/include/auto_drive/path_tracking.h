#ifndef PATH_TRACKING_H_
#define PATH_TRACKING_H_

#include<boost/filesystem.hpp>
#include<driverless_msgs/PathTrackingInfo.h>
#include<interface/Driverless.h>
#include<interface/Other.h>
#include<driverless_msgs/ControlCmd.h>
#include"auto_drive/function.h"
#include<std_msgs/Float32.h>
#include<std_msgs/Float32.h>

#include<boost/thread.hpp>
#include<std_msgs/UInt32.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<boost/bind.hpp>
#include<ros/ros.h>
#include<climits>
#include<thread>
#include<mutex>
#include<vector>

class PathTracking
{
public:
	PathTracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	~PathTracking();
	bool init(const gpsMsg_t& vehicle_point );
	bool isRunning() {return is_running_;}
	void setPath(const path_t& path);
	bool extendGlobalPath(float extendDis);
	
	bool update(float speed, float road_wheelangle,  //vehicle state
			   const gpsMsg_t& vehicle_point,      //vehicle positoin
			   const float& path_offset); 
	void getTrackingCmd(float& speed, float& roadWheelAngle);

private:
	void publishInfo();
private:
	ros::NodeHandle nh_, nh_private_;
	ros::Publisher pub_info_;
	
	std::mutex path_mutex_;
	path_t path_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;

	gpsMsg_t target_point_;
	
	float min_foresight_distance_;
	float disThreshold_;
	float vehicle_speed_;
	float road_wheelangle_;

	float t_speed_, t_roadwheel_angle_;
	
	driverless_msgs::PathTrackingInfo info_;
	
	float lateral_err_;
	
	float foreSightDis_speedCoefficient_;
	float foreSightDis_latErrCoefficient_;
	float wheel_base_;
	bool is_running_;
	size_t destination_index_;
	
};


#endif