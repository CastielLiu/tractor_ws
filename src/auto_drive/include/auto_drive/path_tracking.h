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
	PathTracking(ros::NodeHandle nh,ros::NodeHandle nh_private);
	~PathTracking();
	bool init();
	void setPathPoints(const std::vector<gpsMsg_t>& points);
	int update(float speed, float road_wheelangle,  //vehicle state
			   const gpsMsg_t& current_point,      //vehicle positoin
			   const float& path_offset); 
	void getTrackingCmd(float& speed, float& roadWheelAngle);

private:
	void publishInfo();
private:
	ros::Publisher pub_info_;
	
	std::mutex path_points_mutex_;
	std::vector<gpsMsg_t> path_points_;
	
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
	
};


#endif