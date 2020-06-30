#include "auto_drive/path_tracking.h"

/* path tracking module for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

PathTracking::PathTracking(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
	nh_(nh),
    nh_private_(nh_private)
{
	is_running_ = false;
}

PathTracking::~PathTracking()
{
}

bool PathTracking::init(const gpsMsg_t& vehicle_point)
{
	pub_info_ = nh_.advertise<driverless_msgs::PathTrackingInfo>("/path_tracking_info",1);

	nh_private_.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);
	nh_private_.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,0.3);
	
	nh_private_.param<float>("min_foresight_distance",min_foresight_distance_,4.0);
	nh_private_.param<float>("wheel_base", wheel_base_, 1.5);

	if(path_.points.empty())
	{
		ROS_ERROR("[PathTracking]: please call setPath before init pathTracking!");
		return false;
	}

	nearest_point_index_ = findNearestPoint(path_, vehicle_point);
	if(nearest_point_index_ <0)
		return false;

	target_point_index_ = nearest_point_index_;
	return true;
}

// 0 update no
// 1 update ok
// 2 path_tracking over
int PathTracking::update(float speed, float road_wheelangle,  //vehicle state
						 const gpsMsg_t& vehicle_point,      //vehicle positoin
						 const float& path_offset)
{
	vehicle_speed_ = speed;
	road_wheelangle_ = road_wheelangle;

	static int cnt = 0;
	target_point_ = path_.points[target_point_index_];
	
	lateral_err_ = calculateDis2path(vehicle_point.x, vehicle_point.y, path_, nearest_point_index_, //input
					            nearest_point_index_); //output
	// lateral_err_ = lateral_err_ - path_offset; 
	//添加退出条件！！

	disThreshold_ = foreSightDis_latErrCoefficient_ * fabs(lateral_err_) + min_foresight_distance_; 

	if( path_offset != 0.0)
		target_point_ = pointOffset(target_point_, path_offset);

	std::pair<float, float> dis_yaw = get_dis_yaw(target_point_, vehicle_point);

	if( dis_yaw.first < disThreshold_)
	{
		++target_point_index_;
		
	}
	float yaw_err = dis_yaw.second - vehicle_point.yaw;
	
	if(yaw_err==0.0)
		return 0;
	
	float turning_radius = (-0.5 * dis_yaw.first)/sin(yaw_err);

	t_roadwheel_angle_ = generateRoadwheelAngleByRadius(turning_radius, wheel_base_);
	
	t_speed_ = 20.0; //temp
	
	if(++cnt%20==0)
	{
		ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err*180.0/M_PI,lateral_err_);
		ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,road_wheelangle);
		ROS_INFO("path_offset:%f\n",path_offset);
	}
	this->publishInfo();
}

void PathTracking::setPath(const path_t& path)
{
	path_mutex_.lock();
    path_ = path;
	path_mutex_.unlock();
}

void PathTracking::getTrackingCmd(float& speed, float& roadWheelAngle)
{
    speed = t_speed_;
	roadWheelAngle = t_roadwheel_angle_;
}


void PathTracking::publishInfo()
{
	info_.nearest_point_index = nearest_point_index_;
	info_.target_point_index = target_point_index_;
	info_.speed = vehicle_speed_;
	info_.lateral_err = lateral_err_;

	pub_info_.publish(info_);
}
