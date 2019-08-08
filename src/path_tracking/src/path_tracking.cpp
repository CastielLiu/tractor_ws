#include "path_tracking/path_tracking.h"


PathTracking::PathTracking():
	target_point_index_(0),
	nearest_point_index_(0),
	avoiding_offset_(0.0),
	max_roadwheelAngle_(25.0),
	is_avoiding_(false)
{
	cmd_.set_speed =0.0;
	cmd_.set_roadWheelAngle =0.0;
}

PathTracking::~PathTracking()
{
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_utm_ = nh.subscribe("/utm_gps",1,&PathTracking::utm_callback,this);

	sub_avoiding_from_lidar_ = nh.subscribe("/start_avoiding",1,&PathTracking::avoiding_flag_callback,this);
	
	pub_cmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",1);
	
	timer_ = nh.createTimer(ros::Duration(0.03),&PathTracking::timer_callback,this);
	
	pub_info_ = nh.advertise<driverless_msgs::PathTrachingInfo>("/path_tracking_info",1);
	
	nh_private.param<std::string>("path_points_file",path_points_file_,"");

	nh_private.param<float>("speed",path_tracking_speed_,3.0);

	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,0.3);
	
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,5.0);
	nh_private.param<float>("wheel_base", wheel_base_, 1.5);
	
	if(path_points_file_.empty())
	{
		ROS_ERROR("no input path points file !!");
		return false;
	}
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::spinThread, this)));
	
	if(!loadPathPoints(path_points_file_, path_points_))
		return false;
	
	ROS_INFO("pathPoints size:%lu",path_points_.size());
	
	/*for(size_t i=0; i< path_points_.size();i++ )
	{
	  std::cout <<path_points_[i].x <<"\t " <<path_points_[i].y <<std::endl;
	}*/
	
	while(ros::ok() && !is_gps_data_valid(current_point_))
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		sleep(1);
	}
	
	target_point_index_ = findNearestPoint(path_points_,current_point_);
	
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_ERROR("target index:%lu ?? file read over, No target point was found !!!",target_point_index_);
		return false;
	}
	
	target_point_ = path_points_[target_point_index_];
	return true;
}



void PathTracking::run()
{
	size_t i =0;
	
	ros::Rate loop_rate(30);
	
	while(ros::ok() && target_point_index_ < path_points_.size()-2)
	{
		if( avoiding_offset_ != 0.0)
			pointOffset(target_point_,avoiding_offset_);
		
		try
		{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) - avoiding_offset_;
		}catch(const char* str)
		{
			ROS_INFO("%s",str);
			break;
		}
		
		disThreshold_ = foreSightDis_latErrCoefficient_ * fabs(lateral_err_) + min_foresight_distance_;
									 
		std::pair<float, float> dis_yaw = get_dis_yaw(target_point_, current_point_);

		if( dis_yaw.first < disThreshold_)
		{
			target_point_ = path_points_[++target_point_index_];
			continue;
		}
		float yaw_err = dis_yaw.second - current_point_.yaw;
		
		if(yaw_err==0.0) continue;
		
		float turning_radius = (-0.5 * dis_yaw.first)/sin(yaw_err);

		float t_roadWheelAngle = generateRoadwheelAngleByRadius(turning_radius, wheel_base_);
		
		cmd_.set_speed = path_tracking_speed_;
		
		cmd_.set_roadWheelAngle = t_roadWheelAngle;
		
		if(i%20==0)
		{
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
			ROS_INFO("avoiding_offset_:%f\n",avoiding_offset_);
		}
		i++;
		this->publishInfo();
		loop_rate.sleep();
	}
	
	ROS_INFO("driverless completed...");
	
	cmd_.set_roadWheelAngle = 0.0;
	cmd_.set_speed = 0.0;
	
	sleep(5);
}


void PathTracking::publishInfo()
{
	info_.nearest_point_index = nearest_point_index_;
	info_.target_point_index = target_point_index_;
	info_.speed = current_speed_;
	info_.lateral_err = lateral_err_;

	pub_info_.publish(info_);
}


void PathTracking::timer_callback(const ros::TimerEvent&)
{
	pub_cmd_.publish(cmd_);
}


void PathTracking::utm_callback(const gps_msgs::Utm::ConstPtr& msg)
{
	current_point_.x = msg->x;
	current_point_.y = msg->y;
	current_point_.yaw = msg->yaw;
	
	float speed = msg->north_velocity * msg->north_velocity + msg->east_velocity * msg->east_velocity;
	current_speed_ = sqrt(speed)*3.6; //km/h
	
	/*
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
	
	double roll,pitch;
	
	tf::Matrix3x3(quat).getRPY(roll, pitch, current_point_.yaw);
*/
}

void PathTracking::avoiding_flag_callback(const std_msgs::Float32::ConstPtr& msg)
{
	//avoid to left(-) or right(+) the value presents the offset
	avoiding_offset_ = msg->data;
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,"path_tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	PathTracking path_tracking;
	if(!path_tracking.init(nh,nh_private))
		return 1;
	path_tracking.run();
	
	ROS_INFO("path tracking completed.");
	ros::shutdown();

	return 0;
}
