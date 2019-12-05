#include "path_tracking/path_tracking.h"


PathTracking::PathTracking():
	target_point_index_(0),
	nearest_point_index_(0),
	avoiding_offset_(0.0),
	max_roadwheelAngle_(25.0),
	is_avoiding_(false),
	status_(Idle)
{
	cmd_.set_speed =0.0;
	cmd_.set_roadWheelAngle =0.0;
}

PathTracking::~PathTracking()
{
}

bool PathTracking::init(ros::NodeHandle nh,ros::NodeHandle nh_private)
{
	sub_utm_ = nh.subscribe("/ll2utm",1,&PathTracking::odom_callback,this);

	sub_avoiding_from_lidar_ = nh.subscribe("/start_avoiding",1,&PathTracking::avoiding_flag_callback,this);
	
	pub_cmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",1);
	
	timer_ = nh.createTimer(ros::Duration(0.03),&PathTracking::timer_callback,this);
	
	pub_info_ = nh.advertise<driverless_msgs::PathTrackingInfo>("/path_tracking_info",1);
	
	srv_driverless_ = nh.advertiseService("driverless_service",&PathTracking::driverlessService,this);
	
	nh_private.param<std::string>("path_file_dir",path_file_dir_,"");
	
	if(path_file_dir_.empty())
	{
		ROS_ERROR("no input path file directory !!!");
		return false;
	}

	nh_private.param<float>("max_speed",max_speed_,20.0);//km/h

	nh_private.param<float>("foreSightDis_speedCoefficient", foreSightDis_speedCoefficient_,1.8);
	nh_private.param<float>("foreSightDis_latErrCoefficient", foreSightDis_latErrCoefficient_,0.3);
	
	nh_private.param<float>("min_foresight_distance",min_foresight_distance_,4.0);
	nh_private.param<float>("wheel_base", wheel_base_, 1.5);
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::spinThread, this)));
	
	
	while(ros::ok() && !is_gps_data_valid(current_point_))
	{
		ROS_INFO("gps data is invalid, please check the gps topic or waiting...");
		sleep(1);
	}
	
	return true;
}

bool PathTracking::driverlessService(interface::Driverless::Request  &req,
									 interface::Driverless::Response &res)
{
	if(req.command_type == req.START)
	{
		if(this->status_ != Idle)
		{
			res.success = res.FAILED;
			return true;
		}
		fs::path file = fs::path(path_file_dir_)/req.path_file_name;
		if(!fs::exists(file))
		{
			res.success = res.FILE_NOT_EXISTS;
			return true;
		}
		if(req.path_type == req.CURVE_TYPE)
			this->status_ = CurveTracking;
		else if(req.path_type == req.VERTEX_TYPE)
			this->status_ = VertexTracking;
		else
		{
			res.success = res.PATH_TYPE_ERROR;
			return true;
		}
		
		tracking_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::pathTrackingThread, this,file,req.speed)));
	}
	else if(req.command_type == req.STOP)
		this->status_ = Stop;
	else if(req.command_type == req.SUSPEND)
		this->status_ = Suspend;
	else
	{
		res.success = res.CMD_TYPE_ERROR;
		return true;
	}
	
	res.success = res.OK;
	return true;
}

void PathTracking::pathTrackingThread(const fs::path& file, float speed)
{
	if(speed > max_speed_)
		speed = max_speed_;
		
	std::string file_name = fs::system_complete(file).string();
	if(status_ == CurveTracking)
	{
		path_points_.clear();
		loadPathPoints(file_name, path_points_);
	}
	else if(status_ == VertexTracking)
	{
		std::vector<gpsMsg_t> vertexes;
		loadPathPoints(file_name,vertexes);
		generatePathByVertexes(vertexes,path_points_,0.1);
	}
	
	ROS_INFO("path points size:%lu",path_points_.size());
	
	/*for(size_t i=0; i< path_points_.size();i++ )
	{
	  std::cout <<path_points_[i].x <<"\t " <<path_points_[i].y <<std::endl;
	}*/
	
	target_point_index_ = findNearestPoint(path_points_,current_point_);
	
	ROS_INFO("target_point_index_: %lu",target_point_index_);
	
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_INFO("target index:%lu,  file read over, No target point was found !!!",target_point_index_);
		this->status_ = Idle;
		return ;
	}
	
	target_point_ = path_points_[target_point_index_];
	
	ros::Rate loop_rate(30);
	
	int cnt = 0;
	
	while(ros::ok() && target_point_index_ < path_points_.size()-2)
	{
		if(status_ == Suspend)
			continue;
		else if(status_ == Idle)
			break;
			
		
		if( avoiding_offset_ != 0.0)
			target_point_ = pointOffset(target_point_,avoiding_offset_);
		
		try
		{
			lateral_err_ = calculateDis2path(current_point_.x,current_point_.y,path_points_,
											 target_point_index_,&nearest_point_index_) - avoiding_offset_;
		}catch(const char* str)
		{
			ROS_INFO("%s",str);
			break;
		}
		
		disThreshold_ = foreSightDis_latErrCoefficient_ * fabs(lateral_err_) + min_foresight_distance_;//??????
									 
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
		
		cmd_.set_speed = speed;
		
		cmd_.set_roadWheelAngle = t_roadWheelAngle;
		
		if(cnt%20==0)
		{
			ROS_INFO("dis2target:%.2f\t yaw_err:%.2f\t lat_err:%.2f",dis_yaw.first,yaw_err*180.0/M_PI,lateral_err_);
			ROS_INFO("disThreshold:%f\t expect roadwheel angle:%.2f",disThreshold_,t_roadWheelAngle);
			ROS_INFO("avoiding_offset_:%f\n",avoiding_offset_);
		}
		++cnt;
		this->publishInfo();
		loop_rate.sleep();
	}

	ROS_INFO("driverless completed..."); //send msg to screen ??????/

	this->status_ = Stop;

}

void PathTracking::run()
{
	rosSpin_thread_ptr_->join();
	tracking_thread_ptr_->join();
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
	if(this->status_ == Idle)
	{
		cmd_.driverless_mode = false;
	}
	else if(this->status_ == Suspend)
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 1;
	}
	else if(this->status_ == Stop)
	{
		static bool is_first = false;
		static ros::Time first_time;
		if(!is_first)
		{
			is_first = true;
			first_time = ros::Time::now();
		}
		else if((ros::Time::now() - first_time).toSec() > 10.0)
		{
			this->status_ = Idle;
			is_first = false;
		}
		cmd_.driverless_mode = true;
		cmd_.set_brake = 1;
	}
	else
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 0;
	}
	pub_cmd_.publish(cmd_);
}

void PathTracking::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_point_.x = msg->pose.pose.position.x;
	current_point_.y = msg->pose.pose.position.y;
	current_point_.yaw = msg->pose.covariance[0];
	
	current_point_.longitude = msg->pose.covariance[1];
	current_point_.latitude = msg->pose.covariance[2];
	
//	float speed = msg->north_velocity * msg->north_velocity + msg->east_velocity * msg->east_velocity;
//	current_speed_ = sqrt(speed)*3.6; //km/h
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
