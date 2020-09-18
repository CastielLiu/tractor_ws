#include "path_tracking/path_tracking.h"

/* current state   */
#define TrackerIdle    0
#define TrackerSuspend 1
#define TrackerStop    2
#define VertexTracking 3
#define CurveTracking  4

#define __NAME__ "PathTracking"

PathTracking::PathTracking():
	target_point_index_(0),
	nearest_point_index_(0),
	avoiding_offset_(0.0),
	max_roadwheelAngle_(25.0),
	is_avoiding_(false),
	status_(TrackerIdle)
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
	sub_path_offset_ = nh.subscribe("/path_offset",1,&PathTracking::avoiding_flag_callback,this);
	
	timer_ = nh.createTimer(ros::Duration(0.05),&PathTracking::timer_callback,this);

	pub_cmd_ = nh.advertise<driverless_msgs::ControlCmd>("/cmd",1);
	pub_info_ = nh.advertise<driverless_msgs::PathTrackingInfo>("/tracking_info",1);
	pub_state_ = nh.advertise<std_msgs::UInt8>("/system_state", 1);
	
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
	nh_private.param<float>("wheel_base", wheel_base_, 2.0);
	
	//start the ros::spin() thread
	rosSpin_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::spinThread, this)));
	
	
	while(ros::ok() && !is_gps_data_valid(current_point_))
	{
		ROS_INFO("[%s]: gps data is invalid, please check the gps topic or waiting...", __NAME__);
		sleep(1);
	}
	return true;
}

bool PathTracking::driverlessService(interface::Driverless::Request  &req,
									 interface::Driverless::Response &res)
{
	if(req.command_type == req.START)
	{
		if(this->status_ == VertexTracking || this->status_ == CurveTracking)
		{
			res.success = res.SUCCESS;
			return true;
		}
		fs::path file = fs::path(path_file_dir_)/req.path_file_name;
		std::string file_name = fs::system_complete(file).string();
		if(!fs::exists(file))
		{
			res.success = res.PATH_FILE_NOT_EXIST;
			return true;
		}
		if(req.path_type == req.CURVE_TYPE)
		{
			this->status_ = CurveTracking;
			path_points_.clear();
			bool loadOk = loadPathPoints(file_name, path_points_);
			if(!loadOk)
			{
				res.success = res.PATH_FILE_ERROR;
				return true;
			}
			bool extendOk = this->extendGlobalPath(10);
			if(!extendOk)
			{
				res.success = res.PATH_FILE_ERROR;
				return true;
			}

			ROS_INFO("[%s] CurveTracking path points size:%lu", __NAME__, path_points_.size());
		}
		else if(req.path_type == req.VERTEX_TYPE)
		{
			this->status_ = VertexTracking;
			std::vector<gpsMsg_t> vertexes;
			bool loadOk = loadPathPoints(file_name,vertexes);
			if(!loadOk)
			{
				res.success = res.PATH_FILE_ERROR;
				return true;
			}
			bool ok = generatePathByVertexes(vertexes,path_points_,0.1);
			if(!ok)
			{
				res.success = res.PATH_FILE_ERROR;
				return true;
			}

			ok = this->extendGlobalPath(10);
			if(!ok)
			{
				res.success = res.PATH_FILE_ERROR;
				return true;
			}

			ROS_INFO("[%s] VertexTracking path points size:%lu",__NAME__, path_points_.size());
		}
		else
		{
			res.success = res.FAIL;
			return true;
		}
		//启动路径跟踪线程
		tracking_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&PathTracking::pathTrackingThread, this,file,req.speed)));
	}
	else if(req.command_type == req.STOP)
		this->status_ = TrackerStop;
	else if(req.command_type == req.SUSPEND)
		this->status_ = TrackerSuspend;
	else
	{
		res.success = res.FAIL;
		return true;
	}
	
	res.success = res.SUCCESS;
	return true;
}

/*@brief 拓展全局路径,防止车辆临近终点时无法预瞄
 *@param extendDis 拓展长度，保证实际终点距离虚拟终点大于等于extendDis
 */
bool PathTracking::extendGlobalPath(float extendDis)
{
	//取最后一个点与倒数第n个点的连线向后插值
	//总路径点不足n个,退出
	int n = 5;
	//std::cout << "extendGlobalPath: " << path_points_.size() << "\t" << path_points_.size()-1 << std::endl;
	if(path_points_.size()-1 < n)
	{
		ROS_ERROR("[%s] global path points is too few (%d), extend global path failed",path_points_.size()-1, __NAME__);
		return false;
	}
	int endIndex = path_points_.size()-1;
	
	float dx = (path_points_[endIndex].x - path_points_[endIndex-n].x)/n;
	float dy = (path_points_[endIndex].y - path_points_[endIndex-n].y)/n;
	float ds = sqrt(dx*dx+dy*dy);

	gpsMsg_t point;
	float remaindDis = 0.0;
	for(size_t i=1;;++i)
	{
		point.x = path_points_[endIndex].x + dx*i;
		point.y = path_points_[endIndex].y + dy*i;
		point.curvature = 0.0;
		path_points_.push_back(point);
		remaindDis += ds;
		if(remaindDis > extendDis)
			break;
	}
	return true;
}

//建立状态反馈定时器……

void PathTracking::pathTrackingThread(const fs::path& file, float speed)
{
	if(speed > max_speed_)
		speed = max_speed_;
	
	target_point_index_ = findNearestPoint(path_points_,current_point_);
	
	ROS_INFO("nearest_point_index: %lu",target_point_index_);
	
	if(target_point_index_ > path_points_.size() - 10)
	{
		ROS_INFO("target index:%lu,  file read over, No target point was found !!!",target_point_index_);
		this->status_ = TrackerIdle;
		return ;
	}
	
	target_point_ = path_points_[target_point_index_];
	
	ros::Rate loop_rate(20);
	
	int cnt = 0;
	
	while(ros::ok() && target_point_index_ < path_points_.size()-2)
	{
		if(status_ == TrackerSuspend)
		{
			loop_rate.sleep();
			continue;
		}
		else if(status_ == TrackerStop)
			break;
			
//		ROS_INFO("target_point_index_: %d", target_point_index_);
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
	} //路径跟踪完成

	this->status_ = TrackerStop;
	
	ROS_INFO("driverless completed..."); 
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
	if(this->status_ == TrackerIdle)
	{
		cmd_.driverless_mode = false;
	}
	else if(this->status_ == TrackerSuspend)
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 1;
	}
	else if(this->status_ == TrackerStop)
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
			this->status_ = TrackerIdle;
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

	std_msgs::UInt8 state;
	state.data = this->status_;
	pub_state_.publish(state);
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
