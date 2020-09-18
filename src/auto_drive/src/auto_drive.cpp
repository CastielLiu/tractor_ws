#include"auto_drive/auto_drive.h"

/* auto drive node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

/* current state   */
#define SystemIdle    0      
#define TrackerSuspend 1
#define VertexTracking 3
#define CurveTracking  4
#define TrackerComplete 5
#define RecordCurvePath 6
#define RecordVertexPath 7


#define __NAME__ "auto_drive"
#define USE_AVOIDANCE 0

AutoDrive::AutoDrive():
    nh_private_("~"),
    status_(SystemIdle),
    avoid_offset_(0.0),
    tracker_(nh_, nh_private_),
    avoider_(nh_, nh_private_)
{
    cmd_.set_speed =0.0;
	cmd_.set_roadWheelAngle =0.0;
}

AutoDrive::~AutoDrive()
{

}

bool AutoDrive::init()
{
    nh_private_.param<float>("max_speed",max_speed_,20.0);//km/h
    nh_private_.param<std::string>("path_file_dir",path_file_dir_,"");
	if(path_file_dir_.empty())
	{
		ROS_ERROR("no input path file directory !!!");
		return false;
	}
	
	std::string utm_topic=nh_private_.param<std::string>("utm_topic","utm_topic");
	sub_utm_ = nh_.subscribe(utm_topic, 1,&AutoDrive::odom_callback,this);
	sub_state_ = nh_.subscribe("/state", 1, &AutoDrive::state_callback, this);

	// wait for system ok
	while(ros::ok() && !is_gps_data_valid(vehicle_point_))
	{
		ROS_INFO("[AutoDrive]: gps data is invalid, please check the gps topic or waiting...");
		ros::Duration(0.5).sleep();
	}
    std::string cmd_topic = nh_private_.param<std::string>("cmd_topic","/cmd");
    pub_cmd_ = nh_.advertise<driverless_msgs::ControlCmd>(cmd_topic, 1);
	pub_state_ = nh_.advertise<std_msgs::UInt8>("/system_state", 1); //发布系统状态

	update_timer_ = nh_.createTimer(ros::Duration(0.05),&AutoDrive::update_timer_callback,this);

    srv_driverless_ = nh_.advertiseService("driverless_service",&AutoDrive::driverlessService,this);
	driverless_status_client_nh_ = nh_.serviceClient<interface::DriverlessStatus>("driverlessStatus_service");
}

//反馈自动驾驶状态
bool AutoDrive::callDriverlessStatusService(uint8_t status)
{
	interface::DriverlessStatus srv;
	srv.request.status = status;
	driverless_status_client_nh_.call(srv);
	return srv.response.success;
}

bool AutoDrive::driverlessService(interface::Driverless::Request  &req,
								  interface::Driverless::Response &res)
{
	if(req.command_type == req.START)
	{
		//应判断新请求与当前状态是否一致，甚至文件名是否一致！
		if(this->status_ == VertexTracking || this->status_ == CurveTracking)
		{
			res.success = res.SUCCESS;
			return true;
		}
		fs::path file = fs::path(path_file_dir_)/req.path_file_name;
		if(!fs::exists(file))
		{
		    ROS_ERROR("[%s] %s FileNotExist.",__NAME__,fs::system_complete(file).string().c_str());
			res.success = res.PATH_FILE_NOT_EXIST;
			return true;
		}
		std::string file_name = fs::system_complete(file).string();
		if(req.path_type == req.CURVE_TYPE)
		{
			this->status_ = CurveTracking;
			path_.clear();
			
			//载入路径文件，并配置路径跟踪控制器
			if(!loadPath(file_name, path_) || !this->tracker_.setPath(path_))
			{
				res.success = res.PATH_FILE_ERROR;
				this->status_ = SystemIdle;
				return true;
			}
			ROS_INFO("[%s] Curve tracking path points size: %lu",__NAME__, path_.points.size());
		}
		else if(req.path_type == req.VERTEX_TYPE)
		{
			this->status_ = VertexTracking;
			path_t vertex_path;
			if(!loadPath(file_name,vertex_path) || !generatePathByVertexes(vertex_path, path_, 0.1))
			{
				res.success = res.PATH_FILE_ERROR;
				this->status_ = SystemIdle;
				return true;
			}
			ROS_INFO("[%s] Vertex tracking path points size:%lu",__NAME__, path_.points.size());
		}
		else
		{
			res.success = res.FAIL;
			this->status_ = SystemIdle;
			return true;
		}

		if(req.path_type == req.VERTEX_TYPE || req.path_type == req.CURVE_TYPE)
		{
			if(!tracker_.init(vehicle_point_))
			{
				res.success = res.FAIL;
				this->status_ = SystemIdle;
				return true;
			}

			#if USE_AVOIDANCE
			//配置避障控制器
			avoider_.setPath(path_);
			if(!avoider_.init()) 
			{
				res.success = res.FAIL;
				this->status_ = SystemIdle;
				return true;
			}
			#endif
		}
		
		auto_drive_thread_ptr_ = 
            boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&AutoDrive::autoDriveThread, this,req.speed)));
	}
	else if(req.command_type == req.STOP)
		this->status_ = TrackerComplete;
	else if(req.command_type == req.SUSPEND)
		this->status_ = TrackerSuspend;
	else if(req.command_type == req.CONFIRM_STOP)
		this->status_ = SystemIdle;
	else
	{
		res.success = res.FAIL;
		return true;
	}
	
	res.success = res.SUCCESS;
	return true;
}

void AutoDrive::autoDriveThread(float speed)
{
	if(speed > max_speed_)
		speed = max_speed_;
	
	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		if(status_ == TrackerSuspend)
		{
			loop_rate.sleep();
			continue;
		}
		if((status_ == TrackerComplete) || (status_ == SystemIdle))
			break;
			
		bool update_state = tracker_.update(vehicle_speed_, roadwheel_angle_, vehicle_point_, avoid_offset_);
        if(update_state == false)
        {
			callDriverlessStatusService(interface::DriverlessStatus::Request::NORMAL_EXIT);
			if((status_ == VertexTracking) || (status_ == CurveTracking))
			{
				//此处状态为跟踪完成，待上位机确认后置为空闲
				status_ = TrackerComplete;
				ROS_INFO("[%s] reach the destination.", __NAME__);
			}
		}
		else
        	tracker_.getTrackingCmd(cmd_.set_speed, cmd_.set_roadWheelAngle);
		loop_rate.sleep();
	} //自动驾驶完成
	
	ROS_INFO("[%s] automatic drive completed...",__NAME__); //send msg to screen ??????/

#if USE_AVOIDANCE
	avoider_.shutDown();
#endif
}

void AutoDrive::update_timer_callback(const ros::TimerEvent&)
{
	if(this->status_ == SystemIdle)
	{
		cmd_.driverless_mode = false;
		cmd_.set_brake = 0;
	}
	else if(this->status_ == TrackerSuspend)
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 5;
	}
	else if(this->status_ == TrackerComplete)
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 5;
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

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vehicle_point_.x = msg->pose.pose.position.x;
	vehicle_point_.y = msg->pose.pose.position.y;
	vehicle_point_.yaw = msg->pose.covariance[0];
	
	vehicle_point_.longitude = msg->pose.covariance[1];
	vehicle_point_.latitude = msg->pose.covariance[2];
}

void AutoDrive::state_callback(const driverless_msgs::State::ConstPtr& msg)
{
    roadwheel_angle_ = msg->roadWheelAngle;
}
