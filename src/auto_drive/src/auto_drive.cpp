#include"auto_drive/auto_drive.h"

/* auto drive node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

/* current state   */
#define TrackerIdle    0
#define TrackerSuspend 1
#define TrackerStop    2
#define VertexTracking 3
#define CurveTracking  4
#define TrackerComplete 5

/*auto_drive response*/
#define Success         interface::Driverless::Response::SUCCESS
#define FileNotExist    interface::Driverless::Response::PATH_FILE_NOT_EXIST
#define Fail            interface::Driverless::Response::FAIL
#define FileError       interface::Driverless::Response::PATH_FILE_ERROR

#define __NAME__ "auto_drive"
#define USE_AVOIDANCE 0

AutoDrive::AutoDrive():
    nh_private_("~"),
    status_(TrackerIdle),
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
		if(this->status_ == VertexTracking || this->status_ == CurveTracking)
		{
			res.success = Success;
			return true;
		}
		fs::path file = fs::path(path_file_dir_)/req.path_file_name;
		if(!fs::exists(file))
		{
		    ROS_ERROR("[%s] %s FileNotExist.",__NAME__,fs::system_complete(file).string().c_str());
			res.success = FileNotExist;
			return true;
		}
		std::string file_name = fs::system_complete(file).string();
		if(req.path_type == req.CURVE_TYPE)
		{
			this->status_ = CurveTracking;
			path_.clear();
			bool ok = loadPath(file_name, path_);
			if(!ok)
			{
				res.success = FileError;
				return true;
			}
			ROS_INFO("[%s] Curve tracking path points size: %lu",__NAME__, path_.points.size());
		}
		else if(req.path_type == req.VERTEX_TYPE)
		{
			this->status_ = VertexTracking;
			path_t vertex_path;
			bool ok = loadPath(file_name,vertex_path);
			if(!ok)
			{
				res.success = FileError;
				return true;
			}
			generatePathByVertexes(vertex_path, path_, 0.1);
			ROS_INFO("[%s] Vertex tracking path points size:%lu",__NAME__, path_.points.size());
		}
		else
		{
			res.success = Fail;
			return true;
		}
		
		auto_drive_thread_ptr_ = 
            boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&AutoDrive::autoDriveThread, this,req.speed)));
	}
	else if(req.command_type == req.STOP)
		this->status_ = TrackerStop;
	else if(req.command_type == req.SUSPEND)
		this->status_ = TrackerSuspend;
	else if(req.command_type == req.CONFIRM_STOP)
		this->status_ = TrackerIdle;
	else
	{
		res.success = Fail;
		return true;
	}
	
	res.success = Success;
	return true;
}

void AutoDrive::autoDriveThread(float speed)
{
	if(speed > max_speed_)
		speed = max_speed_;
		
	//配置路径跟踪控制器
	this->tracker_.setPath(path_);
    if(!tracker_.init(vehicle_point_))
        return;

#if USE_AVOIDANCE
	//配置避障控制器
	avoider_.setPath(path_);
	if(!avoider_.init()) 
		return;
#endif
	
	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		if(status_ == TrackerSuspend)
		{
			loop_rate.sleep();
			continue;
		}
		if((status_ == TrackerStop) || (status_ == TrackerIdle))
			break;
			
		bool update_state = tracker_.update(vehicle_speed_, roadwheel_angle_, vehicle_point_, avoid_offset_);
        if(update_state == false)
        {
			//此处不退出程序，持续返回完成信息，直到确认退出
			callDriverlessStatusService(interface::DriverlessStatus::Request::NORMAL_EXIT);
			if((status_ == VertexTracking) || (status_ == CurveTracking))
			{
				status_ = TrackerComplete;
				ROS_INFO("[%s] reach the destination.", __NAME__);
			}
		}
		else
        	tracker_.getTrackingCmd(cmd_.set_speed, cmd_.set_roadWheelAngle);
		loop_rate.sleep();
	}
	
	ROS_INFO("[%s] automatic drive completed...",__NAME__); //send msg to screen ??????/

#if USE_AVOIDANCE
	avoider_.shutDown();
#endif
}

void AutoDrive::update_timer_callback(const ros::TimerEvent&)
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
		//停止跟踪一段时间后，将状态置为TrackerIdle
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
