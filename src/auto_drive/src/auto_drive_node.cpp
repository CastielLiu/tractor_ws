#include"auto_drive/auto_drive.h"

/* current state   */
#define TrackerIdle    0
#define TrackerSuspend 1
#define TrackerStop    2
#define VertexTracking 3
#define CurveTracking  4

/*auto_drive response*/
#define Success         0
#define FileNotExist    1
#define Fail            2

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
	sub_utm_ = nh_.subscribe("/ll2utm",1,&AutoDrive::odom_callback,this);

	// wait for system ok
	while(ros::ok() && !is_gps_data_valid(vehicle_point_))
	{
		ROS_INFO("[AutoDrive]: gps data is invalid, please check the gps topic or waiting...");
		sleep(0.5);
	}

    pub_cmd_ = nh_.advertise<driverless_msgs::ControlCmd>("/cmd",1);
    srv_driverless_ = nh_.advertiseService("driverless_service",&AutoDrive::driverlessService,this);
	other_client_nh_ = nh_.serviceClient<interface::Other>("other_service");
}

bool AutoDrive::callOtherService(const std::string& data)
{
	interface::Other other;
	other.request.data = data;
	other_client_nh_.call(other);
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
			res.success = FileNotExist;
			return true;
		}
		if(req.path_type == req.CURVE_TYPE)
			this->status_ = CurveTracking;
		else if(req.path_type == req.VERTEX_TYPE)
			this->status_ = VertexTracking;
		else
		{
			res.success = Fail;
			return true;
		}
		
		auto_drive_thread_ptr_ = 
            boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&AutoDrive::autoDriveThread, this,file,req.speed)));
	}
	else if(req.command_type == req.STOP)
		this->status_ = TrackerStop;
	else if(req.command_type == req.SUSPEND)
		this->status_ = TrackerSuspend;
	else
	{
		res.success = Fail;
		return true;
	}
	
	res.success = Success;
	return true;
}

void AutoDrive::autoDriveThread(const fs::path& file, float speed)
{
	if(speed > max_speed_)
		speed = max_speed_;
		
	std::string file_name = fs::system_complete(file).string();
	if(status_ == CurveTracking)
	{
		path_.clear();
		loadPath(file_name, path_);
		ROS_INFO("CurveTracking__path points size:%lu",path_.points.size());
	}
	else if(status_ == VertexTracking)
	{
		path_t vertex_path;
		loadPath(file_name,vertex_path);
		generatePathByVertexes(vertex_path, path_, 0.1);
		ROS_INFO("VertexTracking__path points size:%lu",path_.points.size());
	}
	
	ROS_INFO("path points size:%lu",path_.points.size());
	
	//配置路径跟踪控制器
	this->tracker_.setPath(path_);
    if(!tracker_.init(vehicle_point_))
        return;
	//配置避障控制器
	avoider_.setPath(path_);
	if(!avoider_.init())
		return;
	
	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		if(status_ == TrackerSuspend)
			continue;
		else if(status_ == TrackerStop)
			break;
		bool update_state = tracker_.update(vehicle_speed_, roadwheel_angle_, vehicle_point_, avoid_offset_);
        if(update_state == 0)
        {
            usleep(0.01);
            continue;
        }
        else if(update_state == 2)
            break;
        tracker_.getTrackingCmd(cmd_.set_speed, cmd_.set_roadWheelAngle);
	}
	callOtherService("auto_drive_complete");
	
	ROS_INFO("driverless completed..."); //send msg to screen ??????/

	this->status_ = TrackerStop;
	avoider_.shutDown();
	
}

void AutoDrive::timer_callback(const ros::TimerEvent&)
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
}

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	vehicle_point_.x = msg->pose.pose.position.x;
	vehicle_point_.y = msg->pose.pose.position.y;
	vehicle_point_.yaw = msg->pose.covariance[0];
	
	vehicle_point_.longitude = msg->pose.covariance[1];
	vehicle_point_.latitude = msg->pose.covariance[2];
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(4);
	spinner.start(); //非阻塞

    AutoDrive auto_drive;
    if(auto_drive.init())
    	ros::waitForShutdown();
    return 0;
}  