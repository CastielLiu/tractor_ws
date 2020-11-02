#include"auto_drive/auto_drive.h"

/* auto drive node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

#define __NAME__ "auto_drive"
#define USE_AVOIDANCE 0

AutoDrive::AutoDrive():
    nh_private_("~"),
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
	sub_state_ = nh_.subscribe("/base_control_state", 1, &AutoDrive::base_ctrl_state_callback, this);


    std::string cmd_topic = nh_private_.param<std::string>("cmd_topic","/cmd");
    pub_cmd_ = nh_.advertise<driverless_msgs::ControlCmd>(cmd_topic, 1);
	pub_state_ = nh_.advertise<std_msgs::UInt8>("/system_state", 1); //发布系统状态

	update_timer_ = nh_.createTimer(ros::Duration(0.05),&AutoDrive::update_timer_callback,this); //控制指令发布定时器

    srv_driverless_ = nh_.advertiseService("driverless_service", &AutoDrive::driverlessService, this);
	srv_recorder_   = nh_.advertiseService("record_path_service",&AutoDrive::recordPathService, this);
	if(!recorder_.init()) return false;

	if(nh_private_.param<bool>("is_debug", false)) //debug mode
	{
		ROS_INFO("[%s] debug mode, not init steering motor.", __NAME__);
		return true;
	}

	// wait for system ok
	while(ros::ok() && !is_gps_data_valid(pose_))
	{
		ROS_INFO("[AutoDrive]: gps data is invalid, please check the gps topic or waiting...");
		ros::Duration(0.5).sleep();
	}
	return true;
}

bool AutoDrive::driverlessService(interface::Driverless::Request  &req,
								  interface::Driverless::Response &res)
{
	if(state_.isBusy() ||    //系统正忙
	   state_.isRecording()) //正在记录路径，无法启动自动驾驶 上位机逻辑正确的情况下不会出现这种情况
	{
		res.success = res.FAIL;
		return true;
	}

	//请求开始自动驾驶，首先中断当前已有的自动驾驶任务
	if(req.command_type == req.START)
	{
		state_.set(state_.State_SystemBusy); //将状态置为忙，正在执行的自动驾驶将自动退出
		ros::Duration(0.5).sleep();   //等待正在自动驾驶的线程退出(如果有)
		int max_try_num = 5;
		while(!auto_drive_thread_mutex_.try_lock()) //尝试加锁, 失败:表明自动驾驶线程正在运行
		{
			if(--max_try_num ==0) //超出最大等待时长
			{
				state_.set(state_.State_SystemIdle);
				ROS_ERROR("[%s] Another task is running now, please waiting...", __NAME__);
				res.success = res.FAIL;
				return true;
			}
			ros::Duration(0.1).sleep();
		}
		auto_drive_thread_mutex_.unlock(); //此处一定不能忘记解锁

		fs::path file = fs::path(path_file_dir_)/req.path_file_name;
		if(!fs::exists(file))
		{
			state_.set(state_.State_SystemIdle);
		    ROS_ERROR("[%s] %s FileNotExist.",__NAME__,fs::system_complete(file).string().c_str());
			res.success = res.PATH_FILE_NOT_EXIST;
			return true;
		}
		std::string file_name = fs::system_complete(file).string();
		if(req.path_type == req.CURVE_TYPE) //连续型路径
		{
			//载入路径文件，并配置路径跟踪控制器
			if(!loadPath(file_name, path_) || !this->tracker_.setPath(path_))
			{
				state_.set(state_.State_SystemIdle);
				ROS_ERROR("[%s] PATH_FILE_ERROR", __NAME__);
				res.success = res.PATH_FILE_ERROR;
				return true;
			}
		}
		else if(req.path_type == req.VERTEX_TYPE)
		{
			if(!loadPath(file_name,path_, 0.1))
			{
				state_.set(state_.State_SystemIdle);
				ROS_ERROR("[%s] PATH_FILE_ERROR", __NAME__);
				res.success = res.PATH_FILE_ERROR;
				return true;
			}
		}
		else
		{
			ROS_ERROR("[%s] Unknown path type", __NAME__);
			res.success = res.FAIL;
			state_.set(state_.State_SystemIdle);
			return true;
		}

		//初始化跟踪控制器
		tracker_.setPath(path_);
		if(!tracker_.init(pose_))
		{
			ROS_ERROR("[%s] init tracker failed.", __NAME__);
			res.success = res.FAIL;
			state_.set(state_.State_SystemIdle);
			return true;
		}
		ROS_INFO("[%s] init tracker complete.", __NAME__);

		#if USE_AVOIDANCE
		//初始化避障控制器
		avoider_.setPath(path_);
		if(!avoider_.init())
		{
			ROS_ERROR("[%s] init avoider failed.", __NAME__);
			res.success = res.FAIL;
			state_.set(state_.State_SystemIdle);
			return true;
		}
		ROS_INFO("[%s] init avoider complete.", __NAME__);
		#endif

		if(req.path_type == req.VERTEX_TYPE)
		{
			state_.set(state_.State_VertexTracking);
			ROS_INFO("[%s] Vertex tracking path points size:%lu",__NAME__, path_.points.size());
		}
		else if(req.path_type == req.CURVE_TYPE)
		{
			state_.set(state_.State_CurveTracking); //状态置为连续型跟踪中
			ROS_INFO("[%s] Curve tracking path points size: %lu",__NAME__, path_.points.size());
		}
		std::thread t(std::bind(&AutoDrive::autoDriveThread, this,req.speed));
		t.detach();
	}
	else if(req.command_type == req.STOP)     //请求停止，状态置为完成
		state_.set(state_.State_CompleteTrack);
	else if(req.command_type == req.SUSPEND)  //请求暂停
		state_.set(state_.State_SuspendTrack);
	else if(req.command_type == req.CONFIRM_STOP) //确认停止，状态置为空闲，释放制动
		state_.set(state_.State_SystemIdle);
	else
	{
		ROS_ERROR("[%s] Unknown command type", __NAME__);
		res.success = res.FAIL;
		return true;
	}
	
	res.success = res.SUCCESS;
	return true;
}

void AutoDrive::autoDriveThread(float speed)
{
	std::lock_guard<std::mutex> lck(auto_drive_thread_mutex_);

	if(speed > max_speed_)
		speed = max_speed_;
	
	ros::Rate loop_rate(20);
	
	while(ros::ok() && state_.isTracking())
	{
		if(state_.get() == state_.State_SuspendTrack)
		{
			loop_rate.sleep();
			continue;
		}
		
		pose_wr_mutex_.lock_shared();
		bool update_state = tracker_.update(vehicle_speed_, roadwheel_angle_, pose_, avoid_offset_);
		pose_wr_mutex_.unlock_shared();

        if(!update_state) //更新失败,抵达目标地或出现异常
        {
			//此处状态为跟踪完成，待上位机确认后置为空闲
			state_.set(state_.State_CompleteTrack);
			ROS_INFO("[%s] reach the destination.", __NAME__);
			break;
		}
		else
        	tracker_.getTrackingCmd(cmd_.set_speed, cmd_.set_roadWheelAngle);
		loop_rate.sleep();
	} //自动驾驶完成
	
	ROS_INFO("[%s] auto drive completed...",__NAME__); 

#if USE_AVOIDANCE
	avoider_.shutDown();
#endif
}

void AutoDrive::update_timer_callback(const ros::TimerEvent&)
{
	if(state_.get()==state_.State_VertexTracking ||
	   state_.get()==state_.State_CurveTracking)
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 0;
	}
	else if(state_.get()==state_.State_CompleteTrack ||
			state_.get()==state_.State_SuspendTrack)
	{
		cmd_.driverless_mode = true;
		cmd_.set_brake = 5;
	}
	else
	{
		cmd_.driverless_mode = false;
		cmd_.set_brake = 0;
	}

	pub_cmd_.publish(cmd_);

	std_msgs::UInt8 state;
	state.data = this->state_.get();
	pub_state_.publish(state);
}

bool AutoDrive::recordPathService(interface::RecordPath::Request  &req,
						   		  interface::RecordPath::Response &res)
{
	if(   state_.isBusy()      //系统正忙, 防止指令重复请求
	   || state_.isTracking()) //正在路径跟踪 上位机逻辑正确的情况下不会出现这种情况
	{
		res.success = res.FAIL;
		return true;
	}

	if(req.command_type == req.START_RECORD_PATH ) //请求开始记录
	{
		if(state_.isRecording()) //正在记录
			recorder_.stopWithoutSave(); //强制终止当前记录

		state_.set(state_.State_SystemBusy);

		if(req.path_type ==req.CURVE_TYPE) //记录连续型路径
		{
			ROS_INFO("[%s] Request start record path: curve type.",__NAME__);
		
			//开始连续型路径记录，传入文件名、位置获取函数
			bool ok = recorder_.startCurvePathRecord(req.path_file_name, &AutoDrive::currentPose, this);
			if(!ok)
			{
				state_.set(state_.State_SystemIdle);
				res.success = res.FAIL;
				return true;
			}
			state_.set(state_.State_CurvePathRecording);
		}
		else if(req.path_type == req.VERTEX_TYPE) //顶点型路径记录
		{
			ROS_INFO("[%s] Request start record path: vertex type.",__NAME__);

			//开始顶点型路径记录
			bool ok = recorder_.startVertexPathRecord(req.path_file_name);
			if(!ok)
			{
				state_.set(state_.State_SystemIdle);
				res.success = res.FAIL;
				return true;
			}
			state_.set(state_.State_VertexPathRecording);
		}
		else
		{
			state_.set(state_.State_SystemIdle);
			ROS_ERROR("[%s] Expected path type error !",__NAME__);
			res.success = res.FAIL;
			return true;
		}
	}
	//请求记录当前点,仅对顶点型有效
	else if(req.command_type == req.RECORD_CURRENT_POINT)
	{
		//若当前非顶点型记录中,上位机逻辑错误
		if(state_.get() != state_.State_VertexPathRecording)
		{
			ROS_ERROR("[%s] Request record current point, but system is not in vertex recording!",__NAME__);
			res.success = res.FAIL;
			return true;
		}

		bool ok = recorder_.recordCurrentVertex(pose_);
		if(!ok)
		{
			res.success = res.FAIL;
			return true;
		}
	}
	//请求停止记录
	else if(req.command_type == req.STOP_RECORD_PATH)
	{
		if(!state_.isRecording()) //未处于记录状态
		{
			recorder_.stopWithoutSave();
			res.success = res.SUCCESS;
			return true;
		}
		ROS_INFO("[%s] Record path completed.",__NAME__);
		recorder_.stopAndSave();
		state_.set(state_.State_SystemIdle);
	}
	
	ROS_INFO("[%s] request complete.", __NAME__);
	res.success = res.SUCCESS;
	return true;
}

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	
	write_lock_t write_lock(pose_wr_mutex_);
	pose_.x = msg->pose.pose.position.x;
	pose_.y = msg->pose.pose.position.y;
	pose_.yaw = msg->pose.covariance[0];
	
	pose_.longitude = msg->pose.covariance[1];
	pose_.latitude = msg->pose.covariance[2];
}

//获取前轮转角值
void AutoDrive::base_ctrl_state_callback(const driverless_msgs::BaseControlState::ConstPtr& msg)
{
    roadwheel_angle_ = msg->roadWheelAngle;
}

//作为回调函数为调用方提供信息
const gpsMsg_t AutoDrive::currentPose()
{
	//ROS_INFO("const gpsMsg_t AutoDrive::currentPose()_");
	read_lock_t read_lock(pose_wr_mutex_);
	return pose_;
}
