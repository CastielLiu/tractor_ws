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

	// wait for system ok
	while(ros::ok() && !is_gps_data_valid(pose_))
	{
		ROS_INFO("[AutoDrive]: gps data is invalid, please check the gps topic or waiting...");
		ros::Duration(0.5).sleep();
	}
    std::string cmd_topic = nh_private_.param<std::string>("cmd_topic","/cmd");
    pub_cmd_ = nh_.advertise<driverless_msgs::ControlCmd>(cmd_topic, 1);
	pub_state_ = nh_.advertise<std_msgs::UInt8>("/system_state", 1); //发布系统状态

	update_timer_ = nh_.createTimer(ros::Duration(0.05),&AutoDrive::update_timer_callback,this);

    srv_driverless_ = nh_.advertiseService("driverless_service", &AutoDrive::driverlessService, this);
	srv_recorder_   = nh_.advertiseService("record_path_service",&AutoDrive::recordPathService, this);
}

bool AutoDrive::driverlessService(interface::Driverless::Request  &req,
								  interface::Driverless::Response &res)
{
	//正在记录路径，无法启动自动驾驶
	//上位机逻辑正确的情况下不会出现这种情况
	if(state_.isRecording()) 
	{
		res.success = res.FAIL;
		return true;
	}

	//请求开始自动驾驶，首先中断当前已有的自动驾驶任务
	if(req.command_type == req.START)
	{
		state_.set(state_.State_SystemIdle); //将状态置为空闲，促使正在执行的自动驾驶退出(如果有)
		ros::Duration(0.5).sleep();   //等待正在自动驾驶的线程退出(如果有)
		int max_try_num = 5;
		while(!auto_drive_thread_mutex_.try_lock()) //尝试加锁, 失败:表明自动驾驶线程正在运行
		{
			if(--max_try_num ==0) //超出最大等待时长
			{
				res.success = res.FAIL;
				return true;
			}
			ros::Duration(0.2).sleep();
		}

		fs::path file = fs::path(path_file_dir_)/req.path_file_name;
		if(!fs::exists(file))
		{
		    ROS_ERROR("[%s] %s FileNotExist.",__NAME__,fs::system_complete(file).string().c_str());
			res.success = res.PATH_FILE_NOT_EXIST;
			return true;
		}
		std::string file_name = fs::system_complete(file).string();
		if(req.path_type == req.CURVE_TYPE) //连续型路径
		{
			state_.set(state_.State_CurveTracking); //状态置为连续型跟踪中
			path_.clear();
			
			//载入路径文件，并配置路径跟踪控制器
			if(!loadPath(file_name, path_) || !this->tracker_.setPath(path_))
			{
				res.success = res.PATH_FILE_ERROR;
				state_.set(state_.State_SystemIdle);
				return true;
			}
			ROS_INFO("[%s] Curve tracking path points size: %lu",__NAME__, path_.points.size());
		}
		else if(req.path_type == req.VERTEX_TYPE)
		{
			state_.set(state_.State_VertexTracking);
			path_t vertex_path;
			if(!loadPath(file_name,vertex_path) || !generatePathByVertexes(vertex_path, path_, 0.1))
			{
				res.success = res.PATH_FILE_ERROR;
				state_.set(state_.State_SystemIdle);
				return true;
			}
			ROS_INFO("[%s] Vertex tracking path points size:%lu",__NAME__, path_.points.size());
		}
		else
		{
			res.success = res.FAIL;
			state_.set(state_.State_SystemIdle);
			return true;
		}

		//初始化跟踪控制器
		tracker_.setPath(path_);
		if(!tracker_.init(pose_))
		{
			res.success = res.FAIL;
			state_.set(state_.State_SystemIdle);
			return true;
		}

		#if USE_AVOIDANCE
		//初始化避障控制器
		avoider_.setPath(path_);
		if(!avoider_.init())
		{
			res.success = res.FAIL;
			state_.set(state_.State_SystemIdle);
			return true;
		}
		#endif

		auto_drive_thread_ptr_ = 
            std::shared_ptr<std::thread >(new std::thread(std::bind(&AutoDrive::autoDriveThread, this,req.speed)));
	}
	else if(req.command_type == req.STOP)     //请求停止，状态置为完成
		state_.set(state_.State_CompleteTracking);
	else if(req.command_type == req.SUSPEND)  //请求暂停
		state_.set(state_.State_SuspendTracking);
	else if(req.command_type == req.CONFIRM_STOP) //确认停止，状态置为空闲，释放制动
		state_.set(state_.State_SystemIdle);
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
	std::lock_guard<std::mutex> lck(auto_drive_thread_mutex_);

	if(speed > max_speed_)
		speed = max_speed_;
	
	ros::Rate loop_rate(20);
	
	while(ros::ok())
	{
		if(state_.get() == state_.State_SuspendTracking)
		{
			loop_rate.sleep();
			continue;
		}

		if((state_.get() == state_.State_CompleteTracking) || 
		   (state_.get() == state_.State_SystemIdle))
			break;
		
		pose_wr_mutex_.lock_shared();
		bool update_state = tracker_.update(vehicle_speed_, roadwheel_angle_, pose_, avoid_offset_);
		pose_wr_mutex_.unlock_shared();

        if(!update_state) //更新失败,抵达目标地或出现异常
        {
			//此处状态为跟踪完成，待上位机确认后置为空闲
			state_.set(state_.State_CompleteTracking);
			ROS_INFO("[%s] reach the destination.", __NAME__);
			break;
		}
		else
        	tracker_.getTrackingCmd(cmd_.set_speed, cmd_.set_roadWheelAngle);
		loop_rate.sleep();
	} //自动驾驶完成
	
	ROS_INFO("[%s] automatic drive completed...",__NAME__); //send msg to screen 

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
	else if(state_.get()==state_.State_CompleteTracking ||
			state_.get()==state_.State_SuspendTracking)
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
	state.data = state_.get();
	pub_state_.publish(state);
}

void AutoDrive::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	std::unique_lock<std::shared_mutex> write_lock(pose_wr_mutex_);
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
	std::shared_lock<std::shared_mutex> read_lock(pose_wr_mutex_);
	return pose_;
}

bool AutoDrive::recordPathService(interface::RecordPath::Request  &req,
						   		  interface::RecordPath::Response &res)
{
	//正在路径跟踪，禁用路径记录
	//上位机逻辑正确的情况下不会出现这种情况
	if(state_.isTracking()) 
	{
		res.success = res.FAIL;
		return true;
	}

	if(req.command_type == req.START_RECORD_PATH ) //请求开始记录
	{
		if(state_.isRecording()) //正在记录
		{
			recorder_.stop(); //终止当前记录
			state_.set(state_.State_SystemIdle);
		}

		if(req.path_type ==req.CURVE_TYPE) //记录连续型路径
		{
			ROS_INFO("[%s] Request start record path: curve type.",__NAME__);
			state_.set(state_.State_CurvePathRecording);

			//开始连续型路径记录，传入文件名、位置获取函数
			bool ok = recorder_.startCurvePathRecord(req.path_file_name, &AutoDrive::currentPose, this);
			if(!ok)
			{
				res.success = Fail;
				return true;
			}
			
		}
		else if(req.path_type == req.VERTEX_TYPE)
		{
			ROS_INFO("[%s] Request start record path: vertex type.",__NAME__);
			state_.set(state_.State_VertexPathRecording);

			//开始顶点型路径记录
			bool ok = recorder_.startVertexPathRecord(req.path_file_name);
			if(!ok)
			{
				res.success = Fail;
				return true;
			}
		}
		else
		{
			ROS_ERROR("[%s] Expected path type error !",__NAME__);
			res.success = Fail;
			return true;
		}
	}
	//请求记录当前点,仅对顶点型有效
	else if(req.command_type == req.RECORD_CURRENT_POINT)
	{
		//若当前非顶点型记录中,大错误!
		if(this->status_ != VertexRecording)
		{
			ROS_ERROR("[%s] Request record current point, but system is in vertex recording!",__NAME__);
			res.success = Fail;
			return true;
		}
		float dis = dis2Points(current_point, last_point, true);
		
		if(dis < 1.0)
		{
			ROS_ERROR("[%s] Request record current point, but too close to the previous point." __NAME__);
			res.success = Fail;
			return true;
		}
		ROS_INFO("[%s] record current point: %.3f\t%.3f\t%.3f ok.",current_point.x,current_point.y,current_point.yaw*180.0/math.pi);
		fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp_);
		last_point = current_point;
	}
	//请求停止记录
	else if(req.command_type == req.STOP_RECORD_PATH)
	{
		sub_utm_.shutdown();
		//当前已经处于空闲状态,但仍然需要返回成功标志,
		//与请求记录时原理一致.
		if(this->status_ == RecorderIdle)
		{
			res.success = Success;
			if(fp_ != NULL)  //冗余判断
			{
				fclose(fp_);
				fp_ = NULL;
			}
			return true;
		}
		ROS_INFO("[%s] Record path completed.",__NAME__);
		fclose(fp_);
		fp_ = NULL;
		this->status_ = RecorderIdle;
	}
	
	res.success = Success;
	return true;
}