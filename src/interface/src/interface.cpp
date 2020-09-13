#include <ros/ros.h>
#include <interface/interface.h>

/* can bus information interface node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

#define __NAME__ "interface_node"

Interface::Interface():
	gps_odom_flag_(false),
	tracking_info_flag_(false)
{
	can2serial_ = new Can2serial;
}

Interface::~Interface()
{
	delete can2serial_;
	can2serial_ = NULL;
}

bool Interface::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	std::string utm_topic = nh_private.param<std::string>("utm_topic","");
	
	if(utm_topic.empty())
	{
		ROS_ERROR("[%s] Please input utm_topic in launch file!", __NAME__);
		return false;
	}

	sub_gps_ = nh.subscribe(utm_topic, 1,&Interface::odom_callback,this);

	sub_pathtracking_info_ = 
		nh.subscribe("/path_tracking_info",1,&Interface::path_tracking_info_callback, this);

	sub_steerMoter_state_ = nh.subscribe("/steer_motor_state", 1, &Interface::steerMotorState_callback, this);
	sub_system_state_ = nh.subscribe("/drive_sytem_state", 1, &Interface::driveSystemState_callback, this);
	sub_brake_state_ = nh.subscribe("/brake_state", 1, &Interface::brakeSystemState_callback, this);
		
	msg_report_timer_ = nh.createTimer(ros::Duration(0.5), &Interface::msgReport_callback,this);
	heartbeat_timer_ = nh.createTimer(ros::Duration(0.5), &Interface::heartbeat_callback, this);
	
	//创建路径记录服务客户端
	client_recordPath_ = nh.serviceClient<interface::RecordPath>("record_path_service");
	//创建自动驾驶服务客户端
	client_driverless_ = nh.serviceClient<interface::Driverless>("driverless_service");
	//创建清除转向电机错误代码客户端
	client_clearMotorError_ = nh.serviceClient<std_srvs::Empty>("clear_motor_error_flag");
	
	//创建自动驾驶状态服务端
	server_driverless_status_ = nh.advertiseService("driverlessStatus_service",&Interface::driverlessStatusService, this);
	
	nh_private.param<std::string>("can2serial_port",can2serial_port_,"");
	nh_private.param<int>("can_baudrate",can_baudrate_,250);
	if(can2serial_port_.empty())
	{
		ROS_ERROR("[%s] Please input can2serial_port in launch file!", __NAME__);
		return false;
	}
	
	if(!can2serial_->configure_port(can2serial_port_))
	{
		ROS_ERROR("[%s] Configure can2serial failed!", __NAME__);
		return false;
	}
	
	can2serial_->configBaudrate(can_baudrate_);
	can2serial_->clearCanFilter();
	can2serial_->StartReading();
	
	return true;
}

void Interface::run()
{
	//接收can总线消息线程
	read_canMsg_thread_ = boost::shared_ptr<boost::thread >
		(new boost::thread(boost::bind(&Interface::readCanMsg, this)));

	ros::spin();
}

void Interface::readCanMsg()
{
	CanMsg_t can_msg;
	CanMsg_t can_msg_response;
	can_msg_response.ID = RESPONSE_CAN_ID;
	can_msg_response.len = 2;
	can_msg_response.type = Can2serial::STD_DATA_FRAME;//标准数据帧
	while(ros::ok())
	{
		if(!can2serial_->getCanMsg(can_msg))
		{
			usleep(10000);
			continue;
		}
		//can2serial_->showCanMsg(can_msg);
		switch(can_msg.ID)
		{
			int file_seq;
			case RECORD_PATH_CAN_ID: //记录路径相关can消息
				file_seq = can_msg.data[1]*256 + can_msg.data[0];        //文件序号
				srv_record_path_.request.path_type = can_msg.data[2];    //路径类型
				srv_record_path_.request.path_file_name = std::to_string(file_seq)+"_"  //文件名 = 序号+类型
														+ std::to_string(srv_record_path_.request.path_type)+".txt";
				
				srv_record_path_.request.command_type = can_msg.data[3]; //指令类型，开始?停止?记录当前?
				
				ROS_INFO("\n[%s] <记录路径> 路径类型: (%d->顶点型, %d->连续型)\t 指令类型: (%d->开始记录, %d->停止记录, %d->记录当前)", __NAME__, 
							srv_record_path_.request.VERTEX_TYPE, srv_record_path_.request.CURVE_TYPE,
							srv_record_path_.request.START_RECORD_PATH, srv_record_path_.request.STOP_RECORD_PATH, srv_record_path_.request.RECORD_CURRENT_POINT);
						
				ROS_INFO("[%s] Request recod path: %s\t type:%d\t cmd:%d",__NAME__, 
									srv_record_path_.request.path_file_name.c_str(),
									srv_record_path_.request.path_type,
									srv_record_path_.request.command_type);
									
				client_recordPath_.call(srv_record_path_);
				can_msg_response.data[0] = 0x00;//response record path
				can_msg_response.data[1] = srv_record_path_.response.success;
				can2serial_->sendCanMsg(can_msg_response); //response
				break;
			case DRIVERLESS_CAN_ID://请求自动驾驶相关can消息
				srv_driverless_.request.command_type = can_msg.data[3];   //指令类型
				srv_driverless_.request.path_type = can_msg.data[2];      //路径类型
				file_seq = can_msg.data[1]*256 + can_msg.data[0];         //路径序号
				// seq_type.txt
				srv_driverless_.request.path_file_name = std::to_string(file_seq)+"_" 
													   + std::to_string(srv_driverless_.request.path_type)+".txt";
				srv_driverless_.request.speed = can_msg.data[4];

				ROS_INFO("\n[%s] <自动驾驶> 路径类型: (%d->顶点型, %d->连续型)\t 指令类型: (%d->开始, %d->停止, %d->暂停， %d->停止确认)", __NAME__, 
							srv_driverless_.request.VERTEX_TYPE, srv_driverless_.request.CURVE_TYPE,
							srv_driverless_.request.START, srv_driverless_.request.STOP, srv_driverless_.request.SUSPEND, srv_driverless_.request.CONFIRM_STOP);

				ROS_INFO("[%s] Request auto drive:%s\t type:%d\t cmd:%d",__NAME__,
								 srv_driverless_.request.path_file_name.c_str(),
								 srv_driverless_.request.path_type,
								 srv_driverless_.request.command_type);
				client_driverless_.call(srv_driverless_);
				can_msg_response.data[0] = 0x01;//response driverless
				can_msg_response.data[1] = srv_driverless_.response.success;
				can2serial_->sendCanMsg(can_msg_response); //response
				break;
			
			case RESET_CAN_ID: //系统复位
			{
				systemResetMsg_t *resetMsg = (systemResetMsg_t *)can_msg.data;
				if(resetMsg->clearMotorError) //清除转向电机错误标志
				{
					std_srvs::Empty empty;
					client_clearMotorError_.call(empty);
				}
				else if(resetMsg->rebootMotor) //重启转向电机
				{
					;
				}

			    break;
			}
			
			default:
				ROS_ERROR("[%s] Unknown CAN ID." __NAME__);
				break;
		}
	}
}

void Interface::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double yaw = msg->pose.covariance[0] *180.0/M_PI;
	double longitude = msg->pose.covariance[1];
	double latitude = msg->pose.covariance[2];
	uint16_t height = msg->pose.pose.position.z * 10 + 2000;
	
	if(longitude > 0 && latitude >0)
		gps_odom_flag_ = true;
	else
		gps_odom_flag_ = false;

	*(uint32_t *)(info_.gpsPos.data) = uint32_t (longitude*10000000);
	*(uint32_t *)(info_.gpsPos.data+4) = uint32_t (latitude*10000000);
	
	*(uint16_t *)(info_.gpsMsg.data) = uint16_t(yaw*10); //yaw
	info_.gpsMsg.data[2] = 10 << 3;  //satellite_num << 3
	info_.gpsMsg.data[2] |= 0x01; // none differentiation positioning 
	*(uint16_t *)(info_.gpsMsg.data+6) = height;
}

void Interface::path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& info)
{
	tracking_info_flag_ = true;
	uint16_t speed = uint16_t(info->speed *10);
	info_.gpsMsg.data[3] = speed%256;
	info_.gpsMsg.data[4] = speed/256;
	
	uint16_t lateral_err = uint16_t(info->lateral_err*100) + 255;
//	ROS_INFO("lateral_err:%d",lateral_err);
	info_.gpsMsg.data[4] |= lateral_err%2 << 7;
	info_.gpsMsg.data[5] = lateral_err/2;
}

void Interface::steerMotorState_callback(const std_msgs::UInt8::ConstPtr& msg)
{

}

void Interface::driveSystemState_callback(const std_msgs::UInt8::ConstPtr& msg)
{

}

void Interface::brakeSystemState_callback(const std_msgs::UInt8::ConstPtr& msg)
{

}

bool Interface::driverlessStatusService(interface::DriverlessStatus::Request  &req,
							 			interface::DriverlessStatus::Response &res)
{
	ROS_INFO("[%s] auto drive complete. ",__NAME__);
	return true;
}

//状态数据定时上报
void Interface::msgReport_callback(const ros::TimerEvent& event)
{
	if(gps_odom_flag_)
	{
		can2serial_->sendCanMsg(info_.gpsPos);
//		can2serial_->showCanMsg(info_.gpsPos);
		usleep(1000);
	}
	if(!tracking_info_flag_)
	{	//若跟踪信息无效,横向偏差置0
		uint16_t lateral_err = uint16_t(0.0*100) + 255;
		info_.gpsMsg.data[4] |= lateral_err%2 << 7;
		info_.gpsMsg.data[5] = lateral_err/2;
	}
	
	can2serial_->sendCanMsg(info_.gpsMsg);
//	can2serial_->showCanMsg(info_.gpsMsg);
}

//发送心跳包
void Interface::heartbeat_callback(const ros::TimerEvent& event)
{
	heartbeatStruct_t *heartbeat = (heartbeatStruct_t *)info_.heartbeat.data;
	/*
	heartbeat->gpsState = 
    heartbeat->steerMotorState  = 
    heartbeat->brakeSystemState = 
    heartbeat->driveSystemState = 
	*/
	can2serial_->sendCanMsg(info_.heartbeat);

}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "interface_node");
	Interface interface;
	if(interface.init())
		interface.run();
	return 0;
}
