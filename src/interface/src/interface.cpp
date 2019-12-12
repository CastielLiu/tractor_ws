#include <ros/ros.h>
#include <interface/interface.h>


Interface::Interface():
	odom_flag_(false),
	tracking_info_flag_(false)
{
	can2serial_ = new Can2serial;
}

Interface::~Interface()
{
	delete can2serial_;
}

bool Interface::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	sub_gps_ = nh.subscribe("/ll2utm",1,&Interface::odom_callback,this);

	sub_pathtracking_info_ = 
		nh.subscribe("/path_tracking_info",1,&Interface::path_tracking_info_callback, this);
		
	timer_ = nh.createTimer(ros::Duration(0.5), &Interface::timer_callback,this);
	
	client_recordPath_ = nh.serviceClient<interface::RecordPath>("record_path_service");
	client_driverless_ = nh.serviceClient<interface::Driverless>("driverless_service");
	other_srv_nh_ = nh.advertiseService("other_service",&Interface::otherService, this);
	
	nh_private.param<std::string>("can2serial_port",can2serial_port_,"");
	nh_private.param<int>("can_baudrate",can_baudrate_,250);
	if(can2serial_port_.empty())
	{
		ROS_ERROR("please input can2serial_port!");
		return false;
	}
	
	if(!can2serial_->configure_port(can2serial_port_))
		return false;
	
	can2serial_->configBaudrate(can_baudrate_);
	
	can2serial_->clearCanFilter();
	
	can2serial_->StartReading();
	
	return true;
}

void Interface::run()
{
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
	can_msg_response.type = Can2serial::STD_DATA_FRAME;//STD_DATA_FRAME = 0x03
	while(ros::ok())
	{
		if(!can2serial_->getCanMsg(can_msg))
		{
			usleep(10000);
			continue;
		}
		can2serial_->showCanMsg(can_msg);
		switch(can_msg.ID)
		{
			int file_seq;
			case RECORD_PATH_CAN_ID:
				file_seq = can_msg.data[1]*256 + can_msg.data[0];
				srv_record_path_.request.path_type = can_msg.data[2];
				srv_record_path_.request.path_file_name = std::to_string(file_seq)+"_"+ std::to_string(srv_record_path_.request.path_type)+".txt";
				srv_record_path_.request.command_type = can_msg.data[3];
				ROS_INFO("request recod path: %s\ttype:%d\tcmd:%d",srv_record_path_.request.path_file_name.c_str(),srv_record_path_.request.path_type,srv_record_path_.request.command_type);
				client_recordPath_.call(srv_record_path_);
				can_msg_response.data[0] = 0x00;//response record path
				can_msg_response.data[1] = srv_record_path_.response.success;
				can2serial_->sendCanMsg(can_msg_response); //response
				break;
			case DRIVERLESS_CAN_ID:
				srv_driverless_.request.command_type = can_msg.data[3];
				srv_driverless_.request.path_type = can_msg.data[2];
				file_seq = can_msg.data[1]*256 + can_msg.data[0];
				// seq_type.txt
				srv_driverless_.request.path_file_name = std::to_string(file_seq)+"_"+ std::to_string(srv_driverless_.request.path_type)+".txt";
				srv_driverless_.request.speed = can_msg.data[4];
				ROS_INFO("requestDriveless:%s\ttype:%d\tcmd:%d",srv_driverless_.request.path_file_name.c_str(),srv_driverless_.request.path_type,srv_driverless_.request.command_type);
				client_driverless_.call(srv_driverless_);
				can_msg_response.data[0] = 0x01;//response driverless
				can_msg_response.data[1] = srv_driverless_.response.success;
				can2serial_->sendCanMsg(can_msg_response); //response
				break;
			default:
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
	
	if(longitude > 0 && latitude >0 )
		odom_flag_ = true;
	else
		odom_flag_ = false;

	*(uint32_t *)(info_.gps.data) = uint32_t (longitude*10000000);
	*(uint32_t *)(info_.gps.data+4) = uint32_t (latitude*10000000);
	
	*(uint16_t *)(info_.status.data) = uint16_t(yaw*10);
	info_.status.data[2] = 10 << 3;  //satellite_num << 3
	info_.status.data[2] |= 0x01; // none differentiation positioning
	*(uint16_t *)(info_.status.data+6) = height;
}

void Interface::path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& info)
{
	tracking_info_flag_ = true;
	uint16_t speed = uint16_t(info->speed *10);
	info_.status.data[3] = speed%256;
	info_.status.data[4] = speed/256;
	
	uint16_t lateral_err = uint16_t(info->lateral_err*100) + 255;
//	ROS_INFO("lateral_err:%d",lateral_err);
	info_.status.data[4] |= lateral_err%2 << 7;
	info_.status.data[5] = lateral_err/2;
}

bool Interface::otherService(interface::Other::Request  &req,
							 interface::Other::Response &res)
{
	if(req.data == "path tracking complete")
	{
		;
	}
	
	return true;
}

void Interface::timer_callback(const ros::TimerEvent& event)
{
	if(odom_flag_)
	{
		can2serial_->sendCanMsg(info_.gps);
//		can2serial_->showCanMsg(info_.gps);
		usleep(1000);
	}
	if(!tracking_info_flag_)
	{
		uint16_t lateral_err = uint16_t(0.0*100) + 255;
		info_.status.data[4] |= lateral_err%2 << 7;
		info_.status.data[5] = lateral_err/2;
	}
	
	can2serial_->sendCanMsg(info_.status);
//	can2serial_->showCanMsg(info_.status);
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "interface_node");
	Interface interface;
	if(interface.init())
		interface.run();
	return 0;
}
