#include<ros/ros.h>
#include<interface/interface.h>


Interface::Interface()
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
	
	sub_gps_ = nh.subscribe("/gps",1,&Interface::gps_callback,this);
	//sub_roadWheelAngle_ = 
	//	nh.subscribe("/roadWheelAngle",1,&Interface::roadWheelAngle_callback,this);
	sub_pathtracking_info_ = 
		nh.subscribe("/path_tracking_info",1,&Interface::path_tracking_info_callback, this);
		
	timer_ = nh.createTimer(ros::Duration(0.5), &Interface::timer_callback,this);
	
	client_recordPath_ = nh.serviceClient<interface::RecordPath>("record_path_service");
	client_driverless_ = nh.serviceClient<interface::Driverless>("driverless_service");
	
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
			case RECORD_PATH_CAN_ID:
				//int file_seq = can_msg.data[0];
				srv_record_path_.request.path_type = can_msg.data[2];
				// seq_type.txt
				srv_record_path_.request.path_file_name = std::to_string(can_msg.data[0])+"_"+ std::to_string(srv_record_path_.request.path_type)+".txt";
				srv_record_path_.request.command_type = can_msg.data[3];
				ROS_INFO("requestRecodPath:%s\ttype:%d\tcmd:%d",srv_record_path_.request.path_file_name.c_str(),srv_record_path_.request.path_type,srv_record_path_.request.command_type);
				client_recordPath_.call(srv_record_path_);
				can_msg.data[7] = srv_record_path_.response.success;
				can2serial_->sendCanMsg(can_msg); //response
				break;
			case DRIVERLESS_CAN_ID:
				srv_driverless_.request.command_type = can_msg.data[0];
				srv_driverless_.request.path_type = can_msg.data[1];
				//int file_seq = can_msg.data[2];
				// seq_type.txt
				srv_driverless_.request.path_file_name = std::to_string(can_msg.data[2]) +"_"+ std::to_string(srv_driverless_.request.path_type)+".txt";
				client_driverless_.call(srv_driverless_);
				can_msg.data[7] = srv_driverless_.response.success;
				can2serial_->sendCanMsg(can_msg); //response
				break;
		}
	}
}

void Interface::gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	*(uint32_t *)(info_.gps.data) = uint32_t ((gps->longitude+210)*10000000);
	*(uint32_t *)(info_.gps.data+4) = uint32_t ((gps->latitude+210)*10000000);
	
}

void Interface::path_tracking_info_callback(const driverless_msgs::PathTrackingInfo::ConstPtr& info)
{
	*(uint16_t *)(info_.status.data) = uint16_t(info->speed *100);
	*(uint16_t *)(info_.status.data+2) = uint16_t((info->lateral_err+3000)*10);
}

void Interface::roadWheelAngle_callback(const std_msgs::Float32 angle)
{
	
	
}

void Interface::timer_callback(const ros::TimerEvent& event)
{
	can2serial_->sendCanMsg(info_.gps);
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "interface_node");
	Interface interface;
	if(interface.init())
		interface.run();
	return 0;
}
