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
	sub_roadWheelAngle_ = 
		nh.subscribe("/roadWheelAngle",1,&Interface::roadWheelAngle_callback,this);
	timer_ = nh.createTimer(ros::Duration(0.5), &Interface::timer_callback,this);
	
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
		can2serial_->showCanMsg(can_msg);
		//switch(can_msg.ID)
		{
			
		}
	}
}

void Interface::gps_callback(const gps_msgs::Inspvax::ConstPtr& gps)
{
	*(int *)(info_.gps.data) = gps->longitude * 100000;
	*(int *)(info_.gps.data+4) = gps->latitude * 100000;
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
