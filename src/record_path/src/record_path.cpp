#include<ros/ros.h>
#include "gps_msgs/Inspvax.h"
#include <unistd.h>
#include<cmath>
#include<gps_msgs/Utm.h>
#include<interface/RecordPath.h>
#include<path_tracking/function.h>

#ifndef PI_
#define PI_ 3.141592653589
#endif

#define RecorderIdle 1
#define CurveRecording 2
#define VertexRecording 3

class Recorder
{
public:
	Recorder();
	~Recorder();
	bool init();
	void recordToFile();
private:
	bool recordPathService(interface::RecordPath::Request  &req,
						   interface::RecordPath::Response &res);
	void utm_gps_callback(const gps_msgs::Utm::ConstPtr& msg);
	float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
	
	std::string file_path_;
	
	FILE *fp_;
	
	gpsMsg_t last_point , current_point;
	
	float sample_distance_;
	ros::ServiceServer srv_record_path_;
	ros::Subscriber sub_utm_ ;
	int status_;
};

Recorder::Recorder():
	status_(RecorderIdle)
{
	last_point = {0.0,0.0,0.0,0.0,0.0};
	current_point = last_point;
}

Recorder::~Recorder()
{
	if(fp_ != NULL)
		fclose(fp_);
}

bool Recorder::init()
{
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	
	private_nh.param<std::string>("file_path",file_path_,"");
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	
	if(file_path_.empty())
	{
		ROS_ERROR("please input record file path and file name in launch file!!!");
		return false;
	}
	
	srv_record_path_ = nh.advertiseService("record_path_service",&Recorder::recordPathService,this);
	
	sub_utm_ = nh.subscribe("/gps_utm",1,&Recorder::utm_gps_callback,this);


	return true;
}

bool Recorder::recordPathService(interface::RecordPath::Request  &req,
								 interface::RecordPath::Response &res)
{
	if(req.command_type == req.START_RECORD_PATH )
	{
		if(this->status_ != RecorderIdle)
		{
			res.success = false;
			return false;
		}
		//seq_type.txt
		std::string path_file_name = file_path_ + req.path_file_name 
			+"_"+std::to_string(req.path_type)+".txt";
		fp_ = fopen(path_file_name.c_str(),"w");
		if(fp_ == NULL)
		{
			ROS_ERROR("open %s failed!",path_file_name.c_str());
			res.success = false;
			return false;
		}
		if(req.path_type ==req.CURVE_TYPE)
			this->status_ = CurveRecording;
		else if(req.path_type == req.VERTEX_TYPE)
			this->status_ = VertexRecording;
		else
		{
			ROS_INFO("path type error !");
			res.success = false;
			return false;
		}
	}
	else if(req.command_type == req.RECORD_CURRENT_POINT)
	{
		if(this->status_ != VertexRecording)
		{
			res.success = false;
			return false;
		}
		fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp_);
	}
	else if(req.command_type == req.STOP_RECORD_PATH)
	{
		fclose(fp_);
		fp_ = NULL;
		this->status_ = RecorderIdle;
	}
	
	res.success = true;
	return true;
	
}


float Recorder::calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return x*x+y*y;
}

void Recorder::utm_gps_callback(const gps_msgs::Utm::ConstPtr& msg)
{
	if(this->status_ == RecorderIdle)
		return ;
	
	current_point.x = msg->x;
	current_point.y = msg->y;
	current_point.yaw = msg->yaw;
	
	if(this->status_ != CurveRecording)
		return;
	
	if(sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		//x,y,theta,offset_l,offset_r,traffic_sign
		fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp_);

		last_point = current_point;
	}
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_data_node");
	
	Recorder recorder;
	
	if(!recorder.init())
		return 1;

	ros::spin();
	
	return 0;
}


