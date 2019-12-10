#include<ros/ros.h>
#include <unistd.h>
#include<cmath>
#include<nav_msgs/Odometry.h>
#include<interface/RecordPath.h>
#include<path_tracking/function.h>

#ifndef PI_
#define PI_ 3.141592653589
#endif

#define RecorderIdle 1
#define CurveRecording 2
#define VertexRecording 3


/*recod path response*/
#define Success 0
#define Fail    1


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
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
	bool is_location_ok();
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
	
	sub_utm_ = nh.subscribe("/ll2utm",1,&Recorder::odom_callback,this);


	return true;
}

bool Recorder::recordPathService(interface::RecordPath::Request  &req,
								 interface::RecordPath::Response &res)
{
	if(!is_location_ok())
	{
		ROS_ERROR("Location status is abnormal, unable to record path");
		res.success = Fail;
		return true;
	}
	if(req.command_type == req.START_RECORD_PATH )
	{
		if(this->status_ != RecorderIdle)
		{
			ROS_ERROR("Recording in progress, instruction invalid!");
			res.success = Fail;
			return true;
		}
		ROS_INFO("command: START_RECORD_PATH");
		if(req.path_type ==req.CURVE_TYPE)
			this->status_ = CurveRecording;
		else if(req.path_type == req.VERTEX_TYPE)
			this->status_ = VertexRecording;
		else
		{
			ROS_ERROR("Expected path type error !");
			res.success = Fail;
			return true;
		}
		
		std::string file = file_path_ + req.path_file_name;
		
		fp_ = fopen(file.c_str(),"w");
		if(fp_ == NULL)
		{
			ROS_ERROR("open %s failed!",file.c_str());
			res.success = Fail;
			return true;
		}
		else
			ROS_INFO("New file: %s created.",file.c_str());
	}
	else if(req.command_type == req.RECORD_CURRENT_POINT)
	{
		if(this->status_ != VertexRecording)
		{
			res.success = Fail;
			return true;
		}
		static gpsMsg_t _last_point;
		float dis = dis2Points(current_point, _last_point, true);
		
		if(dis < 1.0)
		{
			res.success = Fail;
			return true;
		}
		ROS_INFO("RECORD_CURRENT_POINT: %.3f\t%.3f\t%.3f",current_point.x,current_point.y,current_point.yaw);
		fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp_);
	}
	else if(req.command_type == req.STOP_RECORD_PATH)
	{
		if(this->status_ == RecorderIdle)
		{
			res.success = Fail;
			return true;
		}
		ROS_INFO("RECORD_complete");
		fclose(fp_);
		fp_ = NULL;
		this->status_ = RecorderIdle;
	}
	
	res.success = Success;
	return true;
}


float Recorder::calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	return x*x+y*y;
}

bool Recorder::is_location_ok()
{
	if(fabs(current_point.x) < 1.0 && fabs(current_point.y) < 1.0)
		return false;
	return true;
}


void Recorder::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	current_point.yaw = msg->pose.covariance[0];
	
	if(this->status_ != CurveRecording)
		return;
	
	if(sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		printf("recoding: %.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
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


