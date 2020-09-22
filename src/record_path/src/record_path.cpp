#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <interface/RecordPath.h>
#include "auto_drive/structs.h"

#define __NAME__ "record_path"

/* current state   */
#define RecorderIdle 1
#define CurveRecording 2
#define VertexRecording 3

/*recod path response*/
#define Success 0
#define Fail    1

/* record path node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

class Recorder
{
public:
	Recorder();
	~Recorder();
	bool init();
	void recordToFile();
private:
	
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
	bool is_location_ok();
	std::string file_path_;
	
	FILE *fp_;
	
	gpsMsg_t last_point , current_point;
	
	float sample_distance_;
	ros::ServiceServer srv_record_path_;
	ros::Subscriber sub_utm_ ;
	std::string utm_topic_;
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
		ROS_ERROR("[%s] Please input record file path in launch file!", __NAME__);
		return false;
	}
	return true;
}

bool Recorder::openShortFile(const std::string& short_file)
{
	std::string file = file_path_ + short_file;
	fp_ = fopen(file.c_str(),"w");
	if(fp_ == NULL)
	{
		ROS_ERROR("[%s] Open %s failed!",__NAME__, file.c_str());
		return false;
	}
	else
		ROS_INFO("[%s] New path file: %s created.",__NAME__, file.c_str());
	return true;
}

bool Recorder::startCurvePathRecord()
{
		std::string file = file_path_ + req.path_file_name;
		
		
}

bool Recorder::startVertexPathRecord()
{

}

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

float Recorder::calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	return x*x+y*y;
}

void Recorder::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//自节点运行开始更新当前点信息
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	current_point.yaw = msg->pose.covariance[0];
	
	//如果不是曲线记录状态, 直接返回
	if(this->status_ != CurveRecording)
		return;
	
	//验证文件是否可用
	if(fp_ == NULL)
	{
		ROS_ERROR("[%s] system want to record curve path, buf the file is not open!", __NAME__);
		return ;
	}
	
	if(sample_distance_*sample_distance_ <= calculate_dis2(current_point,last_point))
	{
		ROS_INFO("[%s] recoding: %.3f\t%.3f\t%.3f\r\n",__NAME__, current_point.x,current_point.y,current_point.yaw);
		fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp_);

		last_point = current_point;
	}
}

