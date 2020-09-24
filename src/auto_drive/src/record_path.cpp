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
	
	private_nh.param<std::string>("file_path",file_dir_,"");
	private_nh.param<float>("sample_distance",sample_distance_,0.1);
	
	
	if(file_path_.empty())
	{
		ROS_ERROR("[%s] Please input record file path in launch file!", __NAME__);
		return false;
	}
	
	//创建服务,用于外部触发记录路径
	srv_record_path_ = nh.advertiseService("record_path_service",&Recorder::recordPathService,this);
	
	return true;
}

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

bool Recorder::tryOpenFile(const std::string& full_file)
{
	fp_ = fopen(full_file.c_str(),"w");
	if(fp_ == NULL)
	{
		ROS_ERROR("[%s] Open %s failed!",__NAME__, file.c_str());
		return false;
	}
	else
		ROS_INFO("[%s] New path file: %s created.",__NAME__, file.c_str());
	return true;
}

/*@brief 停止连续性路径记录
 *@param discard 是否放弃当前记录，放弃/保存
 */
void Recorder::stopCurveRecord(bool discard)
{
	if(curve_recorder_state_ == CurvePathRecorderState_Idle)
		return ;
		
	curve_recorder_state_ = CurvePathRecorderState_Idle;

	//此处加锁，只有记录线程退出后才能获得
	std::lock_guard<std::mutex> lock(curve_recorder_mutex_);

	if(discard)
	{
		std::string rm_cmd = std::string("rm ") + full_file_now_;
		system(rm_cmd.c_str());
	}
}

/*@brief 连续性路径记录线程
*/
bool Recorder::curvePathRecordThread()
{
	std::lock_guard<std::mutex> lock(curve_recorder_mutex_);

	curve_recorder_state_ = CurvePathRecorderState_Recording;
	while(curve_recorder_state_ == CurvePathRecorderState_Recording)
	{
		current_point_ = getPose_();

		if(calculate_dis2(current_point_,last_point_) >= sample_distance_*sample_distance_)
		{
			printf("[%s] recoding: %.3f\t%.3f\t%.3f\r\n",__NAME__, current_point.x,current_point.y,current_point.yaw);
			fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
			fflush(fp_);
			last_point_ = current_point_;
		}
		sleep_ms(50);
	}

}

void Recorder::forceQuit()
{
	stopCurveRecord(true); //退出连续路径记录器，并丢弃记录文件
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


void Recorder::sleep_ms(int ms)
{
	std::this_thread::sleep_for(std::chrono::microseconds(ms));
}

