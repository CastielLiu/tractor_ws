#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <interface/RecordPath.h>
#include "auto_drive/function.h"

#define __NAME__ "record_path"

#ifndef PI_
#define PI_ 3.141592653589
#endif

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
	std::string utm_topic = private_nh.param<std::string>("utm_topic","");
	
	if(utm_topic.empty())
	{
		ROS_ERROR("[%s] Please input utm_topic in launch file!", __NAME__);
		return false;
	}
	
	if(file_path_.empty())
	{
		ROS_ERROR("[%s] Please input record file path in launch file!", __NAME__);
		return false;
	}
	
	//创建服务,用于外部触发记录路径
	srv_record_path_ = nh.advertiseService("record_path_service",&Recorder::recordPathService,this);
	
	sub_utm_ = nh.subscribe(utm_topic, 1, &Recorder::odom_callback,this);
	
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
	//请求开始记录
	if(req.command_type == req.START_RECORD_PATH )
	{
		//已经处于记录状态,直接返回请求成功,不能作为重复请求或错误处理
		//客户端请求记录后,服务器回传请求成功指令,然后客户端跳转到开始记录界面
		//若长时间没有收到服务器回应,客户端超时返回
		//考虑传输错误等原因,服务器已经接收请求并回应,但客户端未成功接收的问题
		//待客户端再次请求记录时，返回正确标志,以使得客户端页面正常跳转.
		if((req.path_type ==req.CURVE_TYPE && this->status_ == CurveRecording) ||
			(req.path_type ==req.VERTEX_TYPE && this->status_ == VertexRecording))
		{
			res.success = Success;
			return true;
		}
		
		//当前处于记录状态,但新请求记录方法与当前状态不符
		//关闭正在记录的文件,并将状态复位
		if(this->status_ == CurveRecording || this->status_ == VertexRecording)
		{
			fclose(fp_);
			fp_ = NULL;
			this->status_ = RecorderIdle;
		}
		
		if(req.path_type ==req.CURVE_TYPE)
		{
			ROS_INFO("[%s] Request start record path: curve type.",__NAME__);
			last_point = {0.0,0.0,0.0,0.0,0.0}; 
			this->status_ = CurveRecording;
		}
		else if(req.path_type == req.VERTEX_TYPE)
		{
			ROS_INFO("[%s] Request start record path: vertex type.",__NAME__);
			last_point = {0.0,0.0,0.0,0.0,0.0}; 
			this->status_ = VertexRecording;
		}
		else
		{
			ROS_ERROR("[%s] Expected path type error !",__NAME__);
			res.success = Fail;
			return true;
		}
		
		std::string file = file_path_ + req.path_file_name;
		
		fp_ = fopen(file.c_str(),"w");
		if(fp_ == NULL)
		{
			ROS_ERROR("[%s] Open %s failed!",__NAME__, file.c_str());
			res.success = Fail;
			return true;
		}
		else
			ROS_INFO("[%s] New path file: %s created.",__NAME__, file.c_str());
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
		ROS_INFO("[%s] record current point: %.3f\t%.3f\t%.3f ok.",current_point.x,current_point.y,current_point.yaw*180.0/PI_);
		fprintf(fp_,"%.3f\t%.3f\t%.3f\r\n",current_point.x,current_point.y,current_point.yaw);
		fflush(fp_);
		last_point = current_point;
	}
	//请求停止记录
	else if(req.command_type == req.STOP_RECORD_PATH)
	{
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
	//自节点运行开始更新当前点信息
	current_point.x = msg->pose.pose.position.x;
	current_point.y = msg->pose.pose.position.y;
	current_point.yaw = msg->pose.covariance[0];
	
	//如果为处于曲线记录状态, 直接返回
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


int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_path_node");
	
	Recorder recorder;
	
	if(!recorder.init())
		return 0;

	ros::spin();
	
	return 0;
}


