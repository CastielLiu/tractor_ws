#include"auto_drive/avoiding.h"

Avoiding::Avoiding(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
    nh_(nh),
    nh_private_(nh_private),
	is_running_(false)
{
	danger_distance_front_ = 5.0;
}

Avoiding::~Avoiding()
{
    
}

bool Avoiding::init()
{
	if(is_running_)
	{
		ROS_ERROR("[Avoiding]: avoider is running, Need not reinit!");
		return false;
	}
    nh_private_.param<std::string>("objects_topic",objects_topic_,"/detected_bounding_boxs");
	nh_private_.param<float>("deceleration_cofficient",deceleration_cofficient_,50);
	nh_private_.param<float>("safety_distance_side",safety_distance_side_,0.4);
	nh_private_.param<float>("danger_distance_side",danger_distance_side_,0.25);
	nh_private_.param<float>("pedestrian_detection_area_side",pedestrian_detection_area_side_,safety_distance_side_+1.0);
	nh_private_.param<float>("max_deceleration",max_deceleration_,5.0); 
	nh_private_.param<float>("vehicle_width", vehicle_width_, 1.5);
	nh_private_.param<float>("vehicle_length", vehicle_length_, 3.0);
    sub_objects_msg_ = nh_.subscribe(objects_topic_,1,&Avoiding::objects_callback,this);
	is_running_ = true;
    return true;
}

void Avoiding::shutDown()
{
	sub_objects_msg_.shutdown();
	is_running_ = false;
}

void Avoiding::setPath(const path_t& path)
{
	path_mutex_.lock();
	path_ = path;
	path_mutex_.unlock();
}

float Avoiding::getAvoidOffset()
{
	avoiding_offest_mutex_.lock();
    return avoiding_offest_;
	avoiding_offest_mutex_.unlock();
}

controlMsg_t Avoiding::getControlMsg()
{
	std::lock_guard<std::mutex> lc(control_msg_mutex_);
	return control_msg_;
}

bool Avoiding::update(float speed, float road_wheelangle, size_t current_point_index)
{
	if(!is_running_)
		return false;
    road_wheelangle_ = road_wheelangle;
    vehicle_speed_ = speed;
	current_point_index_ = current_point_index;
    current_point_ = path_.points[current_point_index_];
	return true;
}

void Avoiding::objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msgs)
{
	size_t n_object = msgs->boxes.size();
	
	if(n_object==0)
	{
		avoiding_offest_ = 0.0;
		return;
	}
	std::vector<object_t> objects(n_object);
	for(int i=0; i<n_object; ++i)
	{
		const auto &box = msgs->boxes[i];
		const auto &position = box.pose.position;
		const geometry_msgs::Quaternion  &orientation = box.pose.orientation;
		const auto &dimensions = box.dimensions;
		tf::Quaternion quat;
		double roll,yaw,pitch;

		tf::quaternionMsgToTF(orientation, quat);//ros四元数 -> tf四元数
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//tf四元数转欧拉角
		objects[i].rect = rect_t(-position.y, position.x, yaw, dimensions.y, dimensions.x);
		objects[i].rect.inflat(vehicle_width_); //障碍物膨胀
	}
	float forsight_distance = 10.0; //避障前视距离
	size_t start_index = current_point_index_;
	size_t end_index = current_point_index_ + forsight_distance/path_.resolution;
	if(end_index >= path_.points.size())
	{
		ROS_INFO("[Avoiding]: the remaind path points is not enough!")
		this->shutDown();
		return ;
	}
	//计算路径片段在车体坐标系下的坐标  ++++++++++++++++++++++++++++++++++++++++++++++
	size_t points_cnt = end_index-start_index+1;
	std::vector<position_t> local_path_points(points_cnt);
	pose_t vehicle_pose = path_.points[current_point_index_];
	for(int i=0; i<points_cnt; ++i)
	{
		position_t point_in_earth = path_.points[i];
		local_path_points[i] = transform(point_in_earth, vehicle_pose);
	}

	//判断各目标是否为障碍物
	for(int i=0; i<n_object; ++i)
	{
		const pose_t& object_pose = objects[i].rect.pose;
		objects[i].is_obstacle = false;
		for(int j=0; j<points_cnt; ++j)
		{
			position_t point_in_object = transform(local_path_points[j], object_pose);

			if(!inRect(objects[i].rect, point_in_object))
				continue;
			//路径点在目标矩形范围内
			float distance = dis2Points(path_.points[j], path_.points[current_point_index_]);
			if(distance <= danger_distance_front_) 
			{
				std::lock_guard<std::mutex> lc(control_msg_mutex_);
				control_msg_.flag = true;  //所有目标计算完毕后，无特殊情况时，flag复位
				control_msg_.brake = 1.0;
				return ;
			}
			objects[i].is_obstacle = true;
			break;
		}
	}
	//
	for(auto& object:objects)
	{
		if(object.inners.size()==0)
		{
			object.is_obstacle = false;
			continue;
		}

	}



	    
}


void Avoiding::bubbleSort(const float * distance, size_t * index, size_t length)
{
    for(int index=length-1; index>0; index--)
    {
        for(int j=0; j<index; ++j)
        {
            if (distance[j] > distance[j+1])
			{
				size_t ind_temp = index[j];
				index[j] = index[j + 1];
				index[j + 1] = ind_temp;
			}
        }
    }
}
