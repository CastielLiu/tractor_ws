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

//整数整合路径规划
//
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

/** 
 * @brief 停止订阅障碍物信息，避障节点等待唤醒
 */
void Avoiding::shutDown()
{
	sub_objects_msg_.shutdown();
	is_running_ = false;
}

/** 
 * @brief 载入当前路径
 */
void Avoiding::setPath(const path_t& path)
{
	path_mutex_.lock();
	path_ = path;
	path_mutex_.unlock();
}

/** 
 * @brief 获取避障期望偏移
 		  应先获取避障控制消息，控制消息flag为false时，偏移值有效，
		  否则，应按照避障控制消息进行车辆控制
 * @return 偏移值
 */
float Avoiding::getAvoidOffset()
{
	avoiding_offest_mutex_.lock();
    return avoiding_offest_;
	avoiding_offest_mutex_.unlock();
}

/** 
 * @brief 获取避障控制消息
 * @return 控制消息
 */
controlMsg_t Avoiding::getControlMsg()
{
	std::lock_guard<std::mutex> lc(control_msg_mutex_);
	return control_msg_;
}

/** 
 * @brief 更新车辆状态信息
 * @param speed                 车速
 * @param road_wheelangle       前轮转角
 * @param vehicle_pose    		车辆位置
 * @param nearest_point_index 	距离路径最近点的索引
 * 
 * @return 是否更新成功
 */
bool Avoiding::update(float speed, float road_wheelangle, const pose_t& vehicle_pose, size_t nearest_point_index)
{
	if(!is_running_)
		return false;
    road_wheelangle_ = road_wheelangle;
    vehicle_speed_ = speed;
	nearest_point_index_ = nearest_point_index;
    vehicle_pose_ = vehicle_pose;
	return true;
}

/** 
 * @brief ros消息(BoundingBoxArray)转换为自定义object集合
 * @param msgs            输入：ros massages
 * @param objects   	  输出：自定义object集合
 */
void Avoiding::objMsgs2obj(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msgs,
						   std::vector<object_t>& objects)
{
	size_t n_object = msgs->boxes.size();
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
		objects[i].rect.calVertexes(); //计算各顶点坐标
	}
}

void Avoiding::objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msgs)
{
	size_t n_object = msgs->boxes.size();
	if(n_object==0)
	{
		avoiding_offest_ = 0.0;
		std::lock_guard<std::mutex> lc(control_msg_mutex_);
		control_msg_.flag = false;
		return;
	}
	std::vector<object_t> objects(n_object);
	objMsgs2obj(msgs, objects);

	float forsight_distance = 10.0; //避障前视距离
	size_t start_index = nearest_point_index_;
	size_t end_index = nearest_point_index_ + forsight_distance/path_.resolution;
	if(end_index >= path_.points.size())
	{
		ROS_INFO("[Avoiding]: the remaind path points is not enough!");
		this->shutDown();
		return ;
	}

	size_t points_cnt = end_index-start_index+1;
	
	std::vector<pose_t> raw_path_points(points_cnt);//原始路径片段在车体坐标系下的坐标 
	std::vector<pose_t> now_path_points(points_cnt);//当前路径片段在车体坐标系下的坐标

	for(int i=0; i<points_cnt; ++i)
	{
		pose_t pose_in_earth = path_.points[i];
		now_path_points[i] = raw_path_points[i] = transform(pose_in_earth, vehicle_pose_);
		if(avoiding_offest_)
			now_path_points[i].offsetInPlace(avoiding_offest_); 
	}

	std::vector<object_t> obstacles;
	objectClassify(objects, now_path_points, obstacles);
	
	if(obstacles.size() ==0)
	{
		std::lock_guard<std::mutex> lock1(control_msg_mutex_);
		control_msg_.flag = false;
		if(avoiding_offest_==0.0)
			return;
		//当前位于避障路径，且没有障碍物，尝试返回原路径
		bool ok = tryToReturnOriginPath(objects, raw_path_points); 
		if(!ok)
			return;
		std::lock_guard<std::mutex> lock2(avoiding_offest_mutex_);
		avoiding_offest_ = 0.0;
		return ;
	} 
	std::vector<object_t*> sorted_obstacles = sortObstacle(obstacles);

	if(sorted_obstacles[0]->obstacle_distance <= danger_distance_front_) 
	{
		//emergency brake
		std::lock_guard<std::mutex> lc(control_msg_mutex_);
		control_msg_.flag = true; 
		control_msg_.brake = 1.0;
		return ;
	}

	float offset_left = tryOffset(objects, sorted_obstacles, raw_path_points, avoiding_offest_, -1);
	float offset_right= tryOffset(objects, sorted_obstacles, raw_path_points, avoiding_offest_,  1);

	if(offset_left == offset_right ) //all 0
	{
		//无法避障，根据距离最近障碍物的距离进行减速
		float nearest_obstacle_dis = sorted_obstacles[0]->obstacle_distance;
		std::lock_guard<std::mutex> lc(control_msg_mutex_);
		control_msg_.flag = true;
		control_msg_.brake = 0.5;
	}
	else if(-offset_left > offset_right)
		avoiding_offest_ += offset_right;
	else 
		avoiding_offest_ += offset_left;
	   
}

/** 
 * @brief 尝试返回原路径 (当前处于避障路径，且当前没有障碍物)
 * @param objects            所有目标
 * @param origin_path_points 原始路径点
 * 
 * @return 是否可以返回原路径
 */
bool Avoiding::tryToReturnOriginPath(std::vector<object_t>& objects,
									 const std::vector<pose_t>& origin_path_points)
{
	std::vector<object_t> obstacles;
	objectClassify(objects, origin_path_points, obstacles);
	if(obstacles.size()==0)
		return true;
	return false;
}

/** 
 * @brief 尝试路径偏移，并返回相对偏移值
 * @param objects            所有目标
 * @param sorted_obstacles   有序障碍物
 * @param origin_path_points 原始路径点
 * @param old_offset 		 上一时刻的偏移值
 * @param dir 		 		 尝试偏移方向，-1左侧，1右侧
 * @return 再偏移值,返回值等于0时，表明不可偏移
 */
float Avoiding::tryOffset(std::vector<object_t>& objects, 
					      const std::vector<object_t*>& sorted_obstacles,
						  const std::vector<pose_t>& origin_path_points,
						  float old_offset,
						  int dir)
{
	float try_offset = 0.0; //基于上一偏移基础上的再偏移值
	for(const auto& obstacle:sorted_obstacles)
	{
		if(dir == -1)
		{
			if(obstacle->left_offset < try_offset)
				try_offset = obstacle->left_offset;
		}
		else
		{
			if(obstacle->right_offset > try_offset)
				try_offset = obstacle->right_offset;
		}

		if(try_offset+old_offset < maxOffset_left_ ||
		   try_offset+old_offset > maxOffset_right_)
			return 0.0;
	}

	std::vector<pose_t> now_path_points = origin_path_points;
	offsetPathPoints(now_path_points, try_offset+old_offset);
	std::vector<object_t> now_obstacles;
	objectClassify(objects, now_path_points, now_obstacles);
	if(now_obstacles.size() ==0 )
		return try_offset;
	std::vector<object_t*> now_sorted_obstacles = sortObstacle(now_obstacles);
	float offset = tryOffset(objects, now_sorted_obstacles, origin_path_points, try_offset+old_offset, dir);
	try_offset += offset;
	if(offset == 0.0)
		return 0.0;
	return try_offset;
}

/** 
 * @brief 路径偏移(就地处理)
 * @param points   路径点集
 * @param offset   偏移量 (左负右正)
 *  
 * @return
 */
inline void Avoiding::offsetPathPoints(std::vector<pose_t>& points, float offset)
{
	for(auto& point:points)
		point.offsetInPlace(offset);
}

/** 
 * @brief 目标分类(障碍物、非障碍物)
 * @param objects           所有目标
 * @param now_path_points   当前路径点集(偏移后) 
 * @param obstacles         输出障碍物
 *  
 * @return
 */
void Avoiding::objectClassify(std::vector<object_t>& objects, const std::vector<pose_t>& now_path_points,
							  std::vector<object_t>& obstacles)
{
	obstacles.clear();
	size_t n_object = objects.size();
	size_t n_path_points = now_path_points.size();
	//判断各目标是否为障碍物
	for(object_t& object:objects)
	{
		//在此之前已经被认定为"可能"的障碍物，跳过
		//所谓“可能”，即并非阻碍了当前路径，可能阻碍之前提出的路径或者原路径
		if(object.is_obstacle)
			continue;
		// object.inners.clear();
		const pose_t& object_pose = object.rect.pose;
		for(int j=0; j<n_path_points; ++j)
		{
			pose_t local_pose = transform(now_path_points[j], object_pose);
			position_t local_position = local_pose;
			if(!inRect(object.rect, local_position))
				continue;
			object.inners.push_back(now_path_points[j]);
		}
		if(object.inners.size()!=0)
		{
			object.is_obstacle = true;
			object.obstacle_distance = object.inners[0].y;
			object.calOffset();
			obstacles.push_back(object);
		}
	}
}

/** 
 * @brief 障碍物由近及远排序(就地排序)
 * @param obstacles   障碍物vector
 * @param length   	  障碍物个数
 *  
 */
void Avoiding::bubbleSort(std::vector<object_t *>& obstacles, size_t length)
{
    for(int index=length-1; index>0; index--)
    {
        for(int j=0; j<index; ++j)
        {
            if (obstacles[j]->obstacle_distance > obstacles[j+1]->obstacle_distance)
			{
				auto temp = obstacles[j];
				obstacles[j] = obstacles[j + 1];
				obstacles[j + 1] = temp;
			}
        }
    }
}

/** 
 * @brief 障碍物由近及远排序
 * @param obstacles   障碍物vector
 *  
 * @return 返回有序障碍物指针vector
 */
std::vector<object_t*> Avoiding::sortObstacle(std::vector<object_t>& obstacles)
{
	size_t n_obstacle = obstacles.size();
	std::vector<object_t*> sorted_obstacles(n_obstacle);
	for(size_t i=0; i<n_obstacle; ++i)
		sorted_obstacles[i] = &obstacles[i];
	bubbleSort(sorted_obstacles, sorted_obstacles.size());
}
