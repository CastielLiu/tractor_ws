#include"auto_drive/avoiding.h"

Avoiding::Avoiding(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
    nh_(nh),
    nh_private_(nh_private)
{
}

Avoiding::~Avoiding()
{
    
}

bool Avoiding::init()
{
    nh_private.param<std::string>("objects_topic",objects_topic_,"/detected_bounding_boxs");
	nh_private.param<float>("deceleration_cofficient",deceleration_cofficient_,50);
	nh_private.param<float>("safety_distance_side",safety_distance_side_,0.4);
	nh_private.param<float>("danger_distance_side",danger_distance_side_,0.25);
	nh_private.param<float>("pedestrian_detection_area_side",pedestrian_detection_area_side_,safety_distance_side_+1.0);
	nh_private.param<float>("max_deceleration",max_deceleration_,5.0); 
    sub_objects_msg_ = nh.subscribe(objects_topic_,1,&Avoiding::objects_callback,this);
    return true;
}

float Avoiding::getAvoidOffset()
{
    return avoiding_offest_;
}

bool Avoiding::update(float speed, float road_wheelangle, const gpsMsg_t& current_point)
{
    road_wheelangle_ = road_wheelangle;
    vehicle_speed_ = speed;
    current_point_ = current_point;
}

void Avoiding::objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects)
{
	size_t n_object = objects->boxes.size();
	
	if(n_object==0)
	{
		avoiding_offest_ = 0.0)
		return;
	}
	
	size_t *indexArray = new size_t[n_object];
	float * dis2vehicleArray = new float[n_object];
	float * dis2pathArray = new float[n_object];
	
	//object position in vehicle coordination (x,y)
	//object position in  world  coordination (X,Y)
	float x,y;  double X,Y;
	
	for(size_t i=0; i< n_object; i++)
	{
		const jsk_recognition_msgs::BoundingBox& object =  objects->boxes[i];
		
		x = - object.pose.position.y;
		y =   object.pose.position.x;
		
		X =  x * cos(current_point_.yaw) + y * sin(current_point_.yaw) + current_point_.x;
		Y = -x * sin(current_point_.yaw) + y * cos(current_point_.yaw) + current_point_.y;
		
		indexArray[i] = i;
		dis2vehicleArray[i] = sqrt(x * x + y * y);
        size_t nearest_point_index;
        bool ok = calculateDis2path(X, Y, path_points_, target_point_index_, //input
					                nearest_point_index, dis2pathArray[i]); //output
	 
	    if(!ok)
        {
            ROS_INFO("[Avoiding]: calculateDis2path failed");
            return ;
        }
		//printf("target x:%f\ty:%f\t X:%f\tY:%f\t\n",x,y,X,Y);
		//printf("car X:%f\t Y:%f\t yaw:%f\n",current_point_.x,current_point_.y,current_point_.yaw);
		//ROS_INFO("dis2path:%f\t dis2vehicle:%f\t x:%f  y:%f",dis2pathArray[i],dis2vehicleArray[i],x,y);
	}
	bubbleSort(dis2vehicleArray,indexArray,n_object);

	if(is_dangerous(objects, dis2vehicleArray, indexArray, dis2pathArray, n_object))
	{
		this->emergencyBrake();//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		return ;
	}
	
	if(offset_msg_.data!=0.0 && is_backToOriginalLane(objects,dis2vehicleArray,indexArray,dis2pathArray,n_object))
	{
		backToOriginalLane();
	}
	
	if(path_points_[nearest_point_index_].traffic_sign == TrafficSign_Ambulance)
	{
		bool is_ambulance = false;
		bool is_changLaneSafety = true;
		jsk_recognition_msgs::BoundingBox object; 
		for(size_t i=0; i<n_object; i++)
		{
			object = objects->boxes[indexArray[i]];
			float dis2path = dis2pathArray[indexArray[i]];
			float safety_center_distance_x = g_vehicle_width/2 + object.dimensions.y/2 + safety_distance_side_;
			if(object.pose.position.x < -2.0 && dis2path < 1.0) //ambulance
			{
				is_ambulance = true;
			}
			else if(object.pose.position.x <20.0 && dis2path >0.0 &&
			       dis2path-3.0 < safety_center_distance_x)
			{
				is_changLaneSafety  = false; 
				break;
			}
		}
		if(is_ambulance && is_changLaneSafety)
		{
			offset_msg_.data = 2.5;
			pub_avoid_msg_to_gps_.publish(offset_msg_);
		}
	}
	else if(path_points_[nearest_point_index_].traffic_sign != TrafficSign_PickUp &&
		   path_points_[nearest_point_index_].traffic_sign != TrafficSign_TempStop &&
		   path_points_[nearest_point_index_].traffic_sign != TrafficSign_UTurn &&
		   path_points_[nearest_point_index_].traffic_sign != TrafficSign_Avoid)
	{
		//dis2vehicleArray was sorted but dis2pathArray not!
		decision(objects, dis2vehicleArray,indexArray, dis2pathArray,n_object);
	}
	
	delete [] indexArray;
	delete [] dis2vehicleArray;
	delete [] dis2pathArray;
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
