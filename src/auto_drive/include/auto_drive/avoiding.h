#ifndef AVOIDING_H_
#define AVOIDING_H_
#include<ros/ros.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include "tf/transform_datatypes.h"
#include"auto_drive/trans.h"
#include"auto_drive/function.h"
#include<iostream>
#include<vector>
#include<thread>
#include<mutex>
#include<assert.h>

typedef struct Object
{
    rect_t rect; //目标矩形参数信息
    std::vector<position_t> inners; //在此目标内部的路径点局部坐标(相对object坐标系)
    bool is_obstacle;
    float left_offset; //最左侧顶点到路的距离
    float right_offset; //最右侧顶点到路的距离
}object_t;

class Avoiding
{
  public:
    Avoiding(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~Avoiding();
    bool init();
    void shutDown();
    void setPath(const path_t& path);
    bool update(float speed, float road_wheelangle,  //vehicle state
			          size_t current_point_index);      //vehicle position
    float getAvoidOffset();
    controlMsg_t getControlMsg();
    void objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects);
   
    bool is_dangerous(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
					const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object);
  private:
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber sub_objects_msg_;
    bool is_running_;

    enum objectType_t
    {
      Unknown = 0,
      Person = 1,
      Vehicle = 2
    };

//vehicle params
    float max_deceleration_;
    float vehicle_speed_; //m/s
    float road_wheelangle_;
    float deceleration_cofficient_;
    float vehicle_width_;
    float vehicle_length_;

    std::string objects_topic_;

    float safety_distance_front_;
    float safety_distance_side_ ;

    float danger_distance_front_;
    float danger_distance_side_;

    float pedestrian_detection_area_side_; //行人检测区两侧距离

    path_t path_;
    std::mutex path_mutex_;

    size_t target_point_index_;
    size_t nearest_point_index_;

    gpsMsg_t current_point_,target_point_;

    float maxOffset_right_, maxOffset_left_;

    float avoiding_offest_;
    std::mutex avoiding_offest_mutex_;

    controlMsg_t control_msg_;
    std::mutex control_msg_mutex_;
};
#endif