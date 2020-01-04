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

enum objectType_t
{
  Unknown = 0,
  Person = 1,
  Vehicle = 2
};

typedef struct Object
{
    rect_t rect; //目标矩形参数信息
    std::vector<pose_t> inners; //在此目标内部的路径点位姿(相对object坐标系)
    bool is_obstacle;
    float obstacle_distance;
    float left_offset; //最左侧顶点到路的距离 <=0
    float right_offset; //最右侧顶点到路的距离 >=0

    Object(){}
    //目标复制时，仅复制关键信息
    Object(const Object& obj)
    {
      rect = obj.rect;
      is_obstacle = obj.is_obstacle;
      left_offset = obj.left_offset;
      right_offset = obj.right_offset;
      obstacle_distance = obj.obstacle_distance;
    }
    Object operator=(const Object& obj)
    {
      return Object(obj);
    }
    
    void calOffset() //right_offset和left_offset
    {
      size_t size = inners.size();
      if(size == 0)
        return;

      //step1: 求内点直线(在rect内部的路径点所构成的直线)
      pose_t line = inners[0]; //只有一个有向点时
      if(size > 1) //多个有向点时，方向取首尾平均
        line.yaw = (inners[0].yaw + inners[size-1].yaw)/2;

      //step2: 求直线最两侧的矩形顶点到直线的距离
      if(rect.yaw<=0) //矩形为顺时针旋转，最左/右侧顶点分别位于三、一象限
      {
        left_offset = distance(rect.vertexes[2], line);
        right_offset = distance(rect.vertexes[0], line);
      }
      else //矩形为逆时针旋转，最左/右侧顶点分别位于二、四象限
      {
        left_offset = distance(rect.vertexes[1], line);
        right_offset = distance(rect.vertexes[3], line);
      }
    }
    
}object_t;

class Avoiding
{
  public:
    Avoiding(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~Avoiding();
    bool init();
    void shutDown();
    void setPath(const path_t& path);
    float getAvoidOffset();
    controlMsg_t getControlMsg();
    bool update(float speed, float road_wheelangle, const gpsMsg_t& current_point,
			          size_t nearest_point_index);
    void objMsgs2obj(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msgs,
						         const std::vector<object_t>& msgs)；
    void objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects);
    float tryOffset(std::vector<object_t>& objects, 
                    const std::vector<object_t*>& sorted_obstacles,
                    const std::vector<pose_t>& origin_path_points,
                    float old_offset,
                    int dir);
    inline void offsetPathPoints(std::vector<pose_t>& points, float offset);
    void objectClassify(std::vector<object_t>& objects, std::vector<pose_t>& now_path_points,
							          std::vector<object_t>& obstacles);
    void bubbleSort(std::vector<object_t *>& obstacles, size_t length);
    std::vector<object_t*> sortObstacle(std::vector<object_t>& obstacles);

  private:
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber sub_objects_msg_;
    bool is_running_;

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