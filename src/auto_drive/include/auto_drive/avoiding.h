#ifndef AVOIDING_H_
#define AVOIDING_H_
#include<ros/ros.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<iostream>
#include<vector>
#include<assert.h>

class Avoiding
{
  public:
    Avoiding(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~Avoiding();
    bool init();
    float getAvoidOffset();
    void objects_callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects);
    void get_obstacle_msg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects,
                          size_t objectIndex,
                          whatArea_t *obstacleArea,
                          float ** obstacleVertex_x_y,
                          float *obstacleDistance, 
                          size_t *obstacleIndex,
                          size_t &obstacleSequence);
  private:
    ros::NodeHandle nh_, nh_private_;

    enum objectType_t
    {
      Unknown = 0,
      Person = 1,
      Vehicle = 2
    };

    float max_deceleration_;

    std::string objects_topic_;

    float safety_distance_front_;
    float safety_distance_side_ ;

    float danger_distance_front_;
    float danger_distance_side_;

    float pedestrian_detection_area_side_; //行人检测区两侧距离

    float vehicle_speed_; //m/s
    float deceleration_cofficient_;

    std::vector<gpsMsg_t> path_points_;

    size_t target_point_index_;
    size_t nearest_point_index_;

    gpsMsg_t current_point_,target_point_;

    float avoiding_offest_;

    float maxOffset_right_, maxOffset_left_;

    bool is_systemOk_;
};
#endif