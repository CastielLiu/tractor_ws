#ifndef AVOIDING_H_
#define AVOIDING_H_
#include<ros/ros.h>

class Avoiding
{
  public:
    Avoiding();
    ~Avoiding();
  private:
    ros::NodeHandle nh_, nh_private_;

};
#endif