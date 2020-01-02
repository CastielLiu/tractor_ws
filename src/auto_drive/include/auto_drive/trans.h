#ifndef TRANS_H_
#define TRANS_H_
#include<cmath>
#include<iostream>

/** 
 * @brief 位置信息
 */
typedef struct Position
{
    float x,y;

    Position(){}
    Position(float _x,float _y)
    {
        x = _x;
        y = _y;
    }
    void print(const std::string& info = "position")
    {
        std::cout << info << ": (" << x << "," << y << ")" << std::endl;
    }
    Position(const gpsMsg_t& point)
    {
        x = point.x;
        y = point.y;
    }
    
}position_t;

/** 
 * @brief 位姿信息
 */
typedef struct Pose
{
    float x,y,yaw;
    Pose(){}
    Pose(float _x,float _y,float _yaw)
    {
        x = _x;
        y = _y;
        yaw = _yaw;
    }
    Pose(const gpsMsg_t& point)
    {
        x = point.x;
        y = point.y;
        yaw = point.yaw;
    }
    void print(const std::string& info = "pose")
    {
        std::cout << info << ": (" << x << "," << y << "," << yaw <<")" << std::endl;
    }
    
}pose_t;

/** 
 * @brief 矩形框位姿以及尺寸
 */
typedef struct Rect
{
    pose_t pose;
    float x_len, y_len;

    Rect(){}
    Rect(float _x,float _y, float _yaw, float _x_len, float _y_len)
    {
        pose.x = _x;
        pose.y = _y;
        pose.yaw = _yaw;
        x_len = _x_len;
        y_len = _y_len;
    }
    Rect(pose_t& _pose, float _x_len, float _y_len)
    {
        pose = _pose;
        x_len = _x_len;
        y_len = _y_len;
    }
    void inflat(float c)
    {
        x_len += 2*c;
        y_len += 2*c;
    }
   
    
}rect_t;

/** 
 * @brief 全局坐标转换到局部坐标
 * @param global_position   目标点的全局坐标
 * @param local_pose        局部坐标在全局坐标下的位姿
 *
 * @return 目标点的局部坐标
 */
inline position_t transform(const position_t& global_position, const pose_t& local_pose)
{
    position_t dst_position;
    dst_position.x = (global_position.x - local_pose.x) * cos(local_pose.yaw)
                    -(global_position.y - local_pose.y) * sin(local_pose.yaw);
    dst_position.y = (global_position.x - local_pose.x) * sin(local_pose.yaw)
                    +(global_position.y - local_pose.y) * cos(local_pose.yaw);
    return dst_position;
}

/** 
 * @brief 判断点是否在矩形内
 * @param rect   矩形
 * @param point  点
 *
 * @return 判断结果
 */
inline bool inRect(const rect_t& rect, const position_t& point)
{
    position_t dst_point = transform(point, rect.pose);//将point转换到rect的坐标系下
    // dst_point.print();
    if(dst_point.x <= rect.x_len/2 && dst_point.x >= -rect.x_len/2 &&
       dst_point.y <= rect.y_len/2 && dst_point.y >= -rect.y_len/2)
       return true;
    return false;
}


#endif
