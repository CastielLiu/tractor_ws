#ifndef FUNCTION_H_
#define FUNCTION_H_
#include<vector>
#include<iostream>
#include<cmath>
#include<float.h>
#include<assert.h>

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	
	double x;
	double y;
	
	float curvature;
	
	float maxOffset_left;
	float maxOffset_right;
}gpsMsg_t;

float calculateDis2path(const double& X_,const double& Y_,
						 const std::vector<gpsMsg_t>& path_points, 
						 const size_t& target_point_index,
						 size_t * const nearest_point_index_ptr);

bool is_gps_data_valid(gpsMsg_t& point);

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt=true);

std::pair<float, float> get_dis_yaw(gpsMsg_t &point1,gpsMsg_t &point2);

size_t findNearestPoint(const std::vector<gpsMsg_t>& path_points, const gpsMsg_t& current_point);

void pointOffset(gpsMsg_t& point,float offset);

float generateRoadwheelAngleByRadius(const float& radius, const float& wheel_base);

bool loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points);

#endif
