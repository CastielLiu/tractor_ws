#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <float.h>
#include <assert.h>
#include "structs.h"


/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算主体车辆到路径的距离(横向偏差),并更新最近点索引
 *@param x,y         目标点坐标
 *@param path_points 路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param nearest_point_index 输出与目标点最近的路径点索引
 */
float calculateDis2path(const double& x,const double& y,
						 const path_t & path_points, //path
						 size_t   ref_point_index, //参考点索引
						 size_t& nearest_point_index)
{
	int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
	if(ref_point_index == 0)
	{
		ref_point_index = 1;
		searchDir = 1;
	}
	else if(ref_point_index == path_points.size()-1)
	{
		ref_point_index = path_points.size()-2;
		searchDir = -1;
	}
	else
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
					     pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
					     pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
					     pow(path_points[ref_point_index+1].y - y, 2);
		if(dis2next > dis2ref && dis2last > dis2ref) 
			searchDir = 0;
		else if(dis2next > dis2ref && dis2ref > dis2last)
			searchDir = -1;
		else
			searchDir = 1;
	}
	
	//std::cout  <<  "searchDir:"  << "\t" << searchDir << "\r\n";
	while(ref_point_index>0 && ref_point_index<path_points.size()-1)
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
							pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
							pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
							pow(path_points[ref_point_index+1].y - y, 2);
	//std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";		
		if((searchDir == 1 && dis2next > dis2ref) ||
		   (searchDir ==-1 && dis2last > dis2ref) ||
		   (searchDir == 0))
			break;

		ref_point_index += searchDir;
	}
	float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
	anchor_x = path_points[ref_point_index].x;
	anchor_y = path_points[ref_point_index].y;
	anchor_yaw = path_points[ref_point_index].yaw;

	nearest_point_index = ref_point_index;
	//float dis2anchor = sqrt((x-anchor_x)*(x-anchor_x)+(y-anchor_y)*(y-anchor_y));
	float dx = (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
	return dx;
}

/*@brief 计算目标点到达路径的距离,点在路径左侧为负,右侧为正
 *@brief 该函数主要用于计算目标到路径的距离,并考虑路径终点问题
 *@param x,y         目标点坐标
 *@param path_points 路径点集
 *@param ref_point_index 参考点索引，以此参考点展开搜索，加速计算
 *@param max_search_index 最大搜索索引,超出此索引的目标物则输出距离为FLT_MAX
 */
float calculateDis2path2(const double& x,const double& y,
						 const std::vector<gpsMsg_t>& path_points, 
						 size_t  ref_point_index, //参考点索引
						 size_t  max_search_index)
{
	int searchDir; //搜索方向 -1:向后搜索， 1：向前搜索， 0 搜索完毕
	if(ref_point_index == 0)
	{
		ref_point_index = 1;
		searchDir = 1;
	}
	else if(ref_point_index == path_points.size()-1)
	{
		ref_point_index = path_points.size()-2;
		searchDir = -1;
	}
	else
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
					     pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
					     pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
					     pow(path_points[ref_point_index+1].y - y, 2);
		if(dis2next > dis2ref && dis2last > dis2ref) 
			searchDir = 0;
		else if(dis2next > dis2ref && dis2ref > dis2last)
			searchDir = -1;
		else
			searchDir = 1;
	}
	
	//std::cout  <<  "searchDir:"  << "\t" << searchDir << "\r\n";
	while(ref_point_index>0 && ref_point_index<path_points.size()-1)
	{
		float dis2ref  = pow(path_points[ref_point_index].x   - x, 2) + 
							pow(path_points[ref_point_index].y   - y, 2);
		float dis2last = pow(path_points[ref_point_index-1].x - x, 2) + 
							pow(path_points[ref_point_index-1].y - y, 2);
		float dis2next = pow(path_points[ref_point_index+1].x - x, 2) + 
							pow(path_points[ref_point_index+1].y - y, 2);
	//std::cout  << ref_point_index << "\t" <<  sqrt(dis2last)  << "\t" << sqrt(dis2ref) << "\t" << sqrt(dis2next) << "\r\n";		
		if((searchDir == 1 && dis2next > dis2ref) ||
		   (searchDir ==-1 && dis2last > dis2ref) ||
		   (searchDir == 0))
			break;

		ref_point_index += searchDir;
	}
	float anchor_x,anchor_y, anchor_yaw; //锚点的位置和航向
	anchor_x = path_points[ref_point_index].x;
	anchor_y = path_points[ref_point_index].y;
	anchor_yaw = path_points[ref_point_index].yaw;
	
	//若参考索引大于最大搜索索引,且目标物与车辆纵向距离大于一定阈值,则输出 FLT_MAX
	if(ref_point_index >= max_search_index)
	{
		anchor_x = path_points[max_search_index].x;
		anchor_y = path_points[max_search_index].y;
		anchor_yaw = path_points[max_search_index].yaw;
		float dy = (x-anchor_x)*sin(anchor_yaw) + (y-anchor_y) * cos(anchor_yaw);
		//float dx = (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
	//printf("dx:%.2f\tdy:%.2f\tref_point_index:%d\tmax_search_index:%d\n",dx,dy,ref_point_index,max_search_index);
		
		if(dy > 0.5)
			return FLT_MAX;
	}
	
	//printf("dx:%.2f\tdy:%.2f\tref_point_index:%d\n",dx,dy,ref_point_index);
	
	return (x-anchor_x)*cos(anchor_yaw) - (y-anchor_y) * sin(anchor_yaw);
}

inline bool is_gps_data_valid(gpsMsg_t& point)
{
	if(fabs(point.x) >100 && fabs(point.y) >100)
		return true;
	return false;
}

inline float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt=true)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

inline std::pair<float, float> get_dis_yaw(const gpsMsg_t &point1, const gpsMsg_t &point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(x * x + y * y);
	dis_yaw.second = atan2(x,y);
	
	if(dis_yaw.second <0)
		dis_yaw.second += 2*M_PI;
	return dis_yaw;
}

inline int findNearestPoint(const path_t& path, const gpsMsg_t& current_point)
{
	int index = 0;
	float min_dis = FLT_MAX;
	float dis;
	
	for(size_t i=0; i<path.points.size(); )
	{
		dis = dis2Points(path.points[i],current_point,true);
		// printf("i=%d\t dis:%f\n",i,dis);
		// printf("path.points[%d] x:%lf\t y:%lf\n",i,path.points[i].x,path.points[i].y);
		// printf("current_point  x:%lf\t y:%lf\n",current_point.x,current_point.y);
		if(dis < min_dis)
		{
			min_dis = dis;
			index = i;
		}
		i += int(dis)+1;
	}
	
	if(min_dis >10.0)
	{
		index = -1;
		printf("the nearest point is so far! that's to say, the vehicle is far from the reference path!");
	}
	
	return index;
}

inline gpsMsg_t pointOffset(const gpsMsg_t& point,float offset)
{
	gpsMsg_t result = point; //copy all infos of point
	result.x =  offset * cos(point.yaw) + point.x;
	result.y = -offset * sin(point.yaw) + point.y;
	return result;
}

inline float generateRoadwheelAngleByRadius(const float& radius, const float& wheel_base)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return atan(wheel_base/radius)*180/M_PI;    //correct algorithm 
}

inline bool loadPath(const std::string& file_path, path_t& path)
{
	FILE *fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		printf("[loadPath] open %s failed\r\n",file_path.c_str());
		return false;
	}
	if(path.size() != 0)
		path.clear();
	
	gpsMsg_t point;
	while(!feof(fp))
	{
		fscanf(fp,"%lf\t%lf\t%lf\n",&point.x,&point.y,&point.yaw);
		path.points.push_back(point);
	}
	path.resolution = 0.1; 
	fclose(fp);
	if(path.size() < 2)
		return false;
	return true;
}

inline bool generatePathByVertexes(const path_t& vertex_path, path_t& path,float increment)
{
	const std::vector<gpsMsg_t>& vertexes = vertex_path.points;
	if(vertexes.size()<2)
		return false;
	gpsMsg_t startPoint = vertexes[0];
	gpsMsg_t endPoint;
	for(int i=1; i<vertexes.size(); ++i)
	{
		endPoint = vertexes[i];
		float distance = dis2Points(startPoint,endPoint,true);
		int point_cnt = distance / increment;
		
		double x_increment = (endPoint.x - startPoint.x)/point_cnt;
		double y_increment = (endPoint.y - startPoint.y)/point_cnt;
		
		for(int j=0; j<point_cnt; ++j)
		{
			gpsMsg_t now;
			now.x = startPoint.x + x_increment*j;
			now.y = startPoint.y + y_increment*j;
			path.points.push_back(now);
		}
	}
	return true;
}


#endif
