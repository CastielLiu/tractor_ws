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
//闲暇时间对此结构体进行修改，此为路径点信息，并非gps信息

typedef struct
{
	bool flag; // invalid ?
	float roadwheel_angle;
	float speed;
	float brake;
}controlMsg_t;

typedef struct
{
	std::vector<gpsMsg_t> points;
	float resolution; //分辨率,即路径点间的间距
	void clear()
	{
		points.clear();
	}
	size_t size()
	{
		return points.size();
	}
}path_t;

inline bool calculateDis2path(const double& X_,const double& Y_, //point corrdinate
						 const path_t & path, //path
						 const size_t& target_point_index, //the reference point index
						 size_t& nearest_point_index,
						 float& distance)                  //distance between point and path
{
	if(target_point_index == 0)
	{
		nearest_point_index = 0;
	
		//the direction of side c 
		//float yaw_of_c = (path.points[first_point_index].yaw + path.points[second_point_index].yaw)/2;
		float yaw_of_c = atan2(path.points[1].x-path.points[0].x, path.points[1].y-path.points[0].y);
	
		//object : world coordination to local coordination
		float x = (X_-path.points[0].x) * cos(yaw_of_c) - (Y_-path.points[0].y) * sin(yaw_of_c);
		//float y = (X_-path.points[first_point_index].x) * sin(yaw_of_c) + (Y_-path.points[first_point_index].y) * cos(yaw_of_c);
	    distance = x;
		return true;
	}
	
	//ROS_INFO("path.points.size:%d\t target_point_index:%d",path.points.size(),target_point_index);
	
	//this target is tracking target,
	//let the target points as the starting point of index
	//Judging whether to index downward or upward
	float dis2target = pow(path.points[target_point_index].x - X_, 2) + 
					   pow(path.points[target_point_index].y - Y_, 2) ;
	
	float dis2next_target = pow(path.points[target_point_index+1].x - X_, 2) + 
							pow(path.points[target_point_index+1].y - Y_, 2) ;
							
	float dis2last_target = pow(path.points[target_point_index-1].x - X_, 2) + 
					        pow(path.points[target_point_index-1].y - Y_, 2) ;
	
	//std::cout << sqrt(dis2target)<<"\t"<< sqrt(dis2next_target) <<"\t"<< sqrt(dis2last_target) << std::endl;
	
	float first_dis ,second_dis ;  //a^2 b^2 
	size_t first_point_index,second_point_index;
	
	first_dis = dis2target;
	first_point_index = target_point_index;
	
	int is_yawReverse = 0;
	
	if(dis2last_target <dis2target && dis2next_target > dis2target) //downward
	{
		is_yawReverse = 1;
		for(size_t i=1;true;i++)
		{
		/*   prevent size_t index 0-1 data overflow    */
			if(target_point_index >= i)
				second_point_index = target_point_index-i;
			else
			{
				first_point_index = 1;
				second_point_index = 0;
				break;
			}
		/*   prevent size_t index 0-1 data overflow    */
			
			second_dis = pow(path.points[second_point_index].x - X_, 2) + 
						 pow(path.points[second_point_index].y - Y_, 2) ;
			
			if(second_dis < first_dis) //continue 
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else if(dis2next_target < dis2target && dis2last_target > dis2target) //upward
	{
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index + i;
			if(second_point_index >= path.points.size())
			{
				// distance = 0.0;
				// nearest_point_index = 0;
				return false;
			}
			
			second_dis = pow(path.points[second_point_index].x - X_, 2) + 
						 pow(path.points[second_point_index].y - Y_, 2) ;

			if(second_dis < first_dis) //continue
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else //midile
	{
		first_point_index = target_point_index-1;
		//first_point_index = target_point_index;
		
		second_point_index = target_point_index +1;
	}
		

	nearest_point_index = (first_point_index+second_point_index)/2;
	
	//the direction of side c
	//float yaw_of_c = (path.points[first_point_index].yaw + path.points[second_point_index].yaw)/2;
	float yaw_of_c = is_yawReverse*M_PI + atan2(path.points[second_point_index].x-path.points[first_point_index].x,
									   path.points[second_point_index].y-path.points[first_point_index].y);
				
	//object : world coordination to local coordination
	float x = (X_-path.points[first_point_index].x) * cos(yaw_of_c) - (Y_-path.points[first_point_index].y) * sin(yaw_of_c);
	//float y = (X_-path.points[first_point_index].x) * sin(yaw_of_c) + (Y_-path.points[first_point_index].y) * cos(yaw_of_c);
	
	//ROS_ERROR("index1:%d\t index2:%d",first_point_index,second_point_index);

	distance = x;
	return true;
}

inline bool is_gps_data_valid(gpsMsg_t& point)
{
	if(fabs(point.x) >100 && fabs(point.y) >100)
		return true;
	return false;
}

inline float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
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

inline bool loadPath(std::string file_path, path_t& path)
{
	
	FILE *fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		printf("open %s failed",file_path.c_str());
		return false;
	}
	
	gpsMsg_t point;
	
	while(!feof(fp))
	{
		fscanf(fp,"%lf\t%lf\t%lf\n",&point.x,&point.y,&point.yaw);
		path.points.push_back(point);
	}
	path.resolution = 0.1; //应从文件载入
	fclose(fp);
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