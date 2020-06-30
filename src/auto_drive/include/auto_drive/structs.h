#ifndef STRUCTS_H_
#define STRUCTS_H_

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

typedef struct
{
	bool flag; // invalid ? validity
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
	size_t size() const
	{
		return points.size();
	}

	//下标索引重载
	const gpsMsg_t& operator[](size_t i) const
	{
		return points[i];
	}

}path_t;


#endif



