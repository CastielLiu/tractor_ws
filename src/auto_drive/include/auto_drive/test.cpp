#include<trans.h>
#include<iostream>
#include<cmath>
using namespace std;


int main()
{
	position_t src_point={2.0,5.0};
	pose_t dst_pose = {1.0,5.0,-45.0/180.0*M_PI};
	position_t dst_point = transform(src_point, dst_pose);
	cout << dst_point.x << ", " << dst_point.y << endl;
	
	rect_t rect(3,3,0,(2),(2));//45.0/180.0*M_PI
	cout << inRect(rect, position_t(3,3)) << endl;
	cout << inRect(rect, position_t(1.9,2)) << endl;
	cout << inRect(rect, position_t(2.5,2.5)) << endl;
	cout << inRect(rect, position_t(2.45,2.45)) << endl;
}

