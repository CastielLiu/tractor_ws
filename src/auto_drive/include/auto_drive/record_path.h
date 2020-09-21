#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <interface/RecordPath.h>
#include "auto_drive/structs.h"

/* record path node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

class Recorder
{
public:
	Recorder();
	~Recorder();
	bool init();
	void recordToFile();
private:
	bool recordPathService(interface::RecordPath::Request  &req,
						   interface::RecordPath::Response &res);
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
	bool is_location_ok();
	std::string file_path_;
	
	FILE *fp_;
	
	gpsMsg_t last_point , current_point;
	
	float sample_distance_;
	ros::ServiceServer srv_record_path_;
	ros::Subscriber sub_utm_ ;
	std::string utm_topic_;
	int status_;
};
