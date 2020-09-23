#include <ros/ros.h>
#include <unistd.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <interface/RecordPath.h>
#include "auto_drive/structs.h"
#include <functional>

/* record path node for intelligent tractor
 * author: liushuaipeng, southeast university
 * email:  castiel_liu@outlook.com
 */

class Recorder
{
public:
	typedef const gpsMsg_t (*getPoseFun_t)(void);
	Recorder();
	~Recorder();
	bool init();
	void recordToFile();
private:
	bool recordPathService(interface::RecordPath::Request  &req,
						   interface::RecordPath::Response &res);

	template <typename ClassT>
	bool startCurvePathRecord(req.path_file_name, const gpsMsg_t (classT::*fun)(void),classT* obj)
	{
		getPose_ = std::bind(fun, obj);

	}

	float calculate_dis2(gpsMsg_t & point1,gpsMsg_t& point2);
	bool is_location_ok();
	std::string file_path_;
	
	FILE *fp_;
	
	gpsMsg_t last_point , current_point;
	std::function<getPoseFun_t> getPose_;
	
	float sample_distance_;
	ros::ServiceServer srv_record_path_;
	ros::Subscriber sub_utm_ ;
	std::string utm_topic_;
	int status_;
};
