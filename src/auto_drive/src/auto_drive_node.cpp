#include"auto_drive/auto_drive.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "auto_drive_node");
	ros::AsyncSpinner spinner(4);
	spinner.start(); //非阻塞

    AutoDrive auto_drive;
    if(auto_drive.init())
    	ros::waitForShutdown();
    return 0;
}  