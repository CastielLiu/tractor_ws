#include<base_control/base_control.h>
#include<ros/ros.h>
#include<serial/serial.h>
#include<iostream>
#include<boost/thread.hpp>
#include<boost/bind.hpp>

#ifndef PACK
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif


class BaseControl
{
  public:
	BaseControl();
	~BaseControl();
	bool init();
	void run();
  private:
	serial::Serial *serial_port_;
};


int main()
{

}
