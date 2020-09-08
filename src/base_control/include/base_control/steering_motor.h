#ifndef STEERING_MOTOR_H_
#define STEERING_MOTOR_H_
#include<stdint.h>
#include<string>
#include<serial/serial.h>
#include<thread>
#include<memory>
#include <condition_variable>
#include <iostream>
#include <unistd.h>

class SteerMotor
{
  public:
	SteerMotor();
	~SteerMotor();
	bool init(std::string serial_port,int baud_rate);
	void stopReadSerial();
	void startReadSerial();
	void setSteeringSpeed(uint8_t speed);
	void setSteeringRotate(float cycleNum);
	void requestAdcValue(void);
	void requestEnableStatus();
	void requestMotorSpeed();
	void requestErrorMsg();
	void disable();
	void enable();
	bool is_enabled(){return is_enabled_;}
	void setRoadWheelAngle(float angle);
	const float &getRoadWheelAngle(){return road_wheel_angle_;}
	const uint16_t getAdcValue(){return adcValue_;}
	void setRoadWheelAngleResolution(float val){road_wheel_angle_resolution_ = val;}
	void setRoadWheelAngleOffset(float offset){road_wheel_angle_offset_ = offset;}
  private:
	bool configure_port(std::string port,int baud_rate);
	void readSerialPort();
	uint16_t generateModBusCRC_byTable(const uint8_t *ptr,uint8_t size);
	void BufferIncomingData(uint8_t *message, int length);
	void sendCmd(const uint8_t* buf,int len);
	void rotate(float angle, uint8_t speed=20);
	
	serial::Serial *serial_port_;
	bool is_read_serial_;
	std::shared_ptr<std::thread> read_serial_thread_ptr_;
	std::condition_variable condition_variable_;
	std::mutex cv_mutex_;
	bool use_condition_variable_;
	
	
	float road_wheel_angle_;
	uint16_t adcValue_;
	
	float road_wheel_angle_resolution_;
	float road_wheel_angle_offset_;
	bool is_enabled_;
	bool motor_speed_;
	bool errorMsg_;
};

#endif

