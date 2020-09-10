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
#include <cmath>


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
	const float getRoadWheelAngle(){return road_wheel_angle_;}
	const float getMotorSpeed() {return motor_speed_;}
	const uint8_t getErrorMsg() {return error_code_;}
	void clearErrorFlag();
	void reboot();
	
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
	
	//+线程同步相关变量
	std::condition_variable condition_variable_;
	std::mutex cv_mutex_;
	enum DataResponseType
	{
	    DataResponse_EnableStatus,
	    DataResponse_AdcValue,
	    DataResponse_MotorSpeed,
	    DataResponse_ErrorMsg,
	    
	    DataResponse_SetSpeed,
	    DataResponse_SetRotate,
	    DataResponse_SetEnable,
	    
	    DataResponse_ClearErrorFlag,
	    DataResponse_Reboot,
	    
	};
	int response_data_type_;
	//-线程同步相关变量
	
	float road_wheel_angle_;
	
	float road_wheel_angle_resolution_;
	float road_wheel_angle_offset_;
	bool is_enabled_;
	float motor_speed_;
	uint8_t error_code_;
};

#endif

