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
#include <chrono>
#include <assert.h>

class SteerMotor
{
  public:
	SteerMotor();
	~SteerMotor();
	bool init(const std::string& serial_port, int baud_rate);
	bool reInit(const std::string& serial_port, int baud_rate);
	void stop();
	bool selfCheck();
	void startRequestState(int duration);
	void stopRequestState();
	void disable();
	void enable();
	bool is_enabled(){return is_enabled_;}
	void setRoadWheelAngle(float angle);
	float getRoadWheelAngle() const;
	float getMotorSpeed() const;
	uint8_t getErrorMsg() const;
	void clearErrorFlag();
	void reboot();
	
	void setRoadWheelAngleResolution(float val){road_wheel_angle_resolution_ = val;}
	void setRoadWheelAngleOffset(float offset){road_wheel_angle_offset_ = offset;}
  private:
  	void stopReadSerial();
	void startReadSerial();
	bool configure_port(std::string port,int baud_rate);
	void readSerialThread();
	void requestStateThread(int duration);
	void requestAdcValue(void);
	void requestEnableStatus();
	void requestMotorSpeed();
	void requestErrorMsg();
	void setSteeringSpeed(uint8_t speed);
	void setSteeringRotate(float cycleNum);
	void sleep_ms(int ms);

	uint16_t generateModBusCRC_byTable(const uint8_t *ptr,uint8_t size);
	void BufferIncomingData(uint8_t *message, int length);
	void sendCmd(const uint8_t* buf,int len);
	void rotate(float angle, uint8_t speed=20);
	
	serial::Serial *serial_port_;
	bool is_read_serial_;
	std::shared_ptr<std::thread> read_serial_thread_ptr_;

	bool is_request_state_;
	std::shared_ptr<std::thread> request_state_thread_ptr_;
	
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

	std::mutex send_cmd_mutex_; //指令发送互斥锁

	//读串口和请求状态线程互斥锁，用于确认线程退出
	std::mutex read_serial_thread_mutex_;
	std::mutex request_state_thread_mutex_;
	
	float road_wheel_angle_;
	
	float road_wheel_angle_resolution_;
	float road_wheel_angle_offset_;
	bool is_enabled_;
	float motor_speed_;
	uint8_t error_code_;
	uint64_t last_active_time_ms_; //上次活跃(收到串口正确应答)时间

	enum ErrorCode
	{
		Error_None = 0,
		Error_OverCurrent = 1,
		Error_OverVoltage = 2,
		Error_UnderVoltage= 3,
		Error_OverLoad    = 0x0E,
		Error_LockedRotor = 0x11,
		Error_OverSpeed   = 0x14,
		Error_PowerBan    = 0x1A,
	};
	
	//串口有消息反馈则为在线，长时间没有收到消息则为离线
	bool motor_offline_;
};

#endif

