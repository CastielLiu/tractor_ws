#ifndef STEERING_MOTOR_H_
#define STEERING_MOTOR_H_
#include<stdint.h>
#include<string>
#include<boost/thread.hpp>
#include<boost/bind.hpp>
#include<serial/serial.h>

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
	void run(float rotate_angle, uint8_t rotate_speed=20);
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
	
	serial::Serial *serial_port_;
	bool is_read_serial_;
	boost::shared_ptr<boost::thread> read_serial_thread_ptr_;
	
	float road_wheel_angle_;
	uint16_t adcValue_;
	
	float road_wheel_angle_resolution_;
	float road_wheel_angle_offset_;
	bool is_enabled_;
	bool motor_speed_;
	bool errorMsg_;
};

#endif

