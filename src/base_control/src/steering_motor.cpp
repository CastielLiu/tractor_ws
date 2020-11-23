#include"base_control/steering_motor.h"


static const uint16_t CRC16Table[]={
0x0,    0xc0c1, 0xc181, 0x140,  0xc301, 0x3c0,  0x280,  0xc241, 0xc601, 0x6c0,  0x780,  0xc741, 0x500,  0xc5c1, 0xc481,
0x440,  0xcc01, 0xcc0,  0xd80,  0xcd41, 0xf00,  0xcfc1, 0xce81, 0xe40,  0xa00,  0xcac1, 0xcb81, 0xb40,  0xc901, 0x9c0,
0x880,  0xc841, 0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01,
0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641, 0xd201, 0x12c0, 0x1380, 0xd341,
0x1100, 0xd1c1, 0xd081, 0x1040, 0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240, 0x3600, 0xf6c1, 0xf781,
0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0,
0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01,
0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281,
0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441, 0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0,
0x6e80, 0xae41, 0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01,
0x7bc0, 0x7a80, 0xba41, 0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541,
0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041, 0x5000, 0x90c1, 0x9181,
0x5140, 0x9301, 0x53c0, 0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440, 0x9c01, 0x5cc0,
0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40, 0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841, 0x8801,
0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081,
0x4040};


//是否使用线程同步机制
//不使用线程同步时，通过返回数据长度进行判断
#define USE_THREAD_SYNCHRONIZE 1


#define MAX_LOAD_SIZE 100
static uint8_t raw_buffer[MAX_LOAD_SIZE];

SteerMotor::SteerMotor():
	max_speed_(40),
	min_speed_(5),
	max_road_wheel_angle_(40)
{
	serial_port_ = NULL;
	is_read_serial_ = false;
	is_request_state_ = false;
	road_wheel_angle_ = 0.0;
	road_wheel_angle_resolution_ = 180.0/4096;
	road_wheel_angle_offset_ = 0.0; //前轮转角偏移值
	is_enabled_ = false;
	error_code_ = 0x00;
	motor_offline_ = true; //初始时默认离线，
	last_active_time_ms_ = 0;
}

SteerMotor::~SteerMotor()
{
	this->disable();
	if(serial_port_ != NULL)
	{
		serial_port_->close();
		delete serial_port_;
		serial_port_ = NULL;
	}
}

bool SteerMotor::init(const std::string& serial_port,int baud_rate)
{
	if(!configure_port(serial_port, baud_rate))
		return false;
	this->startReadSerial();
	this->startRequestState(50);

	return this->selfCheck();
}

bool SteerMotor::reInit(const std::string& serial_port,int baud_rate)
{
	this->stop();
	return init(serial_port, baud_rate);
}

void SteerMotor::stop()
{
	this->stopReadSerial();
	this->stopRequestState();
}

bool SteerMotor::configure_port(std::string port,int baud_rate)
{
	try 
	{
		if(serial_port_ != NULL && serial_port_->isOpen())
		{
			serial_port_->close();
			delete serial_port_;
			serial_port_ = NULL;
		}
		serial_port_ = new serial::Serial(port,baud_rate,serial::Timeout::simpleTimeout(10)); 

		if (!serial_port_->isOpen())
		{
	        std::stringstream output;
	        output << "Serial port: " << port << " failed to open." << std::endl;
			delete serial_port_;
			serial_port_ = NULL;
			return false;
		} 
		else 
		{
	        std::stringstream output;
	        output << "Serial port: " << port << " opened successfully." << std::endl;
	        std::cout << output.str() <<std::endl;
		}
	} 
	catch (std::exception &e) 
	{
	    std::stringstream output;
	    output << "Error  " << port << ": " << e.what();
	    std::cout << output.str() <<std::endl;
	    return false;
	}

	serial_port_->flush();  //刷新发送接收缓冲区
	return true;
}

void SteerMotor::startReadSerial()
{
	if (is_read_serial_)
		return;
	is_read_serial_=true;
	read_serial_thread_ptr_ = 
	    std::shared_ptr<std::thread >(new std::thread(&SteerMotor::readSerialThread, this));
}

void SteerMotor::stopReadSerial()
{
	is_read_serial_ = false; //促使线程退出while循环，然后退出线程

	//线程退出之前，此处无法获得锁并阻塞，直到线程退出
	std::lock_guard<std::mutex> lck(read_serial_thread_mutex_); 
}

void SteerMotor::readSerialThread()
{
	std::lock_guard<std::mutex> lck(read_serial_thread_mutex_);

	serial_port_->flush(); //clear the old data from receive buffer
	while ( is_read_serial_) 
	{
		int len = 0;
		
#if USE_THREAD_SYNCHRONIZE
	    std::unique_lock<std::mutex> lck(cv_mutex_);
//	    std::cout << "waiting...\n";
        condition_variable_.wait(lck);
#else
		usleep(10000);
#endif
		try
		{
			// read data
			len = serial_port_->read(raw_buffer, MAX_LOAD_SIZE);
		}
		catch (std::exception &e)
		{
	        std::stringstream output;
	        output << "Error reading from serial port: " << e.what();
	        std::cout << output.str() <<std::endl;
    	}
//    	std::cout << "received "<< len << "bytes data" << std::endl;
    	if(len==0) continue;
    	
    	
		// add data to the buffer to be parsed
		
		BufferIncomingData(raw_buffer, len);
		
		/*
		std::cout << std::dec<< "length:" <<len <<std::endl;
		for(size_t i=0;i<len;i++)
			printf("%x\t",raw_buffer[i]);
		printf("\n\n");
		*/
	}
}

//requestAdcValue     1 register  2bytes 
//requestEnableStatus 2 registers 4bytes and the second byte is status (0 false, 1 true)
//requestMotorSpeed   3 registers 6bytes and the first two bytes is speed
//requestErrorMsg     4 registers 8bytes and the second byte is errorMsg

void SteerMotor::BufferIncomingData(uint8_t *message, int length)
{
	static int bufferIndex = 0;
	static const int MaxPkgLen = 15;
	static int dataLength =0;
	static int dataRemaind =0;
	static uint8_t pkg_buffer[MaxPkgLen];
	
//	for(int i=0; i<length; ++i)
//		printf("%0x\t",message[i]);
//	std::cout << std::endl;
	
	for(int i=0; i<length; ++i)
	{
		if(bufferIndex>MaxPkgLen-1) //avoid out of range
			bufferIndex = 0;
			
		uint8_t data = message[i];
//		std::cout << "bufferIndex: " << bufferIndex << "\n";
		if(0 == bufferIndex)
		{
			if(data == 0x01) //device id
				pkg_buffer[bufferIndex++] = data;
		}
		else if(1 == bufferIndex)
		{
			if(data == 0x03) //response read register
				pkg_buffer[bufferIndex++] = data;
			else
				bufferIndex = 0;
		}
		else if(2 == bufferIndex)
		{
		    pkg_buffer[bufferIndex++] = data;
		    dataLength = pkg_buffer[bufferIndex-1];
//			std::cout << " dataLength : " << dataLength << "\n";
		}
		else if(2+dataLength+2 == bufferIndex) //2(id,cmd) +dataLen +2(crcLen)
		{
			pkg_buffer[bufferIndex] = data;
			
			uint16_t check_num = generateModBusCRC_byTable(pkg_buffer,3+dataLength);
			
			
//			for(int i=0; i<=bufferIndex; ++i)
//				printf("0x%0x\t",pkg_buffer[i]);
//			std::cout << std::endl;
//			std::cout << std::hex << (check_num&0xff) << "\t"<< (check_num>>8)  << std::endl;
//			std::cout << std::hex << int(pkg_buffer[bufferIndex]) << "\t" << int(pkg_buffer[bufferIndex-1]);
			
			//判断校验位
			if((check_num&0xff)==pkg_buffer[bufferIndex-1] && (check_num>>8)==pkg_buffer[bufferIndex])
			{
				last_active_time_ms_ = 
					std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            #if USE_THREAD_SYNCHRONIZE
                uint16_t rawVal = pkg_buffer[3]*256 + pkg_buffer[4];
                
                if(response_data_type_ == DataResponse_AdcValue)
				{
					road_wheel_angle_ = 1.0 * rawVal * road_wheel_angle_resolution_ + road_wheel_angle_offset_;
					//std::cout << "road_wheel_angle_: " << road_wheel_angle_ << std::endl;
				}
                else if(response_data_type_ == DataResponse_EnableStatus)
				{
                    is_enabled_ = (rawVal>0);
					//std::cout << "steerMotor enable: " << int(is_enabled_) << "\r\n";
				}   
                else if(response_data_type_ == DataResponse_MotorSpeed)
                    motor_speed_ = rawVal;
                    
                else if(response_data_type_ == DataResponse_ErrorMsg)
                    error_code_ = rawVal;
				else if(response_data_type_ == DataResponse_Reboot)
					std::cout << "Steer Motor responsed the reboot command.\r\n";

            #else
				if(2 == dataLength)//adcValue
				{
					uint16_t adcValue = pkg_buffer[3]*256 + pkg_buffer[4];
					road_wheel_angle_ = 1.0 * adcValue * road_wheel_angle_resolution_ + road_wheel_angle_offset_;
				}
				else if(4 == dataLength) //enableStatus
					is_enabled_ = (pkg_buffer[4] >0);
				else if(6 == dataLength) //motor_speed
					motor_speed_ = pkg_buffer[3]*256+pkg_buffer[4];
				else if(8 == dataLength) //ErrorMsg
					error_code_ = pkg_buffer[3]*256+pkg_buffer[4];
            #endif
			}
			else
				std::cout << "check failed ~\n"; 
			
			//状态索引置零
			bufferIndex = 0;
		}
		else
			pkg_buffer[bufferIndex++] = data;
	}
}

void SteerMotor::sendCmd(const uint8_t* buf,int len)
{
	send_cmd_mutex_.lock();
	serial_port_->write(buf,len);
	send_cmd_mutex_.unlock();
#if USE_THREAD_SYNCHRONIZE
	condition_variable_.notify_one();
#endif
}

void SteerMotor::setSteeringSpeed(uint8_t speed)
{
	static uint8_t steeringSpeedCmd[8] = {0x01,0x06,0x00,0x6a}; //+2bytes speed +2bytes check_num
	uint16_t CRC_checkNum ;
	
	//speed range 1-100
	if(speed >100) 
		speed = 100;
		
	steeringSpeedCmd[4] = speed >> 8;
	steeringSpeedCmd[5] = speed & 0xff;
	CRC_checkNum = generateModBusCRC_byTable(steeringSpeedCmd,6);
	steeringSpeedCmd[6] = CRC_checkNum &0xff;
	steeringSpeedCmd[7] = CRC_checkNum >>8;

#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_SetSpeed;
#endif
	sendCmd(steeringSpeedCmd,8);
}

//启动电机数据请求线程
void SteerMotor::startRequestState(int duration)
{
	if(is_request_state_) return ;

	is_request_state_ = true;

	request_state_thread_ptr_ = 
	    std::shared_ptr<std::thread >(new std::thread(&SteerMotor::requestStateThread, this, duration));
}

//电机数据定时获请求线程
void SteerMotor::requestStateThread(int duration)
{
	std::lock_guard<std::mutex> lck(request_state_thread_mutex_);

	int cmd_inteval = 5; //指令间隔 ms
	int i = 0;
	while(is_request_state_)
	{
		++i; 
        this->requestMotorSpeed();
        sleep_ms(cmd_inteval); 

		if(i%3 == 0)
		{
			this->requestAdcValue();
			sleep_ms(cmd_inteval);
		}
		if(i%5 == 0)
		{
			this->requestEnableStatus();
        	sleep_ms(cmd_inteval);
		}
		if(i%10 == 0)
   		{
			this->requestErrorMsg();
			sleep_ms(cmd_inteval);
    	}
		sleep_ms(duration);
	}
}

//电机自检，首先查看错误码，然后尝试使能电机
bool SteerMotor::selfCheck()
{
	assert(is_read_serial_);
	assert(is_request_state_);
	int max_try_times = 50;
	std::cout << "SteerMotor is self checking..." << std::endl ;
	while(max_try_times--)
	{
		if(this->error_code_ != 0)
		{
			std::cerr << "SteerMotor is in error state! error_code: " << error_code_ << std::endl;
			break;
		}
			 
		this->enable(); //尝试使能
		sleep_ms(100); //等待

		if(this->is_enabled_)
		{
			this->disable(); //使能成功，关闭使能
			std::cout << "SteerMotor self check complete." << std::endl;
			return true;
		}
	}
	std::cerr << "steerMotor self check failed!" << std::endl;
	return false;
}

void SteerMotor::stopRequestState()
{
	is_request_state_ = false; //促使线程退出while循环，然后退出线程

	//线程退出之前，此处无法获得锁并阻塞，直到线程退出
	std::lock_guard<std::mutex> lck(request_state_thread_mutex_); 
}

float SteerMotor::getRoadWheelAngle() const
{
	//assert(is_request_state_);
	return road_wheel_angle_;
}

float SteerMotor::getMotorSpeed() const
{
	//assert(is_request_state_);
	return motor_speed_;
}

#define STEER_MOTOR_OFFLINE 255
#define STEER_ANGLE_SENSOR_ERROR 254

uint8_t SteerMotor::getErrorMsg() const 
{
	//assert(is_request_state_);
	uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>
		(std::chrono::system_clock::now().time_since_epoch()).count();
	if(now - last_active_time_ms_ > 1000)
		return STEER_MOTOR_OFFLINE; //离线
	if(road_wheel_angle_ > 50 || road_wheel_angle_ < -50)
		return STEER_ANGLE_SENSOR_ERROR;

	return error_code_;
}

void SteerMotor::enable()
{
    if(is_enabled_)
        return;

	//										                     				  0x01 //enable
	//											                    			  0x00 //disable
	const uint8_t steeringEnableCmd[11]={0x01,0x10,0x00,0x33,0x00,0x01,0x02,0x00,0x01,0x62,0x53};
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_SetEnable;
#endif
	sendCmd(steeringEnableCmd,11);
}

void SteerMotor::disable()
{
	const uint8_t steeringDisableCmd[11]={0x01,0x10,0x00,0x33,0x00,0x01,0x02,0x00,0x00,0xA3,0x93};
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_SetEnable;
#endif
	sendCmd(steeringDisableCmd, 11);
}

void SteerMotor::setSteeringRotate(float cycleNum)
{	//												  0x01,0x36 //positive address
	//												  0x01,0x35 //nagetive address
	static uint8_t steeringRotateCmd[13] = {0x01,0x10,0x01,0x36,0x00,0x02,0x04};//+4bytes pulseNum +2bytes check_num
	
	int pulseNum = -cycleNum*32768;
	
	//std::cout << "pulseNum: " << pulseNum << std::endl;
	
	if(pulseNum < 0)
		steeringRotateCmd[3] = 0x36; //clock wise ID
	else
		steeringRotateCmd[3] = 0x35; //
	steeringRotateCmd[7] = (pulseNum>>8)&0xFF;
	steeringRotateCmd[8] = (pulseNum>>0)&0xFF;
	steeringRotateCmd[9] = (pulseNum>>24)&0xFF;
	steeringRotateCmd[10] = (pulseNum>>16)&0xFF;
	
	uint16_t CRC_checkNum = generateModBusCRC_byTable(steeringRotateCmd,11);
	steeringRotateCmd[11] = CRC_checkNum &0xff;
	steeringRotateCmd[12] = CRC_checkNum >>8;
	
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_SetRotate;
#endif
	sendCmd(steeringRotateCmd,13);
}

void SteerMotor::requestAdcValue()
{
    //0x01 设备ID
    //0x03 读寄存器
    //0x40,0x0D 寄存器地址
    //0x00,0x01 寄存器长度，读1个字节
    //0x00,0x09 CRC16
	static const uint8_t getAdcValueCmd[8]  = {0x01,0x03,0x40,0x0D,0x00,0x01,0x00,0x09}; 
		//response-> 01 03 02 {0A 32} 3F 31  16bits AD value
		
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_AdcValue;
#endif
	sendCmd(getAdcValueCmd,8);
	
}

void SteerMotor::requestEnableStatus()
{
    //0x01 设备ID
    //0x03 读寄存器
    //0x00,0x33 使能状态寄存器地址
    //0x00,0x02 读取寄存器长度，读2个单位
#if USE_THREAD_SYNCHRONIZE
    uint8_t cmd[8] = {0x01,0x03,0x00,0x33,0x00,0x01};  //读取一个单位
#else
	uint8_t cmd[8] = {0x01,0x03,0x00,0x33,0x00,0x02};
#endif
	
	uint16_t CRC_checkNum = generateModBusCRC_byTable(cmd,6);
	cmd[6] = CRC_checkNum & 0xff;
	cmd[7] = CRC_checkNum >>8;
	
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_EnableStatus;
#endif
	sendCmd(cmd,8);
}

void SteerMotor::requestMotorSpeed()
{
    //0x01 设备ID
    //0x03 读寄存器
    //0x00,0x6A 转速寄存器地址
    //0x00,0x03 读取寄存器长度，读3个字节
    
#if USE_THREAD_SYNCHRONIZE
    uint8_t cmd[8] = {0x01,0x03,0x00,0x6A,0x00,0x01};  //读取一个单位
#else
	uint8_t cmd[8] = {0x01,0x03,0x00,0x6A,0x00,0x03};
#endif

	uint16_t CRC_checkNum = generateModBusCRC_byTable(cmd,6);
	cmd[6] = CRC_checkNum & 0xff;
	cmd[7] = CRC_checkNum >>8;
	
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_MotorSpeed;
#endif
	sendCmd(cmd,8);
}

void SteerMotor::requestErrorMsg()
{
    //0x01 设备ID
    //0x03 读寄存器
    //0x01,0x1F 转速寄存器地址
    //0x00,0x04 读取寄存器长度，读4个字节

#if USE_THREAD_SYNCHRONIZE
    uint8_t getErrorMsgCmd[8] = {0x01,0x03,0x01,0x1F,0x00,0x01};
#else
	uint8_t getErrorMsgCmd[8] = {0x01,0x03,0x01,0x1F,0x00,0x04};
#endif

	uint16_t CRC_checkNum = generateModBusCRC_byTable(getErrorMsgCmd,6);
	getErrorMsgCmd[6] = CRC_checkNum & 0xff;
	getErrorMsgCmd[7] = CRC_checkNum >>8;
	
	
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_ErrorMsg;
#endif
	sendCmd(getErrorMsgCmd,8);
}

void SteerMotor::clearErrorFlag()
{
    static uint8_t clear_cmd[8] = {0x01,0x06,0x01,0x31,0x00,0x08};
    
    uint16_t CRC_checkNum = generateModBusCRC_byTable(clear_cmd,6);
	clear_cmd[6] = CRC_checkNum &0xff;
	clear_cmd[7] = CRC_checkNum >>8;
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_ClearErrorFlag;
#endif
	sendCmd(clear_cmd,8);
}

void SteerMotor::reboot()
{
    static uint8_t cmd[8] = {0x01,0x06,0x01,0x31,0x00,0x2E};
    
    uint16_t CRC_checkNum = generateModBusCRC_byTable(cmd,6);
	cmd[6] = CRC_checkNum &0xff;
	cmd[7] = CRC_checkNum >>8;
#if USE_THREAD_SYNCHRONIZE
    std::unique_lock<std::mutex> lck(cv_mutex_);
    response_data_type_ = DataResponse_Reboot;
#endif
	sendCmd(cmd,8);
}


uint16_t SteerMotor::generateModBusCRC_byTable(const uint8_t *ptr,uint8_t size)
{
	uint16_t CRC16  = 0xffff;
    uint16_t tableNo = 0;
	int i = 0;
 
    for( ; i < size; i++)
    {
        tableNo = ((CRC16 & 0xff) ^ (ptr[i] & 0xff));
        CRC16  = ((CRC16 >> 8) & 0xff) ^ CRC16Table[tableNo];
    }
 
    return CRC16;  
}

void SteerMotor::setSpeedAndAngle(float speed, float angle)
{
	if(speed > max_speed_) speed = max_speed_;
	else if(speed < min_speed_) speed = min_speed_;

	float angle_diff = angle - road_wheel_angle_;

	if(angle_diff > max_road_wheel_angle_)
		angle_diff = max_road_wheel_angle_;
	else if(angle_diff < -max_road_wheel_angle_)
		angle_diff = -max_road_wheel_angle_;
	
	this->rotate(angle_diff, speed);
}

void SteerMotor::setRoadWheelAngle(float angle)
{
    float angle_diff = angle - road_wheel_angle_;

	if(angle_diff > max_road_wheel_angle_)
		angle_diff = max_road_wheel_angle_;
	else if(angle_diff < -max_road_wheel_angle_)
		angle_diff = -max_road_wheel_angle_;
	
    //根据角度偏差大小调节转速
    uint8_t rotate_speed = fabs(angle_diff)/max_road_wheel_angle_ *(max_speed_-min_speed_) + min_speed_;
    
    this->rotate(angle_diff, rotate_speed);
    
//    std::cout << "now: " << road_wheel_angle_ 
//              << "\texpect: " << angle 
//              << "\tdiff:"<<  angle_diff 
//              << "\tspeed: " << int(rotate_speed) << std::endl;
}

//默认旋转速度为20, 可利用PID控制器进行调节
void SteerMotor::rotate(float angle, uint8_t speed)
{
    //degreePerCycle 前轮转动一度对应的电机脉冲数，需要标定
	//x deg roadWheelAngle -> 1 cycle of steeringMotor
	static const float degreePerCycle = 10; //20
	float cycleNum = angle / degreePerCycle;
	
	if(motor_speed_ != speed)
	{
	    setSteeringSpeed(speed);
	    usleep(1000);
	}
	
	setSteeringRotate(cycleNum);
}

void SteerMotor::sleep_ms(int ms)
{
	std::this_thread::sleep_for(std::chrono::microseconds(ms));
}



