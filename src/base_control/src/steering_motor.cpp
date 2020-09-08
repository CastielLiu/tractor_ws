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


#define MAX_LOAD_SIZE 100
static uint8_t raw_buffer[MAX_LOAD_SIZE];


SteerMotor::SteerMotor()
{
    use_condition_variable_ = false;
	serial_port_ = NULL;
	is_read_serial_ = false;
	road_wheel_angle_ = 0.0;
	road_wheel_angle_resolution_ = 180.0/4096;
	road_wheel_angle_offset_ = 0.0; //前轮转角偏移值
	is_enabled_ = false;
	errorMsg_ = 0x00;
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

bool SteerMotor::init(std::string serial_port,int baud_rate)
{
	if(!configure_port(serial_port, baud_rate))
		return false;
	this->startReadSerial();
	
	return true;
}

bool SteerMotor::configure_port(std::string port,int baud_rate)
{
	try 
	{
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

		serial_port_->flush();
		
	} 
	catch (std::exception &e) 
	{
	    std::stringstream output;
	    output << "Error  " << port << ": " << e.what();
	    std::cout << output.str() <<std::endl;
	    return false;
	}

	return true;
}

void SteerMotor::startReadSerial()
{
	if (is_read_serial_)
		return;
	is_read_serial_=true;
	read_serial_thread_ptr_ = 
	    std::shared_ptr<std::thread >(new std::thread(&SteerMotor::readSerialPort, this));
}

void SteerMotor::stopReadSerial()
{
	is_read_serial_ = false;
}

void SteerMotor::readSerialPort()
{
	serial_port_->flush(); //clear the old data from receive buffer
	while ( is_read_serial_) 
	{
		int len = 0;
		
		if(use_condition_variable_)
		{
		    std::unique_lock<std::mutex> lck(cv_mutex_);
            condition_variable_.wait(lck);
		}
		
		usleep(10000);
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
	static const int MaxPkgLen = 10;
	static int dataLength =0;
	static int dataRemaind =0;
	static uint8_t pkg_buffer[MaxPkgLen];
	for(int i=0; i<length; ++i)
	{
		if(bufferIndex>MaxPkgLen-1) //avoid out of range
			bufferIndex = 0;
		uint8_t data = message[i];
		
		if(0 == bufferIndex)
		{
			if(data == 0x01)
			{
				pkg_buffer[bufferIndex++] = data;
			}
		}
		else if(1 == bufferIndex)
		{
			if(data == 0x03)
			{
				pkg_buffer[bufferIndex++] = data;
			}
			else
				bufferIndex = 0;
		}
		else if(2 == bufferIndex)
		{
			if(data == 0x02) //length
			{
				dataLength = data;
				pkg_buffer[bufferIndex++] = data;
			}
			else
				bufferIndex = 0;
		}
		else if(2+dataLength+2 == bufferIndex) //2+dataLen+crcLen
		{
			pkg_buffer[bufferIndex] = data;
			
			uint16_t check_num = generateModBusCRC_byTable(pkg_buffer,3+dataLength);
			
			/*
			for(int i=0; i<=bufferIndex; ++i)
				printf("0x%0x\t",pkg_buffer[i]);
			std::cout << std::endl;
			std::cout << std::hex << (check_num&0xff) << "\t"<< (check_num>>8)  << std::endl;
			std::cout << std::hex << int(pkg_buffer[bufferIndex]) << "\t" << int(pkg_buffer[bufferIndex-1]);
			*/
			
			if((check_num&0xff)==pkg_buffer[bufferIndex-1] && (check_num>>8)==pkg_buffer[bufferIndex])
			{
				if(2 == dataLength)//adcValue
				{
					adcValue_ = pkg_buffer[3]*256 + pkg_buffer[4];
					//std::cout << "adcValue: " << std::dec <<adcValue_ << std::endl;
					road_wheel_angle_ = 1.0 * adcValue_ * road_wheel_angle_resolution_ + road_wheel_angle_offset_;
				}
				else if(4 == dataLength) //enableStatus
					is_enabled_ = (pkg_buffer[4] >0);
				else if(6 == dataLength) //motor_speed
					motor_speed_ = pkg_buffer[3]*256+pkg_buffer[4];
				else if(8 == dataLength) //ErrorMsg
					errorMsg_ = pkg_buffer[4];
				
			}
			else
				printf("check failed ~");
			
			bufferIndex = 0;
		}
		else
			pkg_buffer[bufferIndex++] = data;
	}
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
	sendCmd(steeringSpeedCmd,8);
}

void SteerMotor::sendCmd(const uint8_t* buf,int len)
{
	serial_port_->write(buf,len);
}

void SteerMotor::enable()
{
    if(is_enabled_)
        return;

		//														  0x01 //enable
		//														  0x00 //disable
	const uint8_t steeringEnableCmd[11]={0x01,0x10,0x00,0x33,0x00,0x01,0x02,0x00,0x01,0x62,0x53};
	sendCmd(steeringEnableCmd,11);
}

void SteerMotor::disable()
{
	const uint8_t steeringDisableCmd[11]={0x01,0x10,0x00,0x33,0x00,0x01,0x02,0x00,0x00,0xA3,0x93};
	sendCmd(steeringDisableCmd, 11);
}

void SteerMotor::setSteeringRotate(float cycleNum)
{	//												  0x01,0x36 //positive address
	//												  0x01,0x35 //nagetive address
	static uint8_t steeringRotateCmd[13] = {0x01,0x10,0x01,0x36,0x00,0x02,0x04};//+4bytes pulseNum +2bytes check_num
	
	int pulseNum = -cycleNum*32768;
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
	sendCmd(steeringRotateCmd,13);
}

void SteerMotor::setRoadWheelAngle(float angle)
{
    this->rotate(road_wheel_angle_ - angle);
}

void SteerMotor::rotate(float angle, uint8_t speed)
{
	//deg roadWheelAngle -> 1 cycle of steeringMotor
	static const float degreePerCycle = 20.0;
	float cycleNum = -angle / degreePerCycle;
	
	if(motor_speed_ != speed)
	{
	    setSteeringSpeed(speed);
	    usleep(1000);
	}
	
	setSteeringRotate(cycleNum);
}

void SteerMotor::requestAdcValue()
{
	static const uint8_t getAdcValueCmd[8]  = {0x01,0x03,0x40,0x0D,0x00,0x01,0x00,0x09}; 
		//response-> 01 03 02 {0A 32} 3F 31  16bits AD value
	sendCmd(getAdcValueCmd,8);
	
	if(use_condition_variable_)
	    condition_variable_.notify_one();
}

void SteerMotor::requestEnableStatus()
{
	uint8_t cmd[8] = {0x01,0x03,0x00,0x33,0x00,0x02};
	uint16_t CRC_checkNum = generateModBusCRC_byTable(cmd,8);
	cmd[6] = CRC_checkNum & 0xff;
	cmd[7] = CRC_checkNum >>8;
	sendCmd(cmd,8);
	if(use_condition_variable_)
	    condition_variable_.notify_one();
}

void SteerMotor::requestMotorSpeed()
{
	uint8_t cmd[8] = {0x01,0x03,0x00,0x6A,0x00,0x03};
	uint16_t CRC_checkNum = generateModBusCRC_byTable(cmd,8);
	cmd[6] = CRC_checkNum & 0xff;
	cmd[7] = CRC_checkNum >>8;
	sendCmd(cmd,8);
	if(use_condition_variable_)
	    condition_variable_.notify_one();
}

void SteerMotor::requestErrorMsg()
{
	uint8_t getErrorMsgCmd[8] = {0x01,0x03,0x01,0x1F,0x00,0x04}; 
	uint16_t CRC_checkNum = generateModBusCRC_byTable(getErrorMsgCmd,8);
	getErrorMsgCmd[6] = CRC_checkNum & 0xff;
	getErrorMsgCmd[7] = CRC_checkNum >>8;
	sendCmd(getErrorMsgCmd,8);
	if(use_condition_variable_)
	    condition_variable_.notify_one();
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


