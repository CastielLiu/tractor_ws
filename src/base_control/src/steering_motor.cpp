#include"base_control/steering_motor.h"
#include<serial/serial.h>
#include<boost/thread.hpp>


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
static char raw_buffer[MAX_LOAD_SIZE];



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
	uint16_t requestErrorMsg();
	void steeringDisable();
	void steeringEnable();
	void steerControl(float rotate_angle, uint8_t rotate_speed=20);
	float &getRoadWheelAngle(){return angle_sensor_value_;}
  private:
	bool configure_port(std::string port,int baud_rate);
	void readSerialPort();
	uint16_t generateModBusCRC_byTable(const uint8_t *ptr,uint8_t size);
	void BufferIncomingData(unsigned char *message, int length);
	void sendCmd(const char* buf,int len);
	
	serial::Serial *serial_port_;
	bool is_read_serial_;
	boost::shared_ptr<boost::thread> read_serial_thread_ptr_;
	
	float angle_sensor_value_;
}

SteerMotor::SteerMotor()
{
	serial_port_ = NULL;
	is_read_serial_ = false;
	angle_sensor_value_ = 0.0;
}

SteerMotor::~SteerMotor()
{
	if(serial_port_ != NULL)
	{
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
	read_serial_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&SteerMotor::readSerialPort, this)));
}

void SteerMotor::stopReadSerial()
{
	is_read_serial_ = false;
}

void SteerMotor::readSerialPort()
{
	int len = 0;
	// continuously read data from serial port
	while (is_read_serial_ && ros::ok()) 
	{
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
		
		//ROS_INFO("read length :%d\r\n",len);
		BufferIncomingData(raw_buffer, len);
		
	//	std::cout << std::dec<< "length:" <<len <<std::endl;
		/*for(size_t i=0;i<len;i++)
			printf("%x\t",buffer[i]);
		printf("\n\n");
		*/
	}
}

void SteerMotor::BufferIncomingData(unsigned char *message, int length)
{
	static int bufferIndex = 0;
	static const MaxPkgLen = 10;
	static int dataLength =0;
	static int dataRemaind =0;
	static char pkg_buffer[MaxPkgLen];
	for(int i=0; i<length; ++i)
	{	
		if(bufferIndex>MaxPkgLen-1) //avoid out of range
			bufferIndex = 0;
		char data = message[i];
		
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
			uint16_t check_num = generateModBusCRC_byTable(pkg_buffer,3+dataLength+2);
			if(check_num&0xff == pkg_buffer[bufferIndex-1] && (check_num>>8) == pkg_buffer[bufferIndex])
			{
				if(2 == dataLength)
				{
					uint16_t adcValue = pkg_buffer[3]*256 + pkg_buffer[4];
				
					angle_sensor_value_ = 1.0 * (adcValue - g_AngleSensorAdValueOffset)*g_AngleSensorMaxAngle/g_AngleSensorMaxAdValue ;
				}
			}
			else
				ROS_INFO("check failed ~");
			
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
	CRC_checkNum = generateModBusCRC_byTable(g_setSpeed_8bytesCmd,6);
	steeringSpeedCmd[6] = CRC_checkNum &0xff;
	steeringSpeedCmd[7] = CRC_checkNum >>8;
	sendCmd(steeringSpeedCmd,8); 
}

void SteerMotor::sendCmd(const char* buf,int len)
{
	serial_port_->write(buf,len);
}

void SteerMotor::steeringEnable()
{
	//														0x01 //enable
	//														0x00 //disable
	const uint8_t steeringEnableCmd[11]={0x01,0x10,0x00,0x33,0x00,0x01,0x02,0x00,0x01,0x62,0x53};
	sendCmd(steeringEnableCmd,11);
}

void SteerMotor::steeringDisable()
{
	const uint8_t steeringDisableCmd[11]={0x01,0x10,0x00,0x33,0x00,0x01,0x02,0x00,0x00,0xA3,0x93};
	sendControlCmd(disable_11bytesCmd,11);
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

void SteerMotor::steerControl(float rotate_angle, uint8_t rotate_speed)
{
	//deg roadWheelAngle -> 1 cycle of steeringMotor
	static const float degreePerCycle = 20.0;
	float cycleNum = -rotate_angle / degreePerCycle;
	setSteeringSpeed(rotate_speed);
	usleep(1000);
	setSteeringRotate(cycleNum);
}

void SteerMotor::requestAdcValue()
{
	static const uint8_t getAdcValueCmd[8]  = {0x01,0x03,0x40,0x0D,0x00,0x01,0x00,0x09}; 
		//response-> 01 03 02 {0A 32} 3F 31  16bits AD value
	sendCmd(getAdcValueCmd,8);
}

uint16_t SteerMotor::requestErrorMsg()
{
	uint8_t i=0;
	uint8_t buf[7];
	uint16_t CRC_checkNum;
	uint8_t getErrorMsgCmd[8] = {0x01,0x03,0x01,0x1F,0x00,0x01,0xB4,0x30}; //-> 01 03 02 {00 0E} 39 80  errorMsg
	sendCmd(getErrorMsgCmd,8);
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


