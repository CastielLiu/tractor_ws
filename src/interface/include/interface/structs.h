#ifndef __STRUCTS__
#define __STRUCTS__
#include <can2serial/can2serial.h>

//can总线报文ID
enum 
{
    GPS_CAN_ID = 0x301,          //gps定位信息上报
    GPS_MSG_CAN_ID = 0x302,      //gps其他信息上报
    RECORD_PATH_CAN_ID = 0x200,  //请求记录路径
    DRIVERLESS_CAN_ID = 0x201,   //请求自动驾驶
    RESPONSE_CAN_ID = 0x205,     //应答报文
    RESET_CAN_ID = 0x203,        //系统复位(清除转向电机错误代码)
    HEARTBEAT_CAN_ID = 0x24,     //心跳包
};

typedef struct CanMsgs
{
    CanMsg_t gpsPos;
    CanMsg_t gpsMsg;
    CanMsg_t heartbeat;

    CanMsgs()
    {
        gpsPos.ID = GPS_CAN_ID;
        gpsMsg.ID = GPS_MSG_CAN_ID;
        heartbeat.ID = HEARTBEAT_CAN_ID;
        heartbeat.len = 2;
    }
    
} canMsgs_t;

typedef struct systemResetMsg
{
    uint8_t clearMotorError : 1;
    uint8_t rebootMotor     : 1;
    uint8_t brakeReset      : 1;
} systemResetMsg_t;

typedef struct heartbeatStruct
{
    uint8_t gpsState         :4;
    uint8_t steerMotorState  :4;
    uint8_t brakeSystemState :4;
    uint8_t driveSystemState :4;
} heartbeatStruct_t;

#endif