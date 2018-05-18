#ifndef _SD2400V2_H_
#define _SD2400V2_H_

//#include <stdbool.h>
#include "User_Clock.h"
#include <SoftWire.h>
//#include "stm8s.h"

#define RTC_SCL PC9 
#define RTC_INT PC8
#define RTC_SDA PC10

//TwoWire RTCWire(RTC_SCL,RTC_SDA,SOFT_FAST);
//typedef unsigned char unsigned char;
//typedef unsigned short uint;

/*
typedef struct
{
  unsigned char seconds;  // 0-59
  unsigned char minutes;  // 0-59
  unsigned char hour;     // 0-23
  unsigned char week;     // 0-6
  unsigned char day;      // 0-30
  unsigned char month;    // 0-11
  uint year;    // 2000+
} UTCTimeStruct;
*/

//#define true  1
//#define false 0

#define DevAddrR 0x65
#define DevAddrW 0x64
//控制寄存器地址
#define CTR1  0x0F
#define CTR2  0x10
#define CTR3  0x11
#define TDS0  (1<<4)
#define TDS1  (1<<5)

//定义倒计时中断标志位
#define INTDF 0x10
//闹钟报警中断标志位
#define INTAF 0x20
//读寄存器
void ReadRegs(void);
//设置当前时钟
bool  SetRtcTime(UTCTimeStruct *Time);
//获取当前的实时时钟
bool  GetRtcTime(UTCTimeStruct *Time);
//设置报警输出时间
bool  SetAlarmOutTime(UTCTimeStruct *Time);
//获取报警输出时间
bool  GetAlarmOutTime(UTCTimeStruct *Time);
//从用户区读取数据
bool  ReadUserData(unsigned char Offset,unsigned char *buf,unsigned char len);
//将数据存储到用户区
bool  SaveUserData(unsigned char Offset,unsigned char *buf,unsigned char len);
//设置倒计时
bool RTC_Set_CountdownIn(unsigned char Time,unsigned char unit);
//清除中断信号
 bool ClearAlarmOut(void);
//RTC 芯片的数据和中断端口初始化
void  RTC_GPIO_INI(void);
#endif /*_SD2400V2_H_ */
