
//**变量类型定义******************
#define u8 unsigned char 
#define u16 unsigned int
#define BLE_Serial Serial3
/*
设置BLE模块的工作模式
Param：（0-4）
0——从机（APP、微信） 透传模式
1——主机透传模式
2——主机（室内定位、传感器）观察
者模式
3——从机（iBeacon、传感器）模式
4——iBeacon 探测模式
*/
const   char AT_SET_BLE_SENSOR_MODE[]="AT+HOSTEN0";

const   char AT_SET_BLE_SENSOR_CLASS[]="AT+CLSSA1";

//设置BLE模块的串口波特率为9600
const   char AT_SET_BLE_BOUND[]="AT+BOUD0";
//重启BLE模块
const   char AT_SET_BLE_RST[]="AT+RST";
//开始发送传感器数据广播
const   char START_SEND_SNESOR_DATA[]="AT+ADVEN1";
//停止发送传感器数据广播
const   char STOP_SEND_SNESOR_DATA[]="AT+ADVEN0";
//设置发送广播的间隔
/*
Param：（0-9）
0——100ms
1——200ms
2——300ms
3——400ms
4——500ms
5——750ms
6——1000ms
7——2000ms
8——3000ms
9——4000ms
默认值： 0
*/
const   char SET_SEND_SNESOR_INT[]="AT+ADVIN3";
//设置BLE名称
const   char SET_BLE_NAME[]="AT+NAMEPreSenV1";
const   char WorR_BLE_MAC[]="AT+MAC";
const   char WorR_BLE_UUID[]="AT+STRUUID";
const   char WorR_BLE_MAJOR[]="AT+MAJOR";
const   char WorR_BLE_MINOR[]="AT+MINOR";
const   char WorR_BLE_VID[]="AT+VID";
const   char WorR_BLE_TEMP[]="AT+TEMP";
const   char WorR_BLE_HUM[]="AT+HUMID";
const   char WorR_BLE_BAT[]="AT+BATT";

//RTC
const   char EN_BLE_RTC_ALARM[]="AT+ALAMEN1";
const   char DIS_BLE_RTC_ALARM[]="AT+ALAMEN0";
const   char RW_BLE_RTC_ALARM[]="AT+RTCALAM";
const   char BLE_SLEEP_MODE[]="AT+SLEEP1";
const   char SET_BLE_POWER[]="AT+POWR1";
const   char BLE_CONNECTED[]="+connected";
//开启或关闭RTC功能
//Param（0-2）
//0：表示关闭 RTC 功能
//1：表示打开 RTC
//2：表示打开关下次上电打开
//默认： 0
const   char BLE_RTC_EN[]="AT+RTCOPEN2";
//设置RTC刷新时间
//Param： （1-9）秒
//1： 1 秒刷一次
const   char BLE_RTC_FLUSH_TIME[]="AT+RTCFLSH1";
//设置或查询RTC时间，不带参数据查询，带参数据查询为设置
//Param（xxxx-xx-xx,xx:xx:xx）
// <Param> 默认： 2014-12-05,12:07:08
//查询 返回： +RTCDATE:14-12-05,12:07:08
const   char BLE_RTC_SET_OR_QUEST[]="AT+RTCDATE";
//使能RTC闹钟功能
//Param（0-2）
//0：表示关闭 RTC 定时功能
//1：表示打开 RTC 定时功能
//默认： 0
const   char BLE_RTC_ALARM_EN[]="AT+ALAMEN1";
//设置或查询RTC闹钟时间,不带参数据查询，带参数据查询为设置
//Param=23:01:00
//表示 23 时 01 分 00 秒
//默认： 00:00:00
const   char BLE_RTC_ALARM_TIME[]="AT+RTCALAM";
const   char SET_BLE_MAC[]="AT+MAC552233445565";


/*******************************************************************************
* 函数名 :sim808_check_cmd
* 描述   : 
* 输入   : 
* 输出   : 
* 返回   : 
* 注意   : 

*******************************************************************************/
unsigned char* BLE_check_cmd_Ack(unsigned char *str)
{
  char *strx=0;
    //GPRS_BUF[USART1_RX_STA&0X7F]=0;//
// strx=strstr((const char*)Buffer,(const char*)str);
 
//  return (unsigned char*)strx;
}

/*******************************************************************************
* 函数名 :BLE_send_cmd
* 描述   : 向BLE模块发送指令并等待返回
* 输入   : cmd-命令、ack-等待的状态应答码、等待时长，
*          data-返回的应答数据
* 输出   : 命令
* 返回   : 状态数值
         0-成功
         1失败
* 注意   : 

*******************************************************************************/

static const char BLE_OK[]="+OK";
static const char BLE_ERROR[]= "+ERR";
unsigned char  BLE_send_cmd(char *cmd,String& data,char *ack,u16 waittime)
{ 
  unsigned char res=0;
 unsigned int Reindex;
 // String data;
  data.reserve(64);
  int mux = -1;
  int index = 0;
  BLE_Serial.write((char *)cmd,strlen((char*)cmd));
  delay(100);
  unsigned long startMillis = millis();
  do
  {
     while (BLE_Serial.available() > 0) {
        int a = BLE_Serial.read();
        data += (char)a;
        if (a <= 0) continue; // Skip 0x00 bytes, just in case
       // Serial1.println(data);
       if (ack && data.endsWith(ack)) 
       {
        index = 1;
        goto finish;
      }
      else if (data.endsWith("+RTCDATE:")) {
          String mode = BLE_Serial.readStringUntil('\n');
          data+=mode;
          data="";
          data=mode;
          index = 1;
          goto finish;
          /*
          if (mode.toInt() == 1) {
            mux = stream.readStringUntil('\n').toInt();
            data = "";
            sockets[mux]->got_data = true;
          } else {
            data += mode;
          }
          */
          
        }
        
        
     }
  } while (millis() - startMillis < waittime);
  finish:
    if (!index) {
      data.trim();
      if (data.length()) {
        //DBG("### Unhandled:", data);
        //Serial.println("### Unhandled:", data);
      }
      data = "";
    }
    return index;
}


