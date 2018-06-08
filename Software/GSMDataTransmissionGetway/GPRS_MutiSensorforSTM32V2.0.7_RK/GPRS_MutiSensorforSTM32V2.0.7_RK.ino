// Select your GSM modem:
#define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_ESP8266

#include <TinyGsmClient.h>
#include "hal_defs.h"
#include "math.h"
#include <stdlib.h>
#include "User_Clock.h"
#include "BCD_CON.h"
#include <SoftWire.h>
#include "BLE_AT_Command.h"
#include "GPSData.h"
#include "MODBUS_RTU_CRC16.h"
#include "User_CRC8.h"
#include <libmaple/iwdg.h>
#include <libmaple/nvic.h>
#include <libmaple/pwr.h>
#include <RtcDS3231.h>
#include <RTClock.h>
#include "Arduino.h"
#include <Wire.h>

#include <AT24C1024.h>
#include <EDB.h>

// Uncomment the line appropriate for your platform
#define TABLE_SIZE 131072 // 1 device
//#define TABLE_SIZE 232144 // 2 devices
//#define TABLE_SIZE 393216 // 3 devices
//#define TABLE_SIZE 524288 // 4 devices
// Use the AT24C1024 EEPROM as storage
// Create an EDB object with the appropriate write and read handlers
void writer(unsigned long address, byte data)
{
  EEPROM1024.write(address, data);
}

byte reader(unsigned long address)
{
  return EEPROM1024.read(address);
}


EDB EpromDb(&writer,&reader);


RTClock InRtc (RTCSEL_LSE); // initialise
//#define TINY_GSM_DEBUG Serial

//#define TINY_GSM_DEBUG Serial
#ifdef TINY_GSM_DEBUG
namespace
{
template <typename T>
static void DBG(T last)
{
  TINY_GSM_DEBUG.println(last);
}

template <typename T, typename... Args>
static void DBG(T head, Args... tail)
{
  TINY_GSM_DEBUG.print(head);
  TINY_GSM_DEBUG.print(' ');
  DBG(tail...);
}
}
#else
#define DBG(...)
#endif

typedef enum {
  HANDS_SHAK_ID = (u8)0xC0,
  SMART_HANDS_SHAK_ID = (u8)0xB0,
  GPRS_INFO_ID = (u8)0xC1,
  GPS_DATA_ID = (u8)0xD0,
  BAT_VOL_ID = (u8)0x09,
  BAT_VOL_ALARM_ID = (u8)0x0B,
  SENSOR_DATA_ID = (u8)0xDA,
  SET_MAC_ID = (u8)0x01,
  SENSOR_DATA_ALARM_ID = (u8)0x03
} FrameID_def;


//************IO口功能宏定义***********************
//#define SERIAL_USB
// Use Hardware Serial on Mega, Leonardo, Micro
#define GSM_Serial Serial2
#define ModbusSerial Serial3
#define Lora_Serial  Serial1

#define BUFFER_SIZE 64

//LevelConsistent
#define RS485TX PB10
#define USBDP PA12
#define LoRa_TX PA10
//外部按键输入 数字输入
#define KEY1_INPUT PA0
#define KEY2_INPUT PA1
//LED控制,数字输出
#define LED1 PC6
#define LED2 PC7
#define LED3 PB14
#define LED4 PB15

//外部ADC输入
#define HOST_VOL_ADC_INPUT_PIN PB1
#define A_5V_20mA_CANNEL1_PIN  PC5
#define A_5V_20mA_CANNEL2_PIN  PC4
#define A_5V_20mA_CANNEL3_PIN  PA7
//I2C总线
#define RTC_SCL PB6 
#define RTC_INT PC8
#define RTC_SDA PB7
#define RTC_RST PC9

TwoWire EXRTCWire(RTC_SCL,RTC_SDA,SOFT_FAST);
//#define EX_RTC
//RTC闹钟
#ifdef  EX_RTC
//RtcDS3231<HardWire> ExRtc(Wire);
RtcDS3231<TwoWire> ExRtc(EXRTCWire);
#endif

//TwoWire RTCWire(RTC_SCL,RTC_SDA,SOFT_FAST);

//外部数字状态输入IO
#define EX_DC_PWR_INPUT_STATUS  PB13
#define GPRS_NET_STATUS_INPUT_PIN PB3


#define RS485_BUS_PWR_PIN   PB0 
#define DC12V_PWR_PIN       PB12
#define LORA_PWR_PIN        PB8
#define RS485_DE_PIN        PC11
#define LORA_M0_PIN         PC10
#define LORA_M1_PIN         PB5
#define LORA_AUX_PIN        PC13

#define USB_EN_PIN          PB9

#define GPS_ANT_PWR_CON_PIN PC12
#define GPRS_RST_PIN        PB4
#define GPRS_PWRKEY_PIN     PA15
#define GPRS_PWR_CON_PIN    PD2  

#define RS485_SEND_EN    digitalWrite(RS485_DE_PIN, HIGH)
#define RS485_REVICE_EN  digitalWrite(RS485_DE_PIN, LOW)

#define LORA_PWR_ON digitalWrite(LORA_PWR_PIN, HIGH)
#define LORA_PWR_OFF digitalWrite(LORA_PWR_PIN, LOW)

#define DC12V_PWR_ON  digitalWrite(DC12V_PWR_PIN, HIGH)
#define DC12V_PWR_OFF  digitalWrite(DC12V_PWR_PIN, LOW)

#define RS485_BUS_PWR_ON digitalWrite(RS485_BUS_PWR_PIN, HIGH)
#define RS485_BUS_PWR_OFF digitalWrite(RS485_BUS_PWR_PIN, LOW)

#define GPRS_PWR_ON digitalWrite(GPRS_PWR_CON_PIN, HIGH)
#define GPRS_PWR_OFF digitalWrite(GPRS_PWR_CON_PIN, LOW)

#define GPRS_PWRKEY_HI digitalWrite(GPRS_PWRKEY_PIN, HIGH)
#define GPRS_PWRKEY_LO digitalWrite(GPRS_PWRKEY_PIN, LOW)

#define GPS_ANT_PWR_ON digitalWrite(GPS_ANT_PWR_CON_PIN, HIGH)
#define GPS_ANT_PWR_OFF digitalWrite(GPS_ANT_PWR_CON_PIN, LOW)

#define GPRS_RST_HI digitalWrite(GPRS_RST_PIN, HIGH)
#define GPRS_RST_LO digitalWrite(GPRS_RST_PIN, LOW)

#define RTC_INT_INPUT digitalRead(BLE_RTC_ALARM_INTRRUPT_PIN)
#define BLE_LINK_INPUT digitalRead(BLE_LINK_STATUS_INPUT_PIN)

#define LED1_ON digitalWrite(LED1, HIGH)
#define LED1_OFF digitalWrite(LED1, LOW)
#define LED2_ON digitalWrite(LED2, HIGH)
#define LED2_OFF digitalWrite(LED2, LOW)
#define LED3_ON digitalWrite(LED3, HIGH)
#define LED3_OFF digitalWrite(LED3, LOW)
#define LED4_ON digitalWrite(LED4, HIGH)
#define LED4_OFF digitalWrite(LED4, LOW)

#define  GSM_STATUS_LED_ON      LED3_ON
#define  GSM_STATUS_LED_OFF     LED3_OFF

#define  Server_STATUS_LED_ON   LED4_ON
#define  Server_STATUS_LED_OFF  LED4_OFF


#define USB_PORT_EN   digitalWrite(USB_EN_PIN,LOW)

#define USB_PORT_DIS digitalWrite(USB_EN_PIN,HIGH)

//ADC定义,12位ADC，参考电压为3.3V
#define ADC_RATE 0.8056 //3300/4096
//电池输入电压分压比
#define VBAT_DIVIDER_RATIO   6  
#define VCC 5000.0
//模拟电压输入通道（0~10V）电压分压比
#define ANALOG_DIVIDER_RATIO  11

//终端类型码
#define weatherStation 0x01 //气象站
#define greenHouse 0x02  //大棚

#define LBS 0
#define RS485_Addr 0x01
#define MOD_READ_DATA 0x03

//传感器数据存储起始地址
#define SYS_PARAM_BASE_ADDR   0
#define EEPROM_BASE_ADDR      128

//空引脚
#define GPIO_MODE_ANALOG PA8
#define GPIO_MODE_ANALOG PA4
#define GPIO_MODE_ANALOG PA5
#define GPIO_MODE_ANALOG PA6
#define GPIO_MODE_ANALOG PC0
#define GPIO_MODE_ANALOG PC1
#define GPIO_MODE_ANALOG PC2
#define GPIO_MODE_ANALOG PC3

//握手帧的应答帧结构体变量
 typedef struct
  {
   char HostUserID[4];//设备ID
   int acquisition_cycle;//采集周期
   int transmit_cycle;//发送周期
   int run_mode;//运行模式
   int positioning_mode;//定位模式
   int currentRecord;//上次发送的记录位
  }SYS_PARAM_STRUCT;
  SYS_PARAM_STRUCT SysParamstr;
 

  //传感器数据变量结构体定义
    typedef struct 
    {
      struct tm curTime;
      float SoilHum;//土壤湿度
      float SoilTem;//土壤温度
      int SoilSod;//土壤电导率
      int SoilSat;//土壤盐度
      float Screen_AirHum;//大气百叶箱湿度
      float Screen_AirTemp;//大气百叶箱温度
      unsigned long Screen_Beam;//大气百叶箱光照度
      float Screen_Atmos;//百叶箱大气压力
      float Screen_Gas;//百叶箱大气浓度
      int Screen_CO2;//百叶箱二氧化碳浓度
      int Screen_PM2_5;//百叶箱PM2.5
      int Screen_PM10;//百叶箱PM10
      float Screen_Noise;//百叶箱噪声值
      float UV_AirTemp;//UV传感器空气温度
      float UV_AirHum;//UV传感器空气湿度
      float Uv;//空气紫外线
      float WS;//风速
      int WindDirCode;//风向
      int Co2Ppm;//空气二氧化碳浓度
      float O2;//氧气
      int NH3;//氨气
      int H2S;//硫化氢
      unsigned long greenHouseLux;//大棚光照度
      float greenHouseAtmos;//大棚大气压力
      float greenHouseUV_AirTemp;//大棚UV传感器空气温度
      float greenHouseUV_AirHum;//大棚UV传感器空气湿度
      float greenHouseUV;//大棚紫外线
      int greenHouseCo2;//大棚二氧化碳浓度
      float greenHouseO2;//大棚氧气浓度
      }SENSOR_DATA_STRUCT;
      
  SENSOR_DATA_STRUCT Sensor_Data;
 
 //EEPROM最大存储记录数
  int EPMaxRecord = (TABLE_SIZE - EEPROM_BASE_ADDR)/sizeof(SENSOR_DATA_STRUCT)-1;
  #define RECORDS_TO_CREATE EPMaxRecord

/*RS485传感器地址定义*/
#define SOIL_SENSOR_ADDR                  0x01
#define CO2_PPM_SENSOR_ADDR               0x02
#define UV_HUM_TEMP_SENSOR_ADDR           0x03
#define AMBIENT_LIGHT_SENSOR_ADDR         0x04
#define ATMOSPHERIC_PRESSURE_SENSOR_ADDR  0x05
#define WIND_RATE_SENSOR_ADDR             0x06
#define WIND_DIRECTION_SENSOR_ADDR        0x07


//**********************全局变量定义**********************************
u8 BleAck = 1;
static unsigned char GetGPRSInfoTimeNum;      //获取GPRS信息 超时次数
static bool GSM_Connect_flag = false;         //GSM入网状态
static bool GPRS_Connect_flag = false;        //GPRS
static int GPRS_Connect_Server_Fail_Num = 0;  //GPRS连接服务器失败次数
static bool LBS_Connect_Seriver_flag = false; //基站定们服务器是否连接成功标志，TREU-成功，false-失败，不采集
static unsigned int LBS_Location_TryNum=0;//LBS基站定位失败尝试计数
static int toggle = 0;
static bool RecvGPRSDataflag;
static bool SendDataFlag;
static bool ReciveParamFlag=false;
static bool RequestID;
static bool TimeGetGPRSInfoFlag;
static bool StartConnectSeviceFlag;
static unsigned char DisSockeTimeNum;
static bool DisSocketFlag;
static unsigned char ConectTimeNum;
static unsigned int SysTime;
volatile uint32 RunTimeOutSec=0;
int numberOfTimeout =0;
static bool sendFunctionSign = false;//发送数据的中断函数发生标志

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[] = "CMIOT";
const char user[] = "";
const char pass[] = "";
// Server Address 
const char server[] = "x.x.x.x";
//for http  get 
//const char resource[] = "/vshymanskyy/tinygsm/master/extras/logo.txt";
// port.
int port = 8000;


// or Software Serial on Uno, Nano
//#include <SoftwareSerial.h>
//SoftwareSerial GSM_Serial(2, 3); // RX, TX

void(* resetFunc) (void) = 0;   //declare reset function at address 0
TinyGsm modem(GSM_Serial);
TinyGsmClient client(modem);

unsigned char Realflowcmd[6] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
unsigned char flowRatecmd[6] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x02};
unsigned char ForwardSumcmd[6] = {0x01, 0x03, 0x00, 0x08, 0x00, 0x02};
unsigned char BackwardSumFlowcmd[6] = {0x01, 0x03, 0x00, 12, 0x00, 0x02};
unsigned char DaySumFlowcmd[6] = {0x01, 0x03, 0x00, 136, 0x00, 0x02};
unsigned char MonthSumFlowcmd[6] = {0x01, 0x03, 0x00, 140, 0x00, 0x02};
unsigned char YearSumFlowcmd[6] = {0x01, 0x03, 0x00, 144, 0x00, 0x02};

#define CMD_LEN sizeof(Realflowcmd)

unsigned char *ModbusFunc[7] = {
    Realflowcmd, flowRatecmd, ForwardSumcmd, BackwardSumFlowcmd, DaySumFlowcmd, MonthSumFlowcmd, YearSumFlowcmd};

typedef enum
{
  REALFLOW = 0,
  FLOWRATE,
  FORWARDSUMFLOW,
  BACKWARDSUMLOW,
  DAYSUMFLOW,
  MONTHSUMLOW,
  YEARSUMFLOW
} MODBUS_CMD;

//自定义函数声明
static void SendBatVoltage(u16 currentVoltage, u8 IsAlarmFlag);
static void SendSensorDataToServer(void);
void Data_Acquisition(void);
void saveSensorDataToEEPROM();
float ReadModBusDataFloat(unsigned char cmdindex);
long ReadModBusDatalong(unsigned char cmdindex);
int storageDataFunction();
void SensorDataBaseInit(void);

void SIM800_PWR_CON(void)
{
  GPRS_PWRKEY_LO;
  delay(1500);
  GPRS_PWRKEY_HI;
}

unsigned char BLE_HOST_Mac[] = "112233445566";
unsigned char HostUserID[4] = {7, 1, 2, 3};
unsigned char Com_PWD[4] = {0x1A, 0xC4, 0xEE, 0x0B};
int pinInterrupt = 3; //PA3接中断信号的脚
unsigned char Pwm_v = 0;
#define BUF_MAX_SIZE 64
#define BLE_MAC_LENGHT 6
char SerRevBuf[BUF_MAX_SIZE] = {0};
unsigned char BLE_MAC[BLE_MAC_LENGHT] = {0};
unsigned char UserID[4] = {0};
unsigned char BLE_RSSI;
unsigned char Reindex = 0;
unsigned char SensorData[2] = {0};
unsigned int SenDataInt;
unsigned char SensorBatV[2] = {0};
unsigned int SensorBatVInt;
float Temp = 0.0;
unsigned char SendBuf[128] = {0};
unsigned char Sindex = 0;

float Sht2x_temp, Sht2x_hum;
//串口接收缓冲区定义
unsigned char Buffer[BUFFER_SIZE];
UTCTimeStruct RtcTime = {2017, 9, 5, 0, 16, 01, 23};
GPS_INFO GPS_Store_Data; //存放分离后的gps数据
static Flow_Str myFlow = {0};

//RTC外部中断初始化
  volatile uint16_t interuptCount = 0;
  volatile bool interuptFlag = false;

#ifdef  EX_RTC
void RTCinterruptInitialization()
{
    ExRtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

    if (!ExRtc.IsDateTimeValid()) 
    {
        Serial.println("RTC lost confidence in the DateTime!");
        ExRtc.SetDateTime(compiled);
    }

    if (!ExRtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        ExRtc.SetIsRunning(true);
    }
    RtcDateTime now = ExRtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        ExRtc.SetDateTime(compiled);
    }
    
    ExRtc.Enable32kHzPin(false);
    ExRtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmOne);//设置RTC模式 
}
#endif
void serialEvent1()
{
  //statements
  if (Serial.available())
  {
    byte inchar = (byte)Serial.read();
    Serial.println(inchar);
  }
}
void BLE_SerialserialEvent()
{
  //statements
  if (BLE_Serial.available())
  {
    byte inchar = (byte)BLE_Serial.read();
    SerRevBuf[Reindex++] = inchar;
    //Serial.print(SerRevBuf[Reindex-1],HEX);
    if (Reindex >= BUF_MAX_SIZE)
    {
      Reindex = 0;
    }
  }
}

String BleAckData;

//定时器1中断
void handler_time1(void)
{
  static u8 t = 0;
  static int k = 0;
  if (t > 100) // 定时器10毫秒中断一次
  {
    toggle ^= 1;
    digitalWrite(LED1, toggle);
    t = 0;
    SysTime++;
    RunTimeOutSec++;
    //如果运行超时，复位
    if(RunTimeOutSec>=300)
    {
        RunTimeOutSec=0;
        noInterrupts();
        nvic_sys_reset();
    }
   if (SysTime % 2 == 0)
    {
      TimeGetGPRSInfoFlag = true;
    }
    if (SysTime % 5 == 0)
    {
      SendDataFlag = true;
      //StartConnectSeviceFlag=true;
    }
    if (SysTime % 10 == 0)
    {
      RequestID = true;
    }
    if (DisSocketFlag == true)
    {
      DisSockeTimeNum++;
    }
    ConectTimeNum++;
  }
  else
    t++;
}

//发送GPS数据
void SendGPSInfo(void)
{
  unsigned char chflag;
  unsigned char Data_BCD[6] = {0};
  char intstr[15];
  unsigned char Slen;
  unsigned char NumOfDot = 0;
  unsigned long int Pos_Avr;
  unsigned int Rate = 0;
  int SendIndex = 0;
  unsigned char ch1, ch0;
  u8 index = 0;
  u8 i = 0;
  u8 CrcCode = 0;
  //定位有效才发送
  if (GPS_Store_Data.LocationFlag == '1')
  {
    //Eeprom_read((u8*)UsrNum,USR_NUM_ADDR,2);
    SendIndex = 0;
    Buffer[SendIndex++] = 0xFE;
    Buffer[SendIndex++] = GPS_DATA_ID;
    Buffer[SendIndex++] = 0x1B;
    //Eeprom_read((u8*)UsrNum,USR_NUM_ADDR,2);
    Buffer[SendIndex++] = HostUserID[2];
    Buffer[SendIndex++] = HostUserID[3];

    //GPS(纬度)
    Slen = strlen(GPS_Store_Data.latitudestr);
    FloatStringToIntString(GPS_Store_Data.latitudestr, intstr, &NumOfDot, Slen);

    ASC2BCD(Data_BCD, intstr, strlen(intstr));
    Buffer[SendIndex++] = Data_BCD[0];
    Buffer[SendIndex++] = Data_BCD[1];
    Buffer[SendIndex++] = Data_BCD[2];
    Buffer[SendIndex++] = Data_BCD[3];
    Buffer[SendIndex++] = Data_BCD[4];

    Buffer[SendIndex++] = 0xE0 | NumOfDot;

    if (GPS_Store_Data.NS == 'N')
    {
      Buffer[SendIndex++] = 0x01;
    }
    else if (GPS_Store_Data.NS == 'S')
    {
      Buffer[SendIndex++] = 0x02;
    }

    //GPS(纬度)
    Slen = strlen(GPS_Store_Data.longitudestr);
    FloatStringToIntString(GPS_Store_Data.longitudestr, intstr, &NumOfDot, Slen);

    ASC2BCD(Data_BCD, intstr, strlen(intstr));
    Buffer[SendIndex++] = Data_BCD[0];
    Buffer[SendIndex++] = Data_BCD[1];
    Buffer[SendIndex++] = Data_BCD[2];
    Buffer[SendIndex++] = Data_BCD[3];
    Buffer[SendIndex++] = Data_BCD[4];

    Buffer[SendIndex++] = 0xE0 | NumOfDot;

    if (GPS_Store_Data.EW == 'E')
    {
      Buffer[SendIndex++] = 0x01;
    }
    else if (GPS_Store_Data.EW == 'W')
    {
      Buffer[SendIndex++] = 0x02;
    }

    //海拔
    Slen = strlen(GPS_Store_Data.altitudestr);
    FloatStringToIntString(GPS_Store_Data.altitudestr, intstr, &NumOfDot, Slen);
    ASC2BCD(Data_BCD, intstr, strlen(intstr));

    //GPS_Store_Data.altitude=1800;
    Buffer[SendIndex++] = Data_BCD[0];
    Buffer[SendIndex++] = Data_BCD[1];
    Buffer[SendIndex++] = Data_BCD[2];
    Buffer[SendIndex++] = 0xE0 | NumOfDot;

    //RTC时间
    Buffer[SendIndex++] = HI_UINT16(GPS_Store_Data.D.year);
    Buffer[SendIndex++] = LO_UINT16(GPS_Store_Data.D.year);
    Buffer[SendIndex++] = GPS_Store_Data.D.month;
    Buffer[SendIndex++] = GPS_Store_Data.D.day;
    Buffer[SendIndex++] = GPS_Store_Data.D.hour;
    Buffer[SendIndex++] = GPS_Store_Data.D.minutes;
    Buffer[SendIndex++] = GPS_Store_Data.D.seconds;

    Buffer[SendIndex++] = 23;
    Buffer[SendIndex++] = 0x0D;

    //UartTX_Send_String(Buffer,SendIndex);
    if (client.connected())
    {
      //client.print(SendBuf);
      client.write(Buffer, SendIndex);
    }
    else
    {
      if (GPRS_Connect_flag == true)
      {
        digitalWrite(LED2, LOW);
        GPRS_Connect_flag = false;
        client.stop();
      }
    }
  }
  else
  {
    // sprintf(Buffer,"%d:GPS loction Fail!\r\r\0",BUILD_UINT16(HostUserID[2],HostUserID[3]));
    //Serial.println(Buffer);
    // UartTX_Send_String(Buffer,strlen((char*)Buffer));
  }
  memset(Buffer, 0x00, sizeof(Buffer));
}

/******************************************************************************
 * @fn          ReadModBusDataFloat
 *
 * @brief      从Modbus总线设备上读取一个浮点数
 *
 * @param       cmd-读取的数据类型
 * @return     None
 */
float ReadModBusDataFloat(unsigned char cmdindex)
{

  float readdata;
  unsigned char *cmdstr;
  cmdstr = ModbusFunc[cmdindex];
  // cmdstr=flowRatecmd;
  unsigned int crc16 = N_CRC16(cmdstr, CMD_LEN);
  char str[30];
  sprintf(str, "modbus CRC:%x\r\n", crc16);
  Serial.print(str);
  Sindex = 0;
  for (int i = 0; i < CMD_LEN; i++)
  {
    SendBuf[Sindex++] = cmdstr[i];
  }
  SendBuf[Sindex++] = HI_UINT16(crc16);
  SendBuf[Sindex++] = LO_UINT16(crc16);
  RS485_SEND_EN;
  ModbusSerial.write(SendBuf, Sindex);
  delay(500);
  RS485_REVICE_EN;
  char modRevBuf[10];
  unsigned char ModRexIndex = 0;

  while (ModbusSerial.available())
  {
    char c = ModbusSerial.read();
    modRevBuf[ModRexIndex++] = c;
    if (modRevBuf[0] == RS485_Addr && modRevBuf[1] == MOD_READ_DATA)
    {
      u8 revlen = modRevBuf[2];
      for (int i = 0; i < revlen; i++)
      {
        c = ModbusSerial.read();
        modRevBuf[ModRexIndex++] = c;
      }
      //读校验码
      c = ModbusSerial.read();
      modRevBuf[ModRexIndex++] = c;
      c = ModbusSerial.read();
      modRevBuf[ModRexIndex++] = c;
      unsigned int revCrc16 = BUILD_UINT16(modRevBuf[ModRexIndex - 1], modRevBuf[ModRexIndex - 2]);
      // crc16=N_CRC16(modRevBuf,ModRexIndex-3);

      u8 RealData[4];
      
      RealData[0] = modRevBuf[4];
      RealData[1] = modRevBuf[3];
      RealData[2] = modRevBuf[6];
      RealData[3] = modRevBuf[5];
      
      readdata = *(float *)RealData;
      sprintf(str, "modbus data:%f\r\n",readdata);
      Serial.print(str);
    }
    Serial.print(c, HEX);
  }
  return readdata;
}


/******************************************************************************
 * @fn          ReadModBusDatalong
 *
 * @brief      从Modbus总线设备上读取一个长整形数据
 *
 * @param       cmd-读取的数据类型
 * @return     None
 */
long ReadModBusDatalong(unsigned char cmdindex)
{

  long readdata;
  unsigned char *cmdstr;
  cmdstr = ModbusFunc[cmdindex];
  // cmdstr=flowRatecmd;
  unsigned int crc16 = N_CRC16(cmdstr, CMD_LEN);
  char str[30];
  sprintf(str, "modbus CRC:%x\r\n", crc16);
  Serial.print(str);
  Sindex = 0;
  for (int i = 0; i < CMD_LEN; i++)
  {
    SendBuf[Sindex++] = cmdstr[i];
  }
  SendBuf[Sindex++] = HI_UINT16(crc16);
  SendBuf[Sindex++] = LO_UINT16(crc16);
  ModbusSerial.write(SendBuf, Sindex);
  delay(1000);
  char modRevBuf[10];
  unsigned char ModRexIndex = 0;

  while (ModbusSerial.available())
  {
    char c = ModbusSerial.read();
    modRevBuf[ModRexIndex++] = c;
    if (modRevBuf[0] == RS485_Addr && modRevBuf[1] == MOD_READ_DATA)
    {
      u8 revlen = modRevBuf[2];
      for (int i = 0; i < revlen; i++)
      {
        c = ModbusSerial.read();
        modRevBuf[ModRexIndex++] = c;
      }
      //读校验码
      c = ModbusSerial.read();
      modRevBuf[ModRexIndex++] = c;
      c = ModbusSerial.read();
      modRevBuf[ModRexIndex++] = c;
      unsigned int revCrc16 = BUILD_UINT16(modRevBuf[ModRexIndex - 1], modRevBuf[ModRexIndex - 2]);
      // crc16=N_CRC16(modRevBuf,ModRexIndex-3);

      u8 RealData[4];
      RealData[0] = modRevBuf[5];
      RealData[1] = modRevBuf[6];
      RealData[2] = modRevBuf[3];
      RealData[3] = modRevBuf[4];
      readdata = *(long *)RealData;
      sprintf(str, "modbus data:%ld\r\n",readdata);
      Serial.print(str);
    }
    Serial.print(c, HEX);
  }
  return readdata;
}


//读土壤传感器的温度和湿度
void readSoilTempAndHumi(float *hum,float *tep,unsigned char addres)
{
  unsigned char temp_T[6] = {0x01,0x03,0x00,0x02,0x00,0x02};
  temp_T[0]=addres;
  float humi = 0.0;//定义一个湿度变量
  float temp = 0.0;//定义一个温度变量
  int DataLen=0;
  char cmdata[9];
  static char str[40]={0};
 //向串口发送温度湿度指令
  unsigned char cmdstr [8] = {0};
  unsigned int txcrc16 = N_CRC16(temp_T,sizeof(temp_T));
  int x = 0;
  for (int i = 0; i < sizeof(temp_T); i++)
  {
    cmdstr[x++] = temp_T[i];
  }
  cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
  cmdstr[x++] = ((txcrc16) & 0xFF);
  RS485_SEND_EN;
  ModbusSerial.write(cmdstr, x); 
  delay(10);
  //Serial.write(cmdstr,x);
  RS485_REVICE_EN;
  delay(200);
  int  a=0;
  //noInterrupts();
  //读串口温度湿度数据
  while(ModbusSerial.available()>0)
  {
    char c;
    for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
    // Serial.print(c);
    }
  
    if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
       DataLen=cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //Serial.write((unsigned char*)cmdata,a);
      unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      unsigned int rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-2);
      //Serial.print("EC sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("EC sensor rxcrc:");
      //Serial.println(rxcrc16);
      if (rxcrc16 == CalCrc)
      {
        int humi1 = (cmdata[3]<<8) | cmdata[4];
        if (humi1 < 0)
        {
        humi1 = -humi1;
        humi = (float) humi1/10.0;
        humi=-humi;
        }
        else
        {
          humi = (float) humi1/10.0;
        }

        /*
        Serial.println("humi:");
        Serial.print(humi);
        Serial.println("");
        */
        
      int temp1 = (cmdata[5]<<8) | cmdata[6];
      // Serial.println("");
        if (temp1 < 0)
        {
        temp1 = -temp1;
        temp = (float) temp1/10.0;
        temp=-temp;
        }
        else
        {
        temp = (float) temp1/10.0;
        }
        /*
        Serial.println("temp:");
        Serial.print(temp);
        Serial.println("");
        */
        delay(1000);
      }
    }
    }  
   *hum=humi;
   *tep=temp;
}


//读土壤传感器的电导率和盐度
void readCondAndSalt(int *cod,int *sat,int address)
{
  unsigned char soil_cond[6] = {0x01,0x03,0x00,0x14,0x00,0x02};
  unsigned char cmdstr[8] = {0};
  int x = 0;
  char cmdata[9] ={0};
  int  a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  soil_cond[0] = address;
  int cond = 0;//定义一个电导率变量
  int salt = 0;//定义一个盐度变量
  //写指令
  txcrc16 = N_CRC16(soil_cond,sizeof(soil_cond));

      for (int i = 0;i < sizeof(soil_cond);i++)
      {
        cmdstr[x++] = soil_cond[i];
      }
      cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
      cmdstr[x++] = ((txcrc16) & 0xFF);
      RS485_SEND_EN;
      ModbusSerial.write(cmdstr, x); 
      delay(10);
      //Serial.write(cmdstr,x);
      RS485_REVICE_EN;
      delay(200);
      
    //读数据
  while(ModbusSerial.available()>0)
  {
    for(int j = 0;j < 3;j++)
    {
     c = ModbusSerial.read();
     cmdata [a++] = c;
     //Serial.print(c);
    }
     if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
      DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
       // Serial.print(c);
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //tiaoshi
      //Serial.write((unsigned char*)cmdata,a);
      unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-2);
      //Serial.print("EC sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("EC sensor rxcrc:");
      //Serial.println(rxcrc16);
      if(rxcrc16 == CalCrc)
      {
        cond = (cmdata[3]<<8) | cmdata[4];      
 /*       Serial.println("cond:");
        Serial.print(cond);
        Serial.print("cm/us");
        Serial.println("");
        */
        
        salt = (cmdata[5]<<8) | cmdata[6];
        /*
        Serial.println("salt:");
        Serial.print(salt);
        Serial.print("mg/L");
        Serial.println("");
        */
        //delay(1000);
      }
    }    
  }
  *cod = cond;
  *sat = salt;
}


//读传感器的二氧化碳浓度
int readCo2(unsigned char address)
{
  unsigned char co2_T[6] = {0x01,0x03,0x00,0x05,0x00,0x01};
  int DataLen = 0;
  int x = 0;
  int  a = 0;
  char c;
  char cmdata[7] ={0};
  unsigned char cmdstr[8] ={0};
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  co2_T[0] = address;
  int co2 = 0;//定义一个二氧化碳变量
  //写指令
  txcrc16 = N_CRC16(co2_T,sizeof(co2_T));
      /*char str[2];
      sprintf(str, "cond CRC:%x\r\n", txcrc16);
      Serial.print(str);*/
    for (int i = 0;i < sizeof(co2_T);i++)
    {
        cmdstr[x++] = co2_T[i];
    }
      cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
      cmdstr[x++] = ((txcrc16) & 0xFF);
     // Serial.write(cmdstr,x);
      RS485_SEND_EN;
      ModbusSerial.write(cmdstr, x); 
      delay(100);
      //Serial.write(cmdstr,x);
      RS485_REVICE_EN;
      delay(500);   
    //读数据
  while(ModbusSerial.available()>0)
  {
      for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
     //Serial.print(c);
   }
     if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
      DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
        //Serial.print(c);
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //Serial.write((unsigned char*)cmdata,a);
      unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-2);
      //Serial.print("CO2 sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("CO2 sensor rxcrc:");
      //Serial.println(rxcrc16);
      if(rxcrc16 == CalCrc)
      {    
       co2 = (cmdata[3]<<8) | cmdata[4];
      //Serial.print("CO2:");
      //Serial.print(co2);
      //Serial.print("ppm");
      //Serial.println("");
      }
    }    
  }
  return co2;
}


//读UV（紫外线）传感器的温度和湿度
void readUVSensorAirTempAndHumi(float *hum,float *tep,int address)
{
  unsigned char UV_airTemp[6] = {0x01,0x03,0x00,0x00,0x00,0x02};
  unsigned char cmdstr[8] = {0};
  char cmdata[9] ={0};
  int x = 0;
  int  a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  UV_airTemp[0] = address;
  float humi = 0.0;//定义一个湿度变量
  float temp = 0.0;//定义一个温度变量
  
  //写指令
  txcrc16 = N_CRC16(UV_airTemp,sizeof(UV_airTemp));
  for (int i = 0; i < sizeof(UV_airTemp); i++)
  {
    cmdstr[x++] = UV_airTemp[i];
  }
  cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
  cmdstr[x++] = ((txcrc16) & 0xFF);
  RS485_SEND_EN;
  ModbusSerial.write(cmdstr, x); 
  delay(10);
  //Serial.write(cmdstr,x);
  RS485_REVICE_EN;
  delay(200);
  
  //noInterrupts();
  //读数据
  while(ModbusSerial.available()>0)
  {
    for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
    // Serial.print(c);
    }
    if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
       DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //tiaoshi
      //Serial.write((unsigned char*)cmdata,a);
      //unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      unsigned int CalCrc=cmdata[7] | cmdata[8]<<8;
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-3);
      //Serial.print("UV sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("UV sensor rxcrc:");
      //Serial.println(rxcrc16);
      if (rxcrc16 == cmdata[7] | cmdata[8]<<8)
      {
          int humi1 = (cmdata[3]<<8) | cmdata[4];
          if (humi1 < 0)
          {
          humi1 = -humi1;
          humi = (float) humi1/10.0;
          humi=-humi;
          }
          else
          {
            humi = (float) humi1/10.0;
          }
        // Serial.println("UV sensor's humi:");
        // Serial.print(humi);
        // Serial.print("%RH");
        // Serial.println("");
          
        int temp1 = (cmdata[5]<<8) | cmdata[6];
        // Serial.print(temp1);
          //Serial.println("");
          if (temp1 < 0)
          {
          temp1 = -temp1;
          temp = (float) temp1/10.0;
          temp=-temp;
          }
          else
          {
          temp = (float) temp1/10.0;
          }
          //Serial.println("UV sensor's temp:");
          //Serial.print(temp);
          //Serial.print("℃");
          //Serial.println("");
          delay(1000);
      }
    }
    }  
   *hum = humi;
   *tep = temp; 
}

//读UV(紫外线)传感器的紫外线强度
float readuv(int address)
{
  unsigned char uv_T[6] = {0x01,0x03,0x00,0x08,0x00,0x01};
  unsigned char cmdstr[8] = {0};
  char cmdata[9] ={0};
  int x = 0;
  int  a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  uv_T[0] = address;
  float uv = 0.0;//定义一个二氧化碳变量
  //写指令
  txcrc16 = N_CRC16(uv_T,sizeof(uv_T));
      for (int i = 0;i < sizeof(uv_T);i++)
  {
      cmdstr[x++] = uv_T[i];
  }
      cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
      cmdstr[x++] = ((txcrc16) & 0xFF);
      RS485_SEND_EN;
      ModbusSerial.write(cmdstr, x); 
      delay(10);
      //Serial.write(cmdstr,x);
      RS485_REVICE_EN;
      delay(200);
      
    //读数据
  while(ModbusSerial.available()>0)
  {
      for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
     //Serial.print(c);
 }
     if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
      DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
        //Serial.print(c);
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //tiaoshi
      //Serial.write((unsigned char*)cmdata,a);
      unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-2);
      //Serial.print("UV sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("UV sensor rxcrc:");
      //Serial.println(rxcrc16);
      if(rxcrc16 == CalCrc)
      {    
      float uv1 = (cmdata[3]<<8) | cmdata[4];
      uv = uv1/10.0;
      }
    }    
  }
  return uv;
}



//读光照度传感器的光照度
unsigned long readIllumination(int address)
{
  unsigned char illumination_T[6] = {0x01,0x03,0x00,0x07,0x00,0x02};
  unsigned char cmdstr[8] ={0};
  int x = 0;
  char cmdata[7] ={0};
  int  a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  illumination_T[0] = address;
  unsigned long illu = 0L;//定义一个光照度变量
  //写指令
  txcrc16 = N_CRC16(illumination_T,sizeof(illumination_T));
  for (int i = 0; i < sizeof(illumination_T); i++)
  {
    cmdstr[x++] = illumination_T[i];
  }
  cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
  cmdstr[x++] = ((txcrc16) & 0xFF);
  RS485_SEND_EN;
  ModbusSerial.write(cmdstr, x); 
  delay(10);
  RS485_REVICE_EN;
  delay(200);
  
  //读数据
  while(ModbusSerial.available()>0)
  {
    for(int j = 0;j < 3;j++)
     {
      c = ModbusSerial.read();
      cmdata [a++] = c;
      //Serial.print(c);
     }
    if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
       DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //tiaoshi
      //Serial.write((unsigned char*)cmdata,a);
      //unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      unsigned int CalCrc=cmdata[7] | cmdata[8]<<8;
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-3);
      //Serial.print("Illumination sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("Illumination sensor rxcrc:");
      //Serial.println(rxcrc16);
      if (rxcrc16 == cmdata[7] | cmdata[8]<<8)
      {
        illu = (cmdata[3]<<24) | (cmdata[4]<<16) | (cmdata[5]<<8) | cmdata[6];
        //Serial.println("UV sensor's illu:");
        //Serial.print(illu);
        delay(1000);
      }
    }
    }  
   return illu;
}


//读百叶箱传感器的大气温度和湿度
void readScreenAirTempAndHumi(float *hum,float *tep,int address)
{
  unsigned char Screen_airTemp[6] = {0x01,0x03,0x00,0x00,0x00,0x02};
  unsigned char cmdstr[8] = {0};
  char cmdata[9] ={0};
  int x = 0;
  int a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  Screen_airTemp[0] = address;
  float humi = 0.0;//定义一个湿度变量
  float temp = 0.0;//定义一个温度变量
  //写指令
  txcrc16 = N_CRC16(Screen_airTemp,sizeof(Screen_airTemp));
/*char str[2];
  sprintf(str, "temp&humi CRC:%x\r\n", txcrc16);
  Serial.print(str);*/
  for (int i = 0; i < sizeof(Screen_airTemp); i++)
  {
    cmdstr[x++] = Screen_airTemp[i];
  }
  cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
  cmdstr[x++] = ((txcrc16) & 0xFF);
  RS485_SEND_EN;
  ModbusSerial.write(cmdstr, x); 
  delay(10);
  RS485_REVICE_EN;
  delay(200);
  
  //noInterrupts();
  //读数据
  while(ModbusSerial.available()>0)
  {
    for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
    // Serial.print(c);
    }
    if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
       DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //tiaoshi
      //Serial.write((unsigned char*)cmdata,a);
      unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-2);
      //Serial.print("Screen sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("Screen sensor rxcrc:");
      //Serial.println(rxcrc16);
      if (rxcrc16 == CalCrc)
      {
      int humi1 = (cmdata[3]<<8) | cmdata[4];
      if (humi1 < 0)
      {
      humi1 = -humi1;
      humi = (float) humi1/10.0;
      humi=-humi;
      }
      humi = (float) humi1/10.0;
      
     int temp1 = (cmdata[5]<<8) | cmdata[6];
     //Serial.print(temp1);
      if (temp1 < 0)
      {
      temp1 = -temp1;
     //temp1 = temp1 | 0x8001; 
      temp = (float) temp1/10.0;
      temp=-temp;
      }
      temp = (float) temp1/10.0;
      delay(1000);
      }
    }
    }  
   *tep = temp;
   *hum = humi;
}


//读百叶箱传感器的光照强度
unsigned long readScreenBeam(int address)
{
  unsigned char all_T[6] = {0x01,0x03,0x00,0x07,0x00,0x02};
  unsigned char cmdstr[8] = {0};
  char cmdata[100] ={0};
  int x = 0;
  int a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  all_T[0] = address;
  unsigned long beam = 0L;//定义一个光照度变量
  //写指令
  txcrc16 = N_CRC16(all_T,sizeof(all_T));
      /*char str[2];
      sprintf(str, "cond CRC:%x\r\n", txcrc16);
      Serial.print(str);*/
      for (int i = 0;i < sizeof(all_T);i++)
  {
      cmdstr[x++] = all_T[i];
  }
      cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
      cmdstr[x++] = ((txcrc16) & 0xFF);
      //Serial.write(cmdstr,x);
      RS485_SEND_EN;
      ModbusSerial.write(cmdstr, x); 
      delay(10);
      RS485_REVICE_EN;
      delay(1000);

    //读数据
      while(ModbusSerial.available()>0)
  {
      for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
     //Serial.print(c);
     }
     if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
      DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
        //Serial.print(c);
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //Serial.write(cmdata,a);
      //unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      unsigned int CalCrc=cmdata[7] | cmdata[8]<<8;
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-3);
      //Serial.print("Screen sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("Screen sensor rxcrc:");
      //Serial.println(rxcrc16);
      if(rxcrc16 == cmdata[7] | cmdata[8]<<8)
      {
      beam = (cmdata[3]<<24) | (cmdata[4]<<16) | (cmdata[5]<<8) | cmdata[6];
      //Serial.print("Illuminance:");
      //Serial.println(beam);
      }
    }
  }
  return beam;
} 


//读百叶箱传感器的大气压力
float readScreenAtmos(int address)
{
  unsigned char other[6] = {0x01,0x03,0x00,0x0a,0x00,0x02};
  unsigned char cmdstr[8] = {0};
  char cmdata[100] ={0};
  int x = 0;
  int a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  other[0] = address;
  float atmos = 0.00;//定义一个大气压力变量
  //写指令
  txcrc16 = N_CRC16(other,sizeof(other));
      /*char str[2];
      sprintf(str, "cond CRC:%x\r\n", txcrc16);
      Serial.print(str);*/
      for (int i = 0;i < sizeof(other);i++)
  {
      cmdstr[x++] = other[i];
  }
      cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
      cmdstr[x++] = ((txcrc16) & 0xFF);
      //Serial.write(cmdstr,x);
      RS485_SEND_EN;
      ModbusSerial.write(cmdstr, x); 
      delay(10);
      RS485_REVICE_EN;
      delay(1000);
      
    //读数据
      while(ModbusSerial.available()>0)
  {
      for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
     //Serial.print(c);
}
     if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
      DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
        //Serial.print(c);
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //Serial.write(cmdata,a);
      //unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      unsigned int CalCrc=cmdata[7] | cmdata[8]<<8;
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-3);
      //Serial.print("Screen sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("Screen sensor rxcrc:");
      //Serial.println(rxcrc16);
      if(rxcrc16 == cmdata[7] | cmdata[8]<<8)
      {
      float atmos1 = (cmdata[3]<<24) | (cmdata[4]<<16) | (cmdata[5]<<8) | cmdata[6];
      atmos = atmos1/100.00;
      //Serial.print("Atmospheric_Pressure:");
      //Serial.println(atmos);
      delay(10);
      }
    }
  }
  return atmos;
} 


//读风速传感器的风速
float readWindSpeed(int address)
{
  unsigned char wind_speed[6] = {0x01,0x03,0x00,0x00,0x00,0x01};
  unsigned char cmdstr[8] = {0};
  int x = 0;
  char cmdata[7] ={0};
  int  a = 0;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  wind_speed[0] = address;
  float ws = 0.0;//定义一个风速变量
  
  //写指令
  txcrc16 = N_CRC16(wind_speed,sizeof(wind_speed));
  char str[2];
  //sprintf(str, "read WindSpeed Cmd CRC:%x\r\n", txcrc16);
  //Serial.print(str);
  for (int i = 0; i < sizeof(wind_speed); i++)
  {
    cmdstr[x++] = wind_speed[i];
  }
  cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
  cmdstr[x++] = ((txcrc16) & 0xFF);
  RS485_SEND_EN;
  ModbusSerial.write(cmdstr, x); 
  /*for(int k=0;k<sizeof(cmdstr);k++)
  {
    Serial.print(cmdstr[k],HEX);
    Serial.print(" ");
  }
  Serial.println("");*/
  delay(10);
  RS485_REVICE_EN;
  delay(1000);

  
  //读数据
  while(ModbusSerial.available()>0)
  {
    for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
     //Serial.print(c,HEX);
    }
    if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
       DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
        //Serial.print(c,HEX);
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //debug
      //Serial.write((unsigned char*)cmdata,a);
      unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-2);
      //Serial.print("Wind speed sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("Wind speed sensor rxcrc:");
      //Serial.println(rxcrc16);
      if (rxcrc16 == CalCrc)
      {
        float ws1 = (cmdata[3]<<8) | cmdata[4];
        // Serial.println(ws1);
        ws = ws1/10.0;
        //Serial.print("wind Speed:");
        //Serial.println(ws);
        delay(1000);
      }
    }
  }  
   return ws;
}



//读风向传感器的风向
int readWindDirection(int address)
{
  unsigned char wind_direction[6] = {0x01,0x03,0x00,0x00,0x00,0x02};
  char windDirection[10] = {0x00,0x2D,0x5A,0x87,0xB4,0xE1,0x01,0x0E,0x01,0x3B};
  unsigned char cmdstr[8] = {0};
  int x = 0;
  char cmdata[7] ={0};
  int  a = 0;
  int compare = 0x0006;
  char c;
  int DataLen = 0;
  unsigned int txcrc16 = 0;
  unsigned int rxcrc16 = 0;
  wind_direction[0] = address;
  int wind = 0;//定义一个风向变量
  int wind1 = 0;
  //写指令
  txcrc16 = N_CRC16(wind_direction,sizeof(wind_direction));
  //char str[30];
  //sprintf(str, "wind read CRC:%x\r\n", txcrc16);
  //Serial.print(str);

  for (int i = 0; i < sizeof(wind_direction); i++)
  {
    cmdstr[x++] = wind_direction[i];
  }
  cmdstr[x++] = ((txcrc16) >> 8) & 0xFF;
  cmdstr[x++] = ((txcrc16) & 0xFF);
  RS485_SEND_EN;
  ModbusSerial.write(cmdstr, x); 
  /*for(int k=0;k<sizeof(cmdstr);k++)
  {
    Serial.print(cmdstr[k],HEX);
    Serial.print(" ");
  }
  Serial.println("");*/
  delay(10);
  RS485_REVICE_EN;
  delay(100);
  //读数据
  while(ModbusSerial.available()>0)
  {
    for(int j = 0;j < 3;j++)
     {
     c = ModbusSerial.read();
     cmdata [a++] = c;
     //Serial.print(c,HEX);
    }
    if (cmdata[0] == cmdstr[0] && cmdata[1] == cmdstr[1])
    {
       DataLen = cmdata[2];
      for (int i = 0;i < DataLen;i++)
      {
        c = ModbusSerial.read();
        cmdata[a++] = c;
        //Serial.println(c,HEX);
      }
      c = ModbusSerial.read();
      cmdata[a++] = c;
      c = ModbusSerial.read();
      cmdata[a++] = c;
      //Serial.write((unsigned char*)cmdata,a);
      unsigned int CalCrc=cmdata[a-2]<<8 | cmdata[a-1];
      rxcrc16 = N_CRC16 ((unsigned char*)cmdata,a-2);
      //Serial.print("Wind direction sensor CalCrc:");
      //Serial.println(CalCrc);
      //Serial.print("Wind direction sensor rxcrc:");
      //Serial.println(rxcrc16);
      if (rxcrc16 == CalCrc)
      {
        wind = (cmdata[3]<<8) | cmdata[4];
        wind1 = (cmdata[5]<<8) | cmdata[6];
        if(wind < compare)
        {
           if(windDirection[wind] == wind1)
          {
            return wind;
          }
        }
        else
        {
          int six = windDirection[wind]<<8|windDirection[wind+1];
          int seven = windDirection[wind+1]|windDirection[wind+2];
          if(six==wind1 || seven==wind1)
          {
            return wind;
          }
        }
        delay(200);
        //Serial.print("wind Direction:");
        //Serial.println(wind);
      }
    }
  }  
}



// Variables:
int sensorMin = 1023;  // minimum sensor value
int sensorMax = 0;     // maximum sensor value
float sensorValue = 0; // the sensor value

//初始化GPIO的状态


void MCU_GPIO_INIT(void)
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  pinMode(KEY1_INPUT,INPUT);
  pinMode(KEY2_INPUT,INPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  pinMode(EX_DC_PWR_INPUT_STATUS,INPUT);
  pinMode(GPRS_NET_STATUS_INPUT_PIN,INPUT);

  //ADC 输入通道
  pinMode(HOST_VOL_ADC_INPUT_PIN,INPUT_ANALOG);
  pinMode(A_5V_20mA_CANNEL1_PIN,INPUT_ANALOG);
  pinMode(A_5V_20mA_CANNEL2_PIN,INPUT_ANALOG);
  pinMode(A_5V_20mA_CANNEL3_PIN,INPUT_ANALOG);

  pinMode(RS485_BUS_PWR_PIN,OUTPUT);
  pinMode(DC12V_PWR_PIN,OUTPUT);
  pinMode(LORA_PWR_PIN,OUTPUT);
  pinMode(RS485_DE_PIN,OUTPUT);
  pinMode(LORA_M0_PIN,OUTPUT);
  pinMode(LORA_M1_PIN,OUTPUT);
  pinMode(LORA_AUX_PIN,OUTPUT);

  pinMode(USB_EN_PIN,OUTPUT);
  pinMode(GPRS_PWR_CON_PIN, OUTPUT);
  pinMode(GPRS_RST_PIN, OUTPUT);
  pinMode(GPRS_PWRKEY_PIN, OUTPUT);
  pinMode(GPS_ANT_PWR_CON_PIN, OUTPUT);

  //空引脚
  pinMode(GPIO_MODE_ANALOG,INPUT);

  //RTC I2C 接口
  //pinMode (RTC_SCL,INPUT);
  //pinMode (RTC_SDA,INPUT);
  pinMode (RTC_INT,INPUT);
}

//LevelConsistent
void LevelConsistent()
{
  digitalWrite(RTC_SDA,HIGH);
  digitalWrite(RTC_SCL,HIGH);
  GPS_ANT_PWR_OFF;
  digitalWrite(RTC_RST,HIGH);
  digitalWrite(RTC_INT,HIGH);
  digitalWrite(EX_DC_PWR_INPUT_STATUS,LOW);
  digitalWrite(RS485TX,HIGH);
  digitalWrite(HOST_VOL_ADC_INPUT_PIN,LOW);
  GPRS_PWRKEY_LO;
  digitalWrite(USBDP,LOW);
  digitalWrite(LoRa_TX,HIGH);
}

void SYS_Sleep()
{
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);//开电源管理时钟PWR_Regulator_LowPower
  Timer2.detachCompare1Interrupt();
  //LevelConsistent();

  LED1_OFF;
  LED2_OFF;
  LED3_OFF;
  LED4_OFF;
  GPRS_PWR_OFF;
  LORA_PWR_OFF;
  DC12V_PWR_OFF;
  USB_PORT_DIS;
  //pinMode(GPRS_NET_STATUS_INPUT_PIN,OUTPUT);
  //digitalWrite(GPRS_NET_STATUS_INPUT_PIN,LOW);
  //delay(1000);
  //gpio_init_all();
  iwdg_feed();
  //rcc_clk_enable(RCC_PWR);
  PWR_WakeUpPinCmd(ENABLE);//使能唤醒引脚，默认PA0
  PWR_ClearFlag(PWR_FLAG_WU);
  PWR_EnterSTANDBYMode();//进入待机
  //SYS_Sleepy();//进入睡眠模式
  //PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI || PWR_STOPEntry_WFE);//进入停机模式
}

void systemWakeUp()
{
  nvic_sys_reset();
}

void externRTCWakeUP()
{
  //detachInterrupt(RTC_INT);
  //rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);
  //init();
  //Sys_Clocks_Init();
  //systick_init(SYSTICK_RELOAD_VAL);

  MCU_GPIO_INIT();
  LED1_ON;
  Serial1.begin(9600);
  delay(10);
  DC12V_PWR_ON;
  delay(200);

  RS485_BUS_PWR_ON;
  delay(200);
  LORA_PWR_ON;
  delay(200);
  USB_PORT_EN;
  delay(200);
  Serial.begin(115200);
  ModbusSerial.begin(9600);
}


void setup()
{
 // PWR_ClearFlag(PWR_FLAG_WU);
  //PWR_ClearFlag(PWR_FLAG_SB);
  /*
  rcc_clk_enable(RCC_PWR);
  //  if(PWR_GetFlagStatus(PWR_FLAG_WU)==SET)
   {
     //Serial.println("Standby mode Wake UP!");
     PWR_ClearFlag(PWR_FLAG_WU);
     LED1_ON;
   }
   */
  MCU_GPIO_INIT();
  
  GPRS_RST_LO;
  DC12V_PWR_ON;
  delay(1000);
  RS485_BUS_PWR_ON;
  LORA_PWR_ON;
  delay(1000);
  //UsbDebugSerial.begin(9600);
  //初始化USB模拟串口
  USB_PORT_EN;
  delay(1000);
  Serial.begin(115200);
  //使能USB
   
   EXRTCWire.begin();
   EXRTCWire.setClock(400000);


   
 // SWire.begin(0);
 // SWire.setClock(100000); //100K速率
  // Restart takes quite some time
  // To skip it, call init() instead of restart()

  //RTCWire.begin();

  //RTCWire.i2c_start();
  //RTCWire.i2c_stop();
  //恢复出厂设置
  //modem.factoryDefault();

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");
  //BLE_Serial.begin(115200);
  ModbusSerial.begin(9600);
  //BLE_Serial.println("SoftwareSerial start Work!");
  RtcTime.year = 2017;
  RtcTime.month = 9;
  RtcTime.day = 20;
  RtcTime.week = 1;
  RtcTime.hour = 20;
  RtcTime.minutes = 01;
  RtcTime.seconds = 10;
  memset(Buffer, 0x00, sizeof(Buffer));


  Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
  Timer2.setPeriod(10000); // in microseconds，10MS
  Timer2.setCompare1(1);   // overflow might be small
  Timer2.attachCompare1Interrupt(handler_time1);
  LBS_Location_TryNum=0;
  //iwdg_init(IWDG_PRE_256,1000);
 // attachInterrupt(PB1,ext1,FALLING);

   memset((SYS_PARAM_STRUCT*)&SysParamstr,0x00,sizeof(SYS_PARAM_STRUCT));
 //采集数据函数的（RTC）外部中断
  //HardWire.endTransmission();
  #ifdef  EX_RTC
  RTCinterruptInitialization();//初始化RTC中断并检测
  RtcDateTime now = ExRtc.GetDateTime();
  RtcDateTime alarmTime = now + SysParamstr.acquisition_cycle; // into the future
  DS3231AlarmOne alarm1(alarmTime.Day(),alarmTime.Hour(),alarmTime.Minute(), alarmTime.Second(),DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  ExRtc.SetAlarmOne(alarm1);
   // throw away any old alarm state before we ran
  ExRtc.LatchAlarmsTriggeredFlags();
  // setup external interupt 
  attachInterrupt(RTC_INT, InteruptServiceRoutine, FALLING);
  #endif

  //发送数据函数的（RTC）内部中断
  noInterrupts();
  time_t sendAlarmTime = 0;
  sendAlarmTime = InRtc.getTime();
  SysParamstr.transmit_cycle=240;
  sendAlarmTime += SysParamstr.transmit_cycle;
  InRtc.createAlarm(SendDataFunctionSign,sendAlarmTime);
  interrupts();

  SensorDataBaseInit();
  sendFunctionSign = true;

  if(digitalRead(KEY2_INPUT)==LOW)
  {
      delay(100);
      if(digitalRead(KEY2_INPUT)==LOW)
      {
        EpromDb.open(EEPROM_BASE_ADDR);
        EpromDb.clear(); 
        EpromDb.open(SYS_PARAM_BASE_ADDR);
        EpromDb.clear(); 
        Serial.println("Clear System param!");
      }

  }

 //for(int j=0;j<10;j++)
 //{
   //上电采集数据
  Data_Acquisition();
  //delay(5000);
 //}
 RS485_BUS_PWR_OFF;

  //开启GPRS模块电源
  GPRS_PWR_ON;
  digitalWrite(LED2, HIGH);
  SIM800_PWR_CON();
  digitalWrite(LED2, LOW);
  // Set console baud rate
  Serial1.begin(9600);
  delay(10);
  // Set GSM module baud rate
  GSM_Serial.begin(9600);
  delay(3000);
  Serial.println(F("Initializing modem..."));
  modem.restart();
  GPS_ANT_PWR_ON;
} 

//读取EEPROM里的参数
void readEPServerParam()
{
    EpromDb.open(SYS_PARAM_BASE_ADDR);
    int paramCount=EpromDb.count();
    Serial.print("Current server param Count:");
    Serial.println(paramCount);
    EDB_Status result = EpromDb.readRec(paramCount,EDB_REC SysParamstr);
    if (result == EDB_OK)
    {
      Serial.print("Acquisition cycle:"); 
      Serial.println(SysParamstr.acquisition_cycle);
      Serial.print("Transmit cycle:"); 
      Serial.println(SysParamstr.transmit_cycle);   
    }
    else 
    Serial.println(result);
    delay(100);
    //完成清除数据库 
    //EpromDb.clear();
}


//发送数据的中断函数标志
void SendDataFunctionSign()
{
 // noInterrupts();
  //Data_Acquisition();//采集传感器数据和存储数据函数
  rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);
  Serial.println("RTC alarm interrupt");
  sendFunctionSign = true;
  systemWakeUp();
 // interrupts();
}

//采集数据的中断函数标志
void InteruptServiceRoutine()
{
  externRTCWakeUP();
  interuptCount++;
  interuptFlag = true;
  //判断采集数据中断标志
    if (Alarmed())
    {
      Serial.print(">>Interupt Count: ");
      Serial.print(interuptCount);
      Serial.println("<<");
    }
  //rtc_attach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT, SendDataFunctionSign);
 // SYS_Sleep();//休眠
}

void sensorDataInit()
{
    Sensor_Data.SoilHum = 0xff;//土壤湿度
    Sensor_Data.SoilTem = 0xff;//土壤温度
    Sensor_Data.SoilSod = 0xff;//土壤电导率
    Sensor_Data.SoilSat = 0xff;//土壤盐度
    Sensor_Data.Screen_AirHum = 0xff;//百叶箱湿度
    Sensor_Data.Screen_AirTemp = 0xff;//百叶箱温度
    Sensor_Data.Screen_Beam = 0xff;//百叶箱光照度
    Sensor_Data.Screen_Atmos = 0xff;//百叶箱大气压力
    Sensor_Data.Screen_Gas = 0xff;//百叶箱大气浓度
    Sensor_Data.Screen_CO2 = 0xff;//百叶箱二氧化碳浓度
    Sensor_Data.Screen_PM2_5 = 0xff;//百叶箱PM2.5
    Sensor_Data.Screen_PM10 = 0xff;//百叶箱PM10
    Sensor_Data.Screen_Noise = 0xff;//百叶箱噪声值
    Sensor_Data.UV_AirTemp = 0xff;//UV传感器空气温度
    Sensor_Data.UV_AirHum = 0xff;//UV传感器空气湿度
    Sensor_Data.Uv = 0xff;//空气紫外线
    Sensor_Data.WS = 0xff;//风速
    Sensor_Data.WindDirCode = 0xff;//风向
    Sensor_Data.Co2Ppm = 0xff;//空气二氧化碳浓度
    Sensor_Data.O2 = 0xff;//氧气
    Sensor_Data.NH3 = 0xff;//氨气
    Sensor_Data.H2S = 0xff;//硫化氢
    Sensor_Data.greenHouseLux = 0xff;//大棚光照度
    Sensor_Data.greenHouseAtmos = 0xff;//大棚大气压力
    Sensor_Data.greenHouseUV_AirTemp = 0xff;//大棚UV传感器空气温度
    Sensor_Data.greenHouseUV_AirHum = 0xff;//大棚UV传感器空气湿度
    Sensor_Data.greenHouseUV = 0xff;//大棚紫外线
    Sensor_Data.greenHouseCo2 = 0xff;//大棚二氧化碳浓度
    Sensor_Data.greenHouseO2 = 0xff;//大棚氧气浓度
}

void Data_Acquisition(void)
{
      sensorDataInit();
      delay(10);
      char str[100]={0};
      //EC传感器
      //readSoilTempAndHumi(&Sensor_Data.SoilHum,&Sensor_Data.SoilTem,SOIL_SENSOR_ADDR);
      //readCondAndSalt(&Sensor_Data.SoilSod,&Sensor_Data.SoilSat,SOIL_SENSOR_ADDR);
      delay(200);
      memset((SENSOR_DATA_STRUCT*)&Sensor_Data,0x00,sizeof(SENSOR_DATA_STRUCT));
      //UV紫外线传感器
      readUVSensorAirTempAndHumi(&Sensor_Data.greenHouseUV_AirHum,&Sensor_Data.greenHouseUV_AirTemp,UV_HUM_TEMP_SENSOR_ADDR);
      Sensor_Data.greenHouseUV=readuv(UV_HUM_TEMP_SENSOR_ADDR);
      //光照度传感器
      Sensor_Data.greenHouseLux=readIllumination(AMBIENT_LIGHT_SENSOR_ADDR);
      //百叶箱
      readScreenAirTempAndHumi(&Sensor_Data.Screen_AirHum,&Sensor_Data.Screen_AirTemp,ATMOSPHERIC_PRESSURE_SENSOR_ADDR);
      Sensor_Data.Screen_Beam = readScreenBeam(ATMOSPHERIC_PRESSURE_SENSOR_ADDR);
      Sensor_Data.Screen_Atmos = readScreenAtmos(ATMOSPHERIC_PRESSURE_SENSOR_ADDR);
      //风向
      Sensor_Data.WindDirCode=readWindDirection(WIND_DIRECTION_SENSOR_ADDR);
      //风速
      Sensor_Data.WS = readWindSpeed(WIND_RATE_SENSOR_ADDR);
      delay(300);
      //CO2传感器
      Sensor_Data.greenHouseCo2 = readCo2(CO2_PPM_SENSOR_ADDR);
      #ifdef  EX_RTC
      RtcDateTime saveTime = ExRtc.GetDateTime();
      //const saveTime& dt;
      Sensor_Data.curTime.tm_year = saveTime.Year();
      Sensor_Data.curTime.tm_mon = saveTime.Month();
      Sensor_Data.curTime.tm_mday = saveTime.Day();
      Sensor_Data.curTime.tm_hour = saveTime.Hour();
      Sensor_Data.curTime.tm_min = saveTime.Minute();
      Sensor_Data.curTime.tm_sec = saveTime.Second();
      #else
      //使用内部RTC
      //InRtc.getTime(&Sensor_Data.curTime);
      UTCTime CurrentSec=0;
      CurrentSec = InRtc.getTime();
      osal_ConvertUTCTime(&RtcTime,CurrentSec);
      Sensor_Data.curTime.tm_year = RtcTime.year;
      Sensor_Data.curTime.tm_mon = RtcTime.month;
      Sensor_Data.curTime.tm_mday = RtcTime.day;
      Sensor_Data.curTime.tm_hour = RtcTime.hour;
      Sensor_Data.curTime.tm_min = RtcTime.minutes;
      Sensor_Data.curTime.tm_sec = RtcTime.seconds;
      #endif
      
      memset(str,0x00,sizeof(str));
      sprintf(str,"CO2 sensor CO2:%dppm\r\n",Sensor_Data.greenHouseCo2);
      Serial.println(str);
      delay(10);

      memset(str,0x00,sizeof(str));
      sprintf(str,"UV sensor AirHum:%.2f%%RH,AirTemp:%.2f℃,UV:%.2fw/m3\r\n",Sensor_Data.greenHouseUV_AirHum,Sensor_Data.greenHouseUV_AirTemp,Sensor_Data.greenHouseUV);
      Serial.println(str);
      delay(10);

      memset(str,0x00,sizeof(str));
      sprintf(str,"AMBIENT_LIGHT:%ld lux\r\n",Sensor_Data.greenHouseLux);
      Serial.println(str);
      delay(10);

      memset(str,0x00,sizeof(str));
      sprintf(str,"WindDirCode:%d\r\n",Sensor_Data.WindDirCode);
      Serial.println(str);
      delay(10);

      memset(str,0x00,sizeof(str));
      sprintf(str,"Thermometer screen's humi:%.2f%%RH,Thermometer screen's temp:%.2f℃\r\n",Sensor_Data.Screen_AirHum,Sensor_Data.Screen_AirTemp);
      Serial.println(str);
      delay(10);

      memset(str,0x00,sizeof(str));
      sprintf(str,"Atmospheric_Pressure:%.2fkpa",Sensor_Data.Screen_Atmos);
      Serial.println(str);
      delay(10);

      memset(str,0x00,sizeof(str));
      sprintf(str,"Illuminance:%dLux\r\n",Sensor_Data.Screen_Beam);
      Serial.println(str);
      delay(10);

      memset(str,0x00,sizeof(str));
      sprintf(str,"Wind_Speed:%.2fm/s\r\n",Sensor_Data.WS);
      Serial.println(str);
      delay(10);

    //将采集到的数据存入EEPROM中
    saveSensorDataToEEPROM();
}


void saveSensorDataToEEPROM()
{
    Serial.print("EEPROM max record:");
    Serial.println(RECORDS_TO_CREATE);
    EpromDb.open(EEPROM_BASE_ADDR);
    int recno = EpromDb.count();
    if(recno >= RECORDS_TO_CREATE)
    {
      EpromDb.clear();
    }
    EDB_Status result = EpromDb.appendRec(EDB_REC Sensor_Data);
    if (result == EDB_OK) 
    {
        Serial.println("Save a SensorData DONE!");
      //设置下次发送数据的（RTC）内部中断时间
        readEPServerParam();
        time_t nowTime = 0;
        nowTime = InRtc.getTime();
        //SysParamstr.transmit_cycle = 30;
        nowTime += SysParamstr.transmit_cycle; 
        InRtc.setAlarmTime(nowTime);
    }
    else
    {
      nvic_sys_reset();
    }
    delay(100);
}

void SensorDataBaseInit(void)
{    
     int SensorCount=0;
     EDB_Status result1;
     EDB_Status result2;
     EpromDb.open(EEPROM_BASE_ADDR);
     SensorCount=EpromDb.count();
   if(SensorCount==0)
   {
       result1 = EpromDb.create(EEPROM_BASE_ADDR,TABLE_SIZE, (unsigned int)sizeof(Sensor_Data));
   }
   EpromDb.limit();

   EpromDb.open(SYS_PARAM_BASE_ADDR);
   int param =EpromDb.count();
   if(param == 0)
   {
    result2 = EpromDb.create(SYS_PARAM_BASE_ADDR,TABLE_SIZE, (unsigned int)sizeof(SysParamstr));
   }
    else
   {
     
      if(digitalRead(KEY2_INPUT)==LOW)
      {
         EpromDb.clear(); 
         Serial.println("Clear System param!");
      }
      
   }
    EpromDb.limit();

   if(result1==EDB_OK && result2==EDB_OK )
    {
      Serial.println("Table of SD and P create DONE!");
    }
    else if(result1==EDB_OK && result2== EDB_ERROR)
    {
      Serial.println("Table of SD create done,but P create fail!");
    }
    else if(result1==EDB_ERROR && result2== EDB_OK)
    {
      Serial.println("Table of P create done,but SD create fail!");
    }
    else if (result1==EDB_ERROR && result2== EDB_ERROR)
    {
      Serial.println("Table of SD and P both create fail!");
    }
}


bool Alarmed()
{
  bool wasAlarmed = false;
  #ifdef  EX_RTC
  if (interuptFlag)  // check our flag that gets sets in the interupt
  {
    wasAlarmed = true;
    interuptFlag = false; // reset the flag
        
      // this gives us which alarms triggered and
      // then allows for others to trigger again
    DS3231AlarmFlag flag = ExRtc.LatchAlarmsTriggeredFlags();

    if (flag & DS3231AlarmFlag_Alarm1)
    {
      Serial.println("alarm one triggered");
        //采集传感器数据和存储数据
        Data_Acquisition();
        //设置下次采集闹钟
        //RtcDateTime now = ExRtc.GetDateTime();
        //RtcDateTime alarmTime = now + SysParamstr.acquisition_cycle; // into the future
        //DS3231AlarmOne alarm1(alarmTime.Day(),alarmTime.Hour(),alarmTime.Minute(), alarmTime.Second(),DS3231AlarmOneControl_HoursMinutesSecondsMatch);
        //ExRtc.SetAlarmOne(alarm1);
    }
  }
  #endif
  return wasAlarmed;
}

//读取EEPROM里的传感器数据 
void readEPSensorData()
{   
  readEPServerParam();
  int nowRecord = SysParamstr.currentRecord;
  if(nowRecord == 0)
  {
    nowRecord = 1;
  }
  delay(100);
  EpromDb.open(EEPROM_BASE_ADDR);
  int DataCount=EpromDb.count();
  Serial.print("Current SensorData Count:");
  Serial.println(DataCount);
  for(int record=nowRecord; record<=DataCount; record++)
  {   
      EpromDb.open(EEPROM_BASE_ADDR);
      EDB_Status result = EpromDb.readRec(record,EDB_REC Sensor_Data);
      if (result == EDB_OK)
      {
        Serial.print("Recno: "); 
        Serial.println(record);
        Serial.print("Save sensor data time:");
        
        Serial.print(Sensor_Data.curTime.tm_year);
        Serial.print("-");
        Serial.print(Sensor_Data.curTime.tm_mon);
        Serial.print("-");
        Serial.print(Sensor_Data.curTime.tm_mday);
        Serial.print("  ");
        
        Serial.print(Sensor_Data.curTime.tm_hour);
        Serial.print(":");
        Serial.print(Sensor_Data.curTime.tm_min);
        Serial.print(":");
        Serial.println(Sensor_Data.curTime.tm_sec);
        
        //Serial.print("UV sensor air temp: "); 
        //Serial.println(Sensor_Data.UV_AirTemp);
        //Serial.print("UV sensor air humi: "); 
        //Serial.println(Sensor_Data.UV_AirHum);

        if (client.connected())
        { 
         SendSensorDataToServer();//发送传感器数据到服务器
         delay(1000);
         SysParamstr.currentRecord = record;
         saveParamToEP();
         /*
         if(EpromDb.deleteRec(record)==EDB_OK)
         {
           Serial.print("Delete Rec:");
           Serial.println(record);
         }
         */
        }
        else
        {
          client.stop();
          return;
        }
        delay(100);
      }
      else 
      {
         Serial.println(result);
      }
        //发送传感器数据
        //Serial.println("GPRS IS CONNECTING!\r");
        //SendGPSInfo();
  }
  EpromDb.open(EEPROM_BASE_ADDR);
  EpromDb.clear();
  delay(100);
  SysParamstr.currentRecord = 1;
  saveParamToEP();
}         

void saveParamToEP()
{
  EpromDb.open(SYS_PARAM_BASE_ADDR);
  int recno=EpromDb.count();
  if(recno<=0)
  {
    EDB_Status result = EpromDb.appendRec(EDB_REC SysParamstr);
    if (result == EDB_OK) 
    {
      Serial.println("Save a server param  DONE!");
    }
  }
  else
  {
    EpromDb.updateRec(recno, EDB_REC SysParamstr);
    Serial.println("Update a server param  DONE!");
  }
}

void loop()
{
  //判断发送数据中断标志
if (sendFunctionSign == true)
{
  iwdg_feed();
  //清除超时计数
  RunTimeOutSec=0;
  if (GSM_Connect_flag == false)
  {
    Serial.print("Connect net number:");
    Serial.println(numberOfTimeout);
    numberOfTimeout +=1;
    if(numberOfTimeout>3)
    {
      numberOfTimeout=0;
      SYS_Sleep();
    }
    Serial.print(F("Waiting for network..."));
    if (!modem.waitForNetwork())
    {
      iwdg_feed();
      Serial.println("Connect NetWork Fail!");
      //关闭GPRS模块电源
      GPRS_PWR_OFF;
      delay(3000);
      GPRS_PWR_ON;
      delay(1000);
      //digitalWrite(LED2, HIGH);
      GSM_STATUS_LED_ON;
      SIM800_PWR_CON();
      digitalWrite(LED2, LOW);
      GPRS_Connect_flag = false;
      GSM_Connect_flag = false;
      //return;
    }
    else
    {
      Serial.println(" OK");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn, user, pass))
      {
        iwdg_feed();
        Serial.println(" fail");
        delay(2000);
        iwdg_feed();
        //return;
      }
      else
      {
        int CSQ = 0;
        CSQ = modem.getSignalQuality();
        GSM_Connect_flag = true;
        StartConnectSeviceFlag=true;
        //digitalWrite(LED1, HIGH);
    
        GSM_STATUS_LED_ON;
        Server_STATUS_LED_OFF;
        //resetFunc();
        //nvic_sys_reset();
        Serial.println("\nGSM Enter network OK\r");
        char string[25];
        sprintf(string, "CSQ:%d\r", CSQ);
        Serial.println(string);
        String IMEI = modem.getIMEI();
        Serial.println("IMEI:" + IMEI);
        //Serial.print(CSQ,DEC);
        Serial.println("\r");
        String SIMCCID = modem.getSimCCID();
        Serial.println("SIMCCID:" + SIMCCID);

        //modem.sendSMS("+8613577182976", "this is SIM868");
        //String Pos=modem.getGsmLocation();
        //Serial.println("GSM LOC:"+Pos);
        //连接基站定位服务连接基站定位服务
        if(SysParamstr.positioning_mode != 'U' )
      {
        if (modem.LBS_Connect())
        {
          iwdg_feed();
          if (LBS_Connect_Seriver_flag == false)
          {
            LBS_Connect_Seriver_flag = true;
          }
          Serial.println("LBS Connect OK!");
          //0,102.654510,25.064799,550,17/09/14,11:20:30
          if(LBS_Location_TryNum<3)
          {
              String LOCData = modem.Get_LOCData();
              //判断返回的字符串是否有效
              if (LOCData.length()>20)
              {
                //解析字符串
                LBS_Location_TryNum=0;
                lbs_parse(LOCData, &GPS_Store_Data);
                RtcTime.year=GPS_Store_Data.D.year;
                RtcTime.month= (GPS_Store_Data.D.month);
                RtcTime.day=(GPS_Store_Data.D.day);
                RtcTime.hour= (GPS_Store_Data.D.hour);
                RtcTime.minutes=(GPS_Store_Data.D.minutes);
                RtcTime.seconds=(GPS_Store_Data.D.seconds);
                UTCTime CurrentSec=osal_ConvertUTCSecs(&RtcTime);
                //osal_ConvertUTCTime();
                //发送GPS数据
                InRtc.setTime(CurrentSec);
              }
              //如果LBS返回的字符串无效，说明定位不成功
              else
              {
                //重新连接网络
                GSM_Connect_flag=false;
                LBS_Location_TryNum++;
              }
              //发送GPS数据
              SendGPSInfo();
              LOCData=modem.getGsmLocation();
              Serial.println("LBS Data:" + LOCData + "\r\n");
           }
        }
        else
        {
          Serial.println("LBS Connect Fail!");
        }
       }
      }
    }
  }
  
  if (GPRS_Connect_flag == false && GSM_Connect_flag == true && StartConnectSeviceFlag==true)
  {
    Serial.print(F("Connecting to "));
    Serial.print(server);
    Serial.print(":");
    Serial.print(port);
    if (!client.connect(server, port))
    {
      Serial.println("\nConnect to server fail!");
      delay(2000);
      GPRS_Connect_Server_Fail_Num++;
      if (GPRS_Connect_Server_Fail_Num > 1)
      {
        GPRS_Connect_Server_Fail_Num = 0;
        if (GSM_Connect_flag == true)
        {
          GSM_Connect_flag = false;
          digitalWrite(LED1, LOW);
          modem.gprsDisconnect();
          delay(200);
        }
      }
      //return;
    }
    else
    {
      GPRS_Connect_flag = true; 
      GPRS_Connect_Server_Fail_Num = 0;
      Serial.println("Connect Serive OK!");
      GSM_STATUS_LED_OFF;
      Server_STATUS_LED_ON;

      readEPServerParam();
      //digitalWrite(LED2, HIGH);
      // FE C0 11 00 00 00 01 00 0D  //指令码：0x01,请求时间，0x02,请求传感器列表
      bool Endfalg=true;
      numberOfTimeout = 0;
      //请求服务器分配用户终端编号
     // for(int n=1;n<=sizeof(SysParamstr.HostUserID);n++)
     if(SysParamstr.HostUserID[0] == 0 && SysParamstr.HostUserID[1] == 0 &&
     SysParamstr.HostUserID[2] == 0 && SysParamstr.HostUserID[3] == 0)
      {
       // if(SysParamstr.HostUserID[n] == 0)
   
          Serial.println("HostUserID is zero,requesting server provide hostUserID");
          //如果设备编号没有被设置，需要不断向服务器发送请求设置编号的指令，此处指令格式自行定义
          
          
        String hostUserID;
        RequestID = true;

     while (Endfalg==true)
      {
        while(RequestID == true)
        {
         while(client.connected())
         {
            GPRS_Connect_flag=true;
            //请求服务器分配用户终端编号
            client.write(SendBuf, Sindex);
            RequestID = false;
            // Print available data
            if (client.available()>0)
            {
              char a = client.read();
              hostUserID += a;
              if (a <= 0)
                continue; // Skip 0x00 bytes, just in case
              if (hostUserID.endsWith("#USERNUM:"))
              {
                String ID = client.readStringUntil('\n');
                char Idstr[5];
                ID.toCharArray(Idstr,5);
                SysParamstr.HostUserID[0]=Idstr[0]-'0';
                SysParamstr.HostUserID[1]=Idstr[1]-'0';
                SysParamstr.HostUserID[2]=Idstr[2]-'0';
                SysParamstr.HostUserID[3]=Idstr[3]-'0';
                Serial.println("Host user ID:" + ID);
                Endfalg=false;
                //将参数存到EEPROM中
                saveParamToEP();
                break;
              }
                iwdg_feed(); 
            }
         }
         if(!client.connected())
         {
            if(GPRS_Connect_flag==true)
            {
          // digitalWrite(LED2,LOW);
            Server_STATUS_LED_OFF;
            GPRS_Connect_flag=false;
            Endfalg=false;
            RequestID = false;
            client.stop();
            }
         }
        numberOfTimeout +=1;
        if(numberOfTimeout > 3)
        {
          numberOfTimeout =0;
          client.stop();
          delay(1000);
          SYS_Sleep();
        }
      }
    }    
  }

  if(GPRS_Connect_flag==true)
  {
      //和服务器建立TCP/IP成功后发送握手帧,我们的数据已删除，此处指令格式自行定义
    

      //发送请求时间指令
      numberOfTimeout =0;
    do{
        if(client.connected())
        {
          client.write(SendBuf, Sindex);
        }
        else
        {
            if(GPRS_Connect_flag==true)
            {
          // digitalWrite(LED2,LOW);
            Server_STATUS_LED_OFF;
            GPRS_Connect_flag=false;
            client.stop();
            }
        }
        numberOfTimeout +=1;
        if(numberOfTimeout > 3)
        {
            numberOfTimeout=0;
            client.stop();
            delay(1000);
            SYS_Sleep();   
        } 
      //client.print("SIM868,heart!");
      unsigned long timeout = millis();
      String RtcData;
      Endfalg=true;
      ReciveParamFlag=false;
      while (client.connected() && millis() - timeout < 10000L && Endfalg==true)
      {
        // Print available data
        while (client.available())
        {
          char c = client.read();
          RtcData += c;
          if (c <= 0)
            continue; // Skip 0x00 bytes, just in case
            //接收服务器发送的设置参数，格式为#PARAM:xxxxxxxx
          if (RtcData.endsWith("#PARAM:"))
          {
            String DataTime = client.readStringUntil('\n');
            String strTemp;
            char Datebuf[11]={0};
            Serial.println("Server RTC:" + DataTime);
            //应答帧#PARAM:2018-01-17,15:01:57,120,300,O,L\n
            //日期字符串
            strTemp=DataTime.substring(0,DataTime.indexOf(","));
            strTemp.toCharArray(Datebuf,11);
            //Serial.write(Datebuf,9);
            RtcTime.year=(Datebuf[0]-'0')*1000+(Datebuf[1]-'0')*100+(Datebuf[2]-'0')*10+(Datebuf[3]-'0');
            RtcTime.month=(Datebuf[5]-'0')*10+(Datebuf[6]-'0');
            RtcTime.day=(Datebuf[8]-'0')*10+(Datebuf[9]-'0');
            Serial.println("Server Date:"+strTemp);

            //时间字符串
            DataTime = DataTime.substring(DataTime.indexOf(",")+1);
            strTemp=DataTime.substring(0,DataTime.indexOf(","));
            memset(Datebuf,0x00,sizeof(Datebuf));
            strTemp.toCharArray(Datebuf,11);
            Serial.println("Server Time:"+strTemp);
            RtcTime.hour=(Datebuf[0]-'0')*10+(Datebuf[1]-'0');
            RtcTime.minutes=(Datebuf[3]-'0')*10+(Datebuf[4]-'0');
            RtcTime.seconds=(Datebuf[6]-'0')*10+(Datebuf[7]-'0');
            UTCTime CurrentSec=osal_ConvertUTCSecs(&RtcTime);
           // osal_ConvertUTCTime(&RtcTime,CurrentSec);
           UTCTime RtcSec = InRtc.getTime();
           InRtc.setTime(CurrentSec);
           if(RtcSec<(CurrentSec-SysParamstr.transmit_cycle)||RtcSec>(CurrentSec+SysParamstr.acquisition_cycle));
           {  
              RtcSec = InRtc.getTime();
              RtcSec += SysParamstr.transmit_cycle; 
              InRtc.setAlarmTime(RtcSec);
              Serial.println("RTC time Error!Reset Alarm!");
           }
            CurrentSec=InRtc.getTime();
            osal_ConvertUTCTime(&RtcTime,CurrentSec);
            char str[50];
            sprintf(str,"%04ld-%02ld-%02ld,%02ld:%02ld:%02ld\r\n",RtcTime.year,RtcTime.month,RtcTime.day,RtcTime.hour,RtcTime.minutes,RtcTime.seconds);
            Serial.println(str);

            //采集周期
            DataTime = DataTime.substring(DataTime.indexOf(",")+1);
            strTemp=DataTime.substring(0,DataTime.indexOf(","));
            SysParamstr.acquisition_cycle=strTemp.toInt();
            Serial.println("Acquisition cycle:"+strTemp);
            //SysParamstr.acquisition_cycle = 20;

            //发送周期
            DataTime = DataTime.substring(DataTime.indexOf(",")+1);
            strTemp=DataTime.substring(0,DataTime.indexOf(","));
            SysParamstr.transmit_cycle=strTemp.toInt();
            Serial.println("Transmit cycle:"+strTemp);
            //SysParamstr.transmit_cycle = 30;

            //运行模式
            DataTime = DataTime.substring(DataTime.indexOf(",")+1);
            strTemp=DataTime.substring(0,DataTime.indexOf(","));
            SysParamstr.run_mode=strTemp.toInt();
            Serial.println("Run mode:"+strTemp);

            //定位模式
            DataTime= DataTime.substring(DataTime.indexOf(",")+1);
            strTemp=DataTime.substring(0,DataTime.indexOf(","));
            SysParamstr.positioning_mode=strTemp.toInt();
            Serial.println("Positioning mode:"+strTemp);

            //将参数存到EEPROM中
            saveParamToEP();
            /*
            String BleCommand;
            BleCommand += BLE_RTC_SET_OR_QUEST;
            BleCommand += DataTime;
            char UserRtcString[30] = {0};
            BleCommand.toCharArray(UserRtcString, BleCommand.length() + 1);
            Serial.println(UserRtcString);
            */
            SendDataFlag=true;
            Endfalg=false;
            ReciveParamFlag=true;
            break;
          }
          //Serial.print(c);
          timeout = millis();
          iwdg_feed();
        }
      }
    }while(ReciveParamFlag==false);
   } 
  }
}

  ///modem.GPS_Connect();
  // Make a HTTP GET request:
  //client.print(String("GET ") + resource + " HTTP/1.0\r\n");
  //client.print(String("Host: ") + server + "\r\n");
  //client.print("Connection: close\r\n\r\n");
  //client.print("I am sim868,hello,world!");
 

    if(client.connected() && GPRS_Connect_flag==true)
    {    
        numberOfTimeout =0;
        readEPSensorData();//读取EEPROM中的传感器数据和发送数据
        SendDataFlag = false;
        client.stop();
        delay(1000);
        GPRS_Connect_flag=false;
        StartConnectSeviceFlag=false;
        GSM_Connect_flag =  false;
        sendFunctionSign = false;
        digitalWrite(LED2,LOW);
        Server_STATUS_LED_OFF;
        GSM_STATUS_LED_OFF;
        SYS_Sleep();//休眠
    }
    else
    {
      if(GPRS_Connect_flag==true)
      {
     // digitalWrite(LED2,LOW);
      Server_STATUS_LED_OFF;
      GPRS_Connect_flag=false;
      client.stop();
      }
    }
  }
 }
}

static void SendSensorDataToServer(void)
{
  unsigned char HiByte, LoByte, flag;
  char intstr[15];
  unsigned int Slen = 0;
  unsigned char NumOfDot = 0;
  unsigned char Data_BCD[4] = {0};
  char weathertr[20] = {0};
  //读电池电量
  unsigned int batAdc = analogRead(HOST_VOL_ADC_INPUT_PIN);
  sensorValue = (float)batAdc * ADC_RATE;
  sensorValue *= VBAT_DIVIDER_RATIO;
  char string[50];
  sprintf(string, "BatV:%02fmv\r", sensorValue);
  Serial.println(string); 
  //模拟电压输入通道1
  u16 AnalogADC1=analogRead(A_5V_20mA_CANNEL1_PIN);
  u16 AnalogADC2=analogRead(A_5V_20mA_CANNEL2_PIN);
  u16 AnalogADC3=analogRead(A_5V_20mA_CANNEL3_PIN);

  u16 AnalogVol[3]={0};
  AnalogVol[0]=((float)AnalogADC1 * ADC_RATE)*ANALOG_DIVIDER_RATIO;
  AnalogVol[1]=((float)AnalogADC2 * ADC_RATE)*ANALOG_DIVIDER_RATIO;
  AnalogVol[2]=((float)AnalogADC3 * ADC_RATE)*ANALOG_DIVIDER_RATIO;

  for(unsigned char i=0;i<3;i++)
  {
     memset(string,0x00,sizeof(string));
     sprintf(string, "Analog Input%d:%02dmv\r", i+1,AnalogVol[i]);
     Serial.println(string); 
     delay(500);
  }
  
  //BLE_Serial.println(string);
  float P = (AnalogVol[0] / VCC - 0.1) / 0.08;
  int KP = P * 1000;
  memset(string, 0x00, 25);
  //sprintf(string, "Pressue:%dKPa\r", KP);
  //Serial.println(string);
  //BLE_Serial.println(string);

  ToBCD(KP, &HiByte, &LoByte, &flag);
  //SendBatVoltage(batAdc,0);
  //把要发送给服务器的数据填到Buf,格式自己定义。
  Sindex = 0;
  SendBuf[Sindex++] = 1;
  SendBuf[Sindex++] = 3;
  SendBuf[Sindex++] = 3;
 
    //client.print(SendBuf);
    client.write(SendBuf, Sindex);
}

/******************************************************************************
 * @fn          sendVoltageWarning
 *
 * @brief       发送簇点的虚拟电量
 *
 * @param       BatCh,电压通道，EX_BAT_ADC_CH-外部电池电压，IN_BAT_ADC_CH，内部电池电压
                currentVoltage-当前采信到的电压值
                IsAlarmFlag-电量报警码
 *
 * @return      voltage
 */
static void SendBatVoltage(u16 currentVoltage, u8 IsAlarmFlag)
{
  u8 pFrame[16];
  u8 SendIndex = 0;
  pFrame[SendIndex++] = 0xFE;
  if (IsAlarmFlag == 0)
  {
    pFrame[SendIndex++] = BAT_VOL_ID;
    pFrame[SendIndex++] = 0x0B;
  }
  else
  {
    pFrame[SendIndex++] = BAT_VOL_ALARM_ID;
    pFrame[SendIndex++] = 0x0C;
  }
  //Eeprom_read((u8*)UsrNum,USR_NUM_ADDR,2);
  pFrame[SendIndex++] = HostUserID[2];
  pFrame[SendIndex++] = HostUserID[3];
  //如果报警，发送报警码
  if (IsAlarmFlag != 0)
  {
    pFrame[SendIndex++] = IsAlarmFlag;
  }

  pFrame[SendIndex++] = HI_UINT16(currentVoltage);
  pFrame[SendIndex++] = LO_UINT16(currentVoltage);
  pFrame[SendIndex++] = HI_UINT16(GPS_Store_Data.D.year);
  pFrame[SendIndex++] = LO_UINT16(GPS_Store_Data.D.year);
  pFrame[SendIndex++] = GPS_Store_Data.D.month;
  pFrame[SendIndex++] = GPS_Store_Data.D.day;
  pFrame[SendIndex++] = GPS_Store_Data.D.hour;
  pFrame[SendIndex++] = GPS_Store_Data.D.minutes;
  pFrame[SendIndex++] = GPS_Store_Data.D.seconds;
  pFrame[SendIndex++] = 0x00;
  pFrame[SendIndex++] = 0x0d;
  if (client.connected())
  {
    //client.print(SendBuf);
    client.write(pFrame, SendIndex);
  }
}