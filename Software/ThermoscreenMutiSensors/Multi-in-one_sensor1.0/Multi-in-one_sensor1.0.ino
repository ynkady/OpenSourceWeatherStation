
/*
 *目前该板子上有温湿度检测、光照强度检测、大气压检测。
 *ModBus指令完全兼容《485 型气象百叶箱.pdf》手册
 *使用软串口收发指令，波特率：9600（可根据485总线长度来适当修改，保证传输数据的稳定性）
 *返回的传感器数据中有小数精度要求的是：温度、湿度。精度为0.1。举例：解析的湿度十进制为485，因为精度是0.1，所以实际值应该为：48.5（主机解析ModBus数据时候需要注意）
*/



#include <AsyncDelay.h>
//#include <Tsl2561Util.h>
#include <SoftwareSerial.h>
#include "SoftwareI2C.h"
#include "HTU21D.h"
#include "bmp180.h"
#include "SparkFunCCS811.h"
#include <EEPROM.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_TSL2561_U.h"


/*
 *ModBus总线协议通信的一些全局变量
 */

//系统参数
#define bufferSize 255  //一帧数据的最大字节数量
unsigned char frame[bufferSize];  //用于保存接收的数据
unsigned char frame_back[bufferSize]; //用于保存发送的数据
unsigned char crc[2]; //保存接收的两位CRC校验位
unsigned char sensor_value[28]; //保存传感器等信息，发送给主机
unsigned char slaveID; //定义modbus RTU从站站号
int baudrate;  //定义通信波特率
unsigned char baudrate_value;

//函数声明
void modbusRTU_slave();  //声明modbus RTU从站函数
unsigned int calculateCRC(unsigned char* _regs,unsigned char arraySize);  //声明校验函数





/*
 *bmp180传感器的温度、气压、海拔全局变量
 */
float temperature;  
double pressure;   
double pressure2;     
double altitude;



/*
 *软串口引脚定义
 *485读写使能脚定义
 */
SoftwareSerial mySerial(2, 3);
 #define EN485_DE     4

int ReadUVintensityPin = A3; //Output from the sensor
/*
 *创建温湿度对象
 */
HTU21D myHumidity;


/*
 *创建光照强度对象
 *启动硬件I2C
 */
//Tsl2561::address_t addr[] = { Tsl2561::ADDR_GND, Tsl2561::ADDR_FLOAT, Tsl2561::ADDR_VDD };
//Tsl2561 Tsl(Wire);

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_HIGH, 12345);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void TSL_displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void TSL_configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  //tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

/*
 *CCS811的I2C地址
 *创建CCS811对象
 */
//#define CCS811_ADDR 0x5A
//CCS811 CCS811_Sensor(CCS811_ADDR);


/*
 *RS485读写使能函数
 */
void enable_modbus(void)
{
  digitalWrite(EN485_DE, HIGH);
  delay(2);
}
void disable_modbus(void)
{
  digitalWrite(EN485_DE, LOW);
  delay(2);
}

/*
 *bmp180的气压、温度、海拔运算函数
 */
void bmp_calculate()
{
  temperature = bmp180GetTemperature(bmp180ReadUT());
  temperature = temperature*0.1;
  pressure = bmp180GetPressure(bmp180ReadUP());
  pressure2 = pressure/101325;
  pressure2 = pow(pressure2,0.19029496);
  altitude = 44330*(1-pressure2);                            //altitude = 44330*(1-(pressure/101325)^0.19029496);
}



/*
 *发送光照强度信息（ModBus协议格式）
 */
void Send_illumination(void)
{
  uint32_t Tsl_lux;
  sensors_event_t event;
  tsl.getEvent(&event);
  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    //Serial.print(event.light); 
    Tsl_lux=(uint32_t)(event.light*2.0);
    Serial.print(Tsl_lux);
    Serial.println(" lux");
     byte Tsl_lux_high1 =  Tsl_lux >> 24;
     byte Tsl_lux_low1  = (Tsl_lux & 0x00ff0000) >> 16;
     byte Tsl_lux_high2 = (Tsl_lux & 0x0000ff00) >> 8;
     byte Tsl_lux_low2  = Tsl_lux & 0x000000ff; 

     sensor_value[14] = Tsl_lux_high1;
     sensor_value[15] = Tsl_lux_low1;
     sensor_value[16] = Tsl_lux_high2;
     sensor_value[17] = Tsl_lux_low2;
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }
  /*
  bool found = false;
  uint16_t scaledFull = 0, scaledIr;
  uint32_t Tsl_lux, ir, milliLux;
  uint8_t id;
  bool gain;
  Tsl2561::exposure_t exposure;
  for( uint8_t i = 0; i < sizeof(addr)/sizeof(addr[0]); i++ ) {
    if( Tsl.begin(addr[i]) ) {
      found = true;


  for( uint8_t g=0; g<2; g++ ) {
    gain = g;
    for( uint8_t e=0; e<3; e++ ) {
      exposure = (Tsl2561::exposure_t)e;

      Tsl.on();

      Tsl.setSensitivity(gain, exposure);
      Tsl2561Util::waitNext(exposure);
      Tsl.id(id);
      Tsl.getSensitivity(gain, exposure);
      Tsl.Tsl_luxLuminosity(scaledFull);
      Tsl.irLuminosity(scaledIr);

      Tsl.off();
    }
  }
  }
  }
  if( Tsl2561Util::normalizedLuminosity(gain, exposure, Tsl_lux = scaledFull, ir = scaledIr) ) {
          if( Tsl2561Util::milliLux(Tsl_lux, ir, milliLux, Tsl2561::packageCS(id)) ) {
           byte Tsl_lux_high1 = Tsl_lux >> 24;
           byte Tsl_lux_low1  = (Tsl_lux & 0x00ff0000) >> 16;
           byte Tsl_lux_high2 = (Tsl_lux & 0x0000ff00) >> 8;
           byte Tsl_lux_low2  = Tsl_lux & 0x000000ff; 

           sensor_value[14] = Tsl_lux_high1;
           sensor_value[15] = Tsl_lux_low1;
           sensor_value[16] = Tsl_lux_high2;
           sensor_value[17] = Tsl_lux_low2;
          }
  }
  */
}


/*
 *发送温度信息（ModBus协议格式）
 */
 
void Send_Temperature(void)
{
  float temp = myHumidity.readTemperature();
  Serial.print("temp:");
  Serial.print(temp);
  Serial.println("du");
  temp *= 10;
  int temp_value = temp;
  unsigned char temp_high = temp_value >> 8;
  unsigned char temp_low = temp_value & 0xff;
  sensor_value[2] = temp_high;
  sensor_value[3] = temp_low;

}

/*
 *发送湿度信息（ModBus协议格式）
 */

void Send_Humidity(void)
{
   float humd = myHumidity.readHumidity();
  Serial.print("hum:");
  Serial.print(humd);
  Serial.println("%");
   humd *= 10;
   int humd_value = humd;
   unsigned char humd_high = humd_value >> 8;
   unsigned char humd_low  = humd_value & 0xff;
   sensor_value[0] = humd_high;
   sensor_value[1] = humd_low;
}

/*
 *发送大气压信息（ModBus协议格式）
 */
void Send_Pressure(void)
{
  bmp_calculate();  
  long int pressure_value = pressure;
  Serial.print(pressure_value);
  Serial.println("pa");
  unsigned char pressure_high1 = pressure_value >> 24;
  unsigned char pressure_low1 = (pressure_value & 0x00ff0000) >> 16;
  unsigned char pressure_high2 = (pressure_value & 0x0000ff00) >> 8;
  unsigned char pressure_low2 = pressure_value & 0x000000ff;
  sensor_value[20] = pressure_high1;
  sensor_value[21] = pressure_low1;
  sensor_value[22] = pressure_high2;
  sensor_value[23] = pressure_low2;
  
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  

}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
 *发送紫外线的值
 */

void Send_ultraviolet(void)
{
  /*
  int ultraviolet_value;
  int uv;
  ultraviolet_value = analogRead(A3);
  ultraviolet_value = ultraviolet_value * 4980.0 / 1024;
  Serial.print("Uv Vol:");
  Serial.print(ultraviolet_value);
  Serial.println("mv");
  */
  int uvLevel = averageAnalogRead(ReadUVintensityPin);
  float outputVoltage = 3.3 * uvLevel/1024;
  Serial.print("Uv out vol:");
  Serial.print(outputVoltage*1000);
  Serial.println("mv");
  float uvIntensity = mapfloat(outputVoltage, 0.1, 2.9, 0.0, 15.0);
  Serial.print("Uv:");
  Serial.print(uvIntensity);
  Serial.println("mw/cm2");
  int uv;
  /*
  if (ultraviolet_value < 50){
      uv = 0;
    }
  else if (ultraviolet_value >= 50 && ultraviolet_value < 227){
      uv = 1;
    }
  else if (ultraviolet_value >= 227 && ultraviolet_value < 318){
      uv = 2;
    }
  else if (ultraviolet_value >= 318 && ultraviolet_value < 408){
      uv = 3;
    }
  else if (ultraviolet_value >= 408 && ultraviolet_value < 503){
      uv = 4;
    }
  else if (ultraviolet_value >= 503 && ultraviolet_value < 606){
      uv = 5;
    }
  else if (ultraviolet_value >= 606 && ultraviolet_value < 696){
      uv = 6;
    }
  else if (ultraviolet_value >= 696 && ultraviolet_value < 795){
      uv = 7;
    }
  else if (ultraviolet_value >= 795 && ultraviolet_value < 881){
      uv = 8;
    }
  else if (ultraviolet_value >= 881 && ultraviolet_value < 976){
      uv = 9;
    }
  else if (ultraviolet_value >= 976 && ultraviolet_value < 1079){
      uv = 10;
    }
  else{
      uv = 11;
    }
   */
    uv=(int)(uvIntensity*10);
	if(uv<0)
	{
		uv=0;
	}
  Serial.print("Uv1:");
  Serial.print(uv);
  Serial.println("mw/cm2");
  
    sensor_value[26] = uv >> 8;
    sensor_value[27] = uv & 0xff;
}

/*
 *发送CO2浓度信息
 *目前板子上没有集成该传感器
 */
//void Send_CO2(void)
//{
//	int CO2_value;
//	if (CCS811_Sensor.dataAvailable())
//	{
//		CCS811_Sensor.readAlgorithmResults();
//
//	
//		CO2_value = CCS811_Sensor.getCO2();
//
//		sensor_value[10] = CO2_value >> 8;
//		sensor_value[11] = CO2_value & 0xff;
//	}
//}

/*
 *发送海拔信息
 *（这里海拔没用到）
 */
void Send_Altitude(void)
{
  bmp_calculate();
}


/*
 *ModBus根据接收的命令来组织成一组ModBus格式数据（包含从机地址码、功能码、n个有效数据位、n个数据、两个CRC校验码）
 *然后通过485总线返回给主机分析
 */
void Send_ModBus(void)
{
  unsigned char start_addr, end_addr, valid_addr;
  unsigned char back_value = 3;
  unsigned int crc16_value;
  
  slaveID = EEPROM.read(0);
  
  if (frame[0] == slaveID && frame[1] == 3 && frame[5] != 0 &&  frame[6] == crc[0] && frame[7] == crc[1]  ) 
  {
    unsigned char reg_len = frame[5] * 2;
    start_addr = frame[3] * 2;
    end_addr = start_addr + reg_len;
    valid_addr = end_addr - start_addr;

    frame_back[0] = frame[0];
    frame_back[1] = frame[1];
    frame_back[2] = valid_addr;
    
    for (int i = start_addr; i < end_addr; i++)
    {
      frame_back[back_value] = sensor_value[i];
      back_value++;
      }
    crc16_value = calculateCRC(&frame_back[0], back_value);

    frame_back[back_value] = crc16_value >> 8;
    back_value += 1;
    frame_back[back_value] = crc16_value & 0x00ff;

    enable_modbus();
    mySerial.write(&frame_back[0], back_value + 1);
    disable_modbus();
  }
}

void modify_slaveID(void)
{
  unsigned char slaveID_temp;
  unsigned char i;
  int crc16_ID;
  unsigned char frame_ID[8];
  unsigned char send_ID_flag = 0;

  slaveID = EEPROM.read(0);

  if (frame[0] == slaveID && frame[1] == 6  && frame[2] == 1 && frame[3] == 0 && frame[5] <= 255 && frame[6] == crc[0] && frame[7] == crc[1]){
    
      slaveID_temp = EEPROM.read(0);
      slaveID = frame[5];

      frame_ID[0] = slaveID_temp;
      for (i = 1; i < 6; i++){
          frame_ID[i] = frame[i];
        }
      crc16_ID = calculateCRC(&frame_ID[0], 6);
      frame_ID[6] = crc16_ID >> 8;
      frame_ID[7] = crc16_ID & 0xff;

      send_ID_flag = 1;

      
      if (send_ID_flag == 1){
      enable_modbus();
      mySerial.write(&frame_ID[0], 8);
      disable_modbus();

      send_ID_flag = 0;
      }
    }
    
    EEPROM.write(0, slaveID);
}

void modify_baudrate(void)
{
  int crc16_baudrate;
  unsigned char frame_baudrate[8];
  char baudrate_flag = 0, i;
  unsigned char baudrate_temp;
  
  slaveID = EEPROM.read(0);
  
  if (frame[0] == slaveID && frame[1] == 6 && frame[2] == 1 && frame[3] == 1 && frame[5] <= 8 && frame[6] == crc[0] && frame[7] == crc[1]){

  for (i = 0; i < 6; i++){
    frame_baudrate[i] = frame[i];
  }
  crc16_baudrate = calculateCRC(&frame_baudrate[0], 6);
  frame_baudrate[6] = crc16_baudrate >> 8;
  frame_baudrate[7] = crc16_baudrate & 0xff;
    

  enable_modbus();
  mySerial.write(&frame_baudrate[0], 8);
  disable_modbus();

  EEPROM.write(1, frame[5]);

  for (i = 0; i < 8; i++){
    frame[i] = 0;
  }
  }

    
}

void setup() {

  baudrate_value = EEPROM.read(1); 
  switch (baudrate_value){
  case 1: baudrate = 4800;    break;
  case 2: baudrate = 9600;    break;
  case 3: baudrate = 14400;   break;
  case 4: baudrate = 19200;   break;
  case 5: baudrate = 38400;   break;
  case 6: baudrate = 56000;   break;
  case 7: baudrate = 57600;   break;
  case 8: baudrate = 115200;  break;
}
  //软串口设置
  mySerial.begin(baudrate);
  //软串口监听
  mySerial.listen();

//  //硬串口设置
  Serial.begin(9600);

  //硬I2C设置
  //Wire.begin();
    /* Initialise the sensor */
  //use tsl.begin() to default to Wire, 
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
  if(!tsl.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Display some basic information on this sensor */
  TSL_displaySensorDetails();
  /* Setup the sensor gain and integration time */
  TSL_configureSensor();
  

  //485读写引脚配置输出模式
  pinMode(EN485_DE, OUTPUT);
  //上电初始化485等待读
  disable_modbus();

  //A3紫外线强度引脚输入模式
  //pinMode(A3,INPUT) ;
  pinMode(ReadUVintensityPin, INPUT);
  analogReference(EXTERNAL);
  //开始温湿度I2C
  myHumidity.begin();

  //开始bmp I2C
  Bmp_begin();
  OSS = 2;  // Oversampling Setting           0: single    1: 2 times    2: 4 times   3: 8 times 

  //开始bmp读取
  BMP180start();

  //开始CCS811 I2C
//  CCS811_Sensor.begin();


}


void loop() {
  
  modbusRTU_slave();  //获取主机ModBus指令函数

  modify_slaveID();

  modify_baudrate();

  /*
   *各个传感器获取数据函数
   */
  Send_Humidity();
  Send_illumination();
  Send_Temperature();
  Send_Humidity();
  Send_Pressure();
  Send_Altitude();
  Send_ultraviolet();
}




/*
 *接收处理主机发来的ModBus命令
 */
void modbusRTU_slave()
{
  unsigned int characterTime; //字符时间
  unsigned int crc16;  //校验位
  unsigned char address = 0;
  unsigned char send_flag = 0;

  if (baudrate > 19200)  //波特率大于19200时进入条件
  {
    characterTime = 750; 
  }
  else
  {
    characterTime = 15000000/baudrate;  //1.5字符时间
  }
  while(mySerial.available()>0)  //如果串口缓冲区数据量大于0进入条件
  {
    if(address<bufferSize)  //接收的数据量应小于一帧数据的最大字节数量
    {
      frame[address]=mySerial.read();
      address++;
    }
    else  //条件不满足时直接清空缓冲区
    {
       mySerial.read();
    }
    delayMicroseconds(characterTime);  //等待1.5个字符时间
    if(mySerial.available()==0)  //1.5个字符时间后缓冲区仍然没有收到数据,认为一帧数据已经接收完成,进入条件
    {    
      unsigned char function = frame[1];
      slaveID = EEPROM.read(0);
      if(frame[0]==slaveID||frame[0]==0)  //站号匹配或者消息为广播形式,进入条件
      {
        crc16 = ((frame[6] << 8) | frame[7]); //得到校验码（两个8位，组成一个16位）
        if(calculateCRC(&frame[0],6)==crc16)  //数据校验通过,进入条件
        {
          if (frame[0]!=0 && (function == 3) || (function == 6))  //功能码03不支持广播消息
          {    
              crc[0] = calculateCRC(&frame[0],6) >> 8;
              crc[1] = calculateCRC(&frame[0],6) & 0xFF;
          }
        }

      }
    }

    send_flag = 1;
  }

  if (!(mySerial.available()>0) && send_flag == 1)
  {
      Send_ModBus();
      send_flag = 0;
    }
}

/*
 *CRC校验函数
 *参数1:待校验数组的起始地址
 *参数2:待校验数组的长度
 *返回值CRC校验结果,16位,低字节在前
 */
unsigned int calculateCRC(unsigned char* _regs,unsigned char arraySize) 
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < arraySize; i++)
  {
    temp = temp ^ *(_regs+i);
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF; 
  return temp; 
}



