
//************************************************************
//实时时钟SD2400读写C51演示程序
//              chendw@whwave.com.cn    2009/01/15
//************************************************************
// To be used with
//#pragma code

#include "sd2400v2.h"
//#include "hal_IO_I2C.h"
//#include "hardware.h"
//#include "device.h"
//#include "hal_uart.h"
#define COUNT_DOWN_TIME_SR   0x13
//RTC中断源
#define Dis_Int    0x00
#define RTC_Alarm  0x01
#define frequency  0x02
#define Count_down 0x03
//时间报警允许寄存器设置
#define EAY  (1<<6)
#define EAMO (1<<5)
#define EAD  (1<<4)
#define EAW  (1<<3)
#define EAH  (1<<2)
#define EAMN (1<<1)
#define EAS  (1<<0)

/* Ports 测试用*/
#define HAL_UART_PORT_0   0x00
#define HAL_UART_PORT_1   0x01
#define HAL_UART_PORT_MAX 0x02

unsigned char   data1,data2,data3,data4;
unsigned char   date[7];   //日期数组
                   /*date[6]=year,date[5]=month,date[4]=day,date[3]=week,
                     date[2]=hour,date[1]=minute,date[0]=second*/

/********SD2400函数名********/
void RTC_GPIO_INI(void);
bool I2CSendByte(unsigned char I2cData); 
unsigned char I2CReceiveByte(void); 
bool I2CReadDate(void);
void I2CWriteStatus(void);
void Delay(unsigned int nn);
bool  I2CWriteDate(void);
bool  WriteTimeOn(void);
bool  WriteTimeOff(void);
bool  SetRtcTime(UTCTimeStruct *Time);
bool  GetRtcTime(UTCTimeStruct *Time);
bool  SetAlarmOutTime(UTCTimeStruct *Time);
bool  GetAlarmOutTime(UTCTimeStruct *Time);
bool  ReadUserData(unsigned char Offset,unsigned char *buf,unsigned char len);
bool  SaveUserData(unsigned char Offset,unsigned char *buf,unsigned char len);
bool RTC_Set_CountdownIn(unsigned char Time,unsigned char unit);
bool  Rtc_Intterupt_Set(unsigned char IntSource);
bool  ClearAlarmOut(void);
void ReadRegs(void);
unsigned char ReadOneByte(unsigned char addr);
unsigned char  ucharToBcd(unsigned char vaule);
unsigned char BcdTouchar(unsigned char vaule);
void localtime_h(uint32 time, UTCTimeStruct *ret_time);
 void ConvSecTimeToCalendar(UTCTimeStruct *t_tm,uint32 t);

 TwoWire RTCWire(RTC_SCL,RTC_SDA,SOFT_FAST);
 

 
//---  微秒级延时--------------------------   
void Delayus(void)  //16MHZ
{    
    /*
    asm("nop"); //一个asm("nop")函数经过示波器测试代表100ns   
    asm("nop");   
    asm("nop");   
    asm("nop");  
    */  
    delayMicroseconds(1);
}   
  
//---- 毫秒级延时程序-----------------------   
void delay_ms(unsigned int nCount )   
{   
   delay(nCount);
}   


unsigned char  ucharToBcd(unsigned char vaule)
{
  unsigned char temp1,temp2,bcd;
  temp1=vaule/10;
  temp2=vaule%10;
  bcd=temp1*16+temp2;
  return bcd;
}
unsigned char BcdTouchar(unsigned char vaule)
{
  unsigned char h,l,c;
  h=vaule>>4;
  l=vaule&0x0f;
  c=h*10+l;
  return c;
}
//RTC 芯片的数据和中断端口初始化
void RTC_GPIO_INI(void)
{
  pinMode (RTC_SCL,INPUT);
  pinMode (RTC_SDA,INPUT);
  pinMode (RTC_INT,INPUT);
  RTCWire.begin();
 // RTCWire.i2c_start();
  //RTCWire.i2c_stop();
}

void ReadRegs(void)    //上电就调用这个函数，把读到的数据通
{
  unsigned char  dat[5],i;
  unsigned char   head=0x55;
  dat[0]=ReadOneByte(7);//读秒报警数据
  dat[1]=ReadOneByte(8);//读分报警数据
  dat[2]=ReadOneByte(9);//读小时报警数据
  dat[3]=ReadOneByte(0x0e);//读报警允许数据
  dat[4]=ReadOneByte(0x10);//读CTR2数据

}



//***************************************************************  
  
//  Send a byte to the slave 写一个数据没有应答  
  
//  return I2C_ERR OR I2C_CRR  
  
//***************************************************************  
  
bool I2CSendByte(unsigned char I2cData)  
  
{  
  
  RTCWire.i2c_shift_out(I2cData);  
  bool I2CStatus;  
  I2CStatus=true;
  return I2CStatus;  
  
}  
  
  
//***************************************************************  
  
//ROUTINE NAME : I2Cm_RxData  
  
//INPUT/OUTPUT : Last byte to receive flag (active high)/Received data byte.  
  
//DESCRIPTION  : Receive a data byte.  
  
//COMMENTS     : Transfer sequence = DATA, ACK, EV7...  
  
//***************************************************************  
unsigned char I2CReceiveByte(void)  
  
{     
 unsigned char ReadByte=0;  
 ReadByte=RTCWire.i2c_shift_in();  
 return ReadByte;    
}  

/******读SD2400实时数据寄存器******/
bool I2CReadDate(void)
{
  unsigned char n;

  RTCWire.i2c_start();
  I2CSendByte(0x65);
  if(!RTCWire.i2c_get_ack())
  { 
   RTCWire.i2c_stop();
   return false;
  }

  for(n=0;n<7;n++)
  {
    date[n]=I2CReceiveByte();
    if (n!=6)         //最后一个数据不应答
    {
      RTCWire.i2c_send_ack();
    }
  }
  RTCWire.i2c_send_nack();
  RTCWire.i2c_stop();
        return true;
}

/******写SD2400实时数据寄存器******/
bool I2CWriteDate(void)
{   
  WriteTimeOn();
  RTCWire.i2c_start();
  I2CSendByte(0x64);
  if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}

  I2CSendByte(0x00);//设置写起始地址
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x01);// second
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x01);//minute
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x88);//hour ,二十四小时制
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x01);//week
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x01);//day
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x07);//month
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x07);//year
  RTCWire.i2c_get_ack(); 
  RTCWire.i2c_stop();

  RTCWire.i2c_start();
  I2CSendByte(0x64);
  RTCWire.i2c_get_ack();
        I2CSendByte(0x12);//设置写起始地址
  RTCWire.i2c_get_ack();   
        I2CSendByte(0x00);//清零数字调整寄存器
  RTCWire.i2c_get_ack();
  RTCWire.i2c_stop();
  
  WriteTimeOff();
        return true;
}

bool  SetRtcTime(UTCTimeStruct *Time)
{
  unsigned char Temp;
  noInterrupts();
  WriteTimeOn();
  RTCWire.i2c_start(); 
  I2CSendByte(0x64);
  if(!RTCWire.i2c_get_ack())
  {
  RTCWire.i2c_stop();
  interrupts();
  Serial.println("wait I2C ACK TimeOut!");
  return false;
  }
  I2CSendByte(0x00);//设置写起始地址
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->seconds));// second
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->minutes));//minute
  RTCWire.i2c_get_ack(); 
  Temp=ucharToBcd(Time->hour);
  Temp|=0x80;//按24小时制设置
        
  I2CSendByte(Temp);//hour ,二十四小时制
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->week));//week
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->day));//day
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->month));//month
  RTCWire.i2c_get_ack(); 
  Temp=Time->year-2000;
  I2CSendByte(ucharToBcd(Temp));//year
  RTCWire.i2c_get_ack(); 
  RTCWire.i2c_stop();

  RTCWire.i2c_start();
  I2CSendByte(0x64);
  RTCWire.i2c_get_ack();
  I2CSendByte(0x12);//设置写起始地址
  RTCWire.i2c_get_ack();   
  I2CSendByte(0x00);//清零数字调整寄存器
  RTCWire.i2c_get_ack();
  RTCWire.i2c_stop();
  
  WriteTimeOff();
        interrupts();
        return true;
}
bool  GetRtcTime(UTCTimeStruct *Time)
{
  unsigned char n;
  bool err=true;
  noInterrupts();
  RTCWire.i2c_start();
  I2CSendByte(0x65);
  if(!RTCWire.i2c_get_ack())
  {
   RTCWire.i2c_stop();
   interrupts();
   return false;
  }
  for(n=0;n<7;n++)
  {
    date[n]=I2CReceiveByte();
    if (n!=6) //最后一个数据不应答
    {
       RTCWire.i2c_send_ack();
    }
  }
  RTCWire.i2c_send_nack();
  RTCWire.i2c_stop();
  n=0;
  Time->seconds=BcdTouchar(date[n++]&0x7f);// second
  Time->minutes=BcdTouchar(date[n++]&0x7f);//minute
  Time->hour=BcdTouchar(date[n++]&0x3f);//hour ,二十四小时制
  Time->week=BcdTouchar(date[n++]&0x07);//week
  Time->day=BcdTouchar(date[n++]&0x3f);//day
  Time->month=BcdTouchar(date[n++]&0x1f);//month
  Time->year=BcdTouchar(date[n++])+2000;//year
         
    if(Time->year<2000||Time->year>2099)
    {
      Time->year=2014;
      err=false;
    }
    if(Time->month>12||Time->month<1)
    {
      Time->month=7;
      err=false;
    }
    if(Time->day>31||Time->day<1)
    {
      Time->day=1;
      err=false;
    }
    if(Time->hour>=24)
    {
      Time->hour=0;
      err=false;
    }
    if(Time->minutes>59)
    {
      Time->minutes=0;
      err=false;
    }
    
       /*
        if(Time->seconds>59)
        {
          Time->seconds=Time->seconds%59;
          Time->minutes+=1;
        }
       */
        interrupts();
        return err;
}

bool  SetAlarmOutTime(UTCTimeStruct *Time)
{
        unsigned char Temp;
        noInterrupts();
        WriteTimeOn();
        Delay(10);
  RTCWire.i2c_start();
  I2CSendByte(0x64);
        if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}

        I2CSendByte(0x07);//设置写起始地址
  RTCWire.i2c_get_ack(); 

  I2CSendByte(ucharToBcd(Time->seconds));// second
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->minutes));//minute
  RTCWire.i2c_get_ack(); 
        //判断是12小制或24小时制

        Temp=ucharToBcd(Time->hour);

  /*2015-12-22注释以下****内的内容*/
//**********************************************
        if(Time->hour>12)
        {
          Temp|=0x80;
        }
        else
        {
         Temp&=0x1f;
        }
//***********************************************
  I2CSendByte(Temp);//hour ,二十四小时制
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->week));//week
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->day));//day
  RTCWire.i2c_get_ack(); 
  I2CSendByte(ucharToBcd(Time->month));//month
  RTCWire.i2c_get_ack(); 
        Temp=Time->year-2000;
  I2CSendByte(ucharToBcd(Temp));//year
  RTCWire.i2c_get_ack(); 
        //设置报警允许字段，年、月、日、小时，分，秒
        I2CSendByte(0x77);//EAH=1;EAMN=1;EAS=1;
        RTCWire.i2c_get_ack();
        RTCWire.i2c_stop();
        WriteTimeOff();
        //选择并允许定时报警中断输出
        Delay(100);
        Rtc_Intterupt_Set(RTC_Alarm);
        interrupts();
        return true;
}


bool  GetAlarmOutTime(UTCTimeStruct *Time)
{
      unsigned char n;
  RTCWire.i2c_start();
  I2CSendByte(0x64);
        if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop();
        return false;}
        I2CSendByte(0x07);
        RTCWire.i2c_get_ack();
         RTCWire.i2c_start();
        I2CSendByte(0x65);
        if(!RTCWire.i2c_get_ack())
        {RTCWire.i2c_stop();}
  for(n=0;n<7;n++)
  {
          //date[n]=BcdTouchar(I2CReceiveByte());
          date[n]=I2CReceiveByte();
          if (n!=6)         //最后一个数据不应答
          {
            RTCWire.i2c_send_ack();
          }
  }
  RTCWire.i2c_send_nack();
  RTCWire.i2c_stop();  
        
        n=0;
  Time->seconds=BcdTouchar(date[n++]);// second
  Time->minutes=BcdTouchar(date[n++]);//minute
  Time->hour=BcdTouchar(date[n++]&0x3f);//hour ,二十四小时制
  Time->week=BcdTouchar(date[n++]);//week
  Time->day=BcdTouchar(date[n++]);//day
  Time->month=BcdTouchar(date[n++]);//month
  Time->year=BcdTouchar(date[n++])+2000;//year
        /*
        n=0;
        Time->seconds=(date[n++]);// second
        Time->minutes=(date[n++]);//minute
  Time->hour=(date[n++]);//hour ,二十四小时制
        Time->week=date[n++];//week
        Time->day=(date[n++]);//day
        Time->month=(date[n++]);//month
        Time->year=(date[n++])+2000;//year
        */
        return true;
}
/*****************************************************************************
 * @fn          SaveAlarmOutTime
 *
 * @brief       存储其它数据用户RAM区
 *
 * @param       Offset-地址偏移量
                buf - 存储的数据Buf
                len - 存储的数据长度
 *
 * @return      true - success
 *               false-fail
 */
bool  SaveUserData(unsigned char Offset,unsigned char *buf,unsigned char len)
{
       unsigned char Temp;
       unsigned char i;  
       if(len>12)
       {
         return false;
       }
       noInterrupts();
        WriteTimeOn();
        Delay(1000);
        RTCWire.i2c_start();
        I2CSendByte(0x64);
        if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}
        I2CSendByte(0x14+Offset);//设置写起始地址
  RTCWire.i2c_get_ack(); 
        
        for(i=0;i<len;i++)
        {
           I2CSendByte(buf[i]);
     RTCWire.i2c_get_ack();  
        }
  RTCWire.i2c_stop();
        WriteTimeOff();
        //选择并允许定时报警中断输出
        Delay(1000);
        interrupts();
        return true;
}


/*****************************************************************************
 * @fn          ReadUserData
 *
 * @brief       从数据用户RAM区读取数据
 *
 * @param       Offset-地址偏移量
                buf - 数据Buf
                len - 数据长度
 *
 * @return      true - success
 *               false-fail
 */
bool  ReadUserData(unsigned char Offset,unsigned char *buf,unsigned char len)
{
    unsigned char n;
    if(len>12)
   {
     return false;
   }
    RTCWire.i2c_start();
    I2CSendByte(0x64);
    if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop();
    return false;}
    I2CSendByte(0x14+Offset);
    RTCWire.i2c_get_ack();
    RTCWire.i2c_start();
    I2CSendByte(0x65);
    if(!RTCWire.i2c_get_ack())
    {RTCWire.i2c_stop();}
    for(n=0;n<len;n++)
    {
        buf[n]=I2CReceiveByte();
        if (n!=(len-1))         //最后一个数据不应答
        {
          RTCWire.i2c_send_ack();
        }
    }
    RTCWire.i2c_send_nack();
    RTCWire.i2c_stop();
    n=0;
    return true;
}


/*****************************************************************************
 * @fn          RTC_Set_CountdownIn
 *
 * @brief       The RTC_Set_CountdownIn function is 设置RTC的倒计时中断时间等参数
 *
 * @param       Time - 倒计时定时器的时间
 *              unit - 倒计时定时器的单位，0x01-秒，0x02-分
 *
 * @return      true - success
 *               false-fail
 */
bool RTC_Set_CountdownIn(unsigned char Time,unsigned char unit)
{
    static unsigned char temp=0x80;
    noInterrupts();
    WriteTimeOn();
    //写控制寄存器 2
    RTCWire.i2c_start();
    I2CSendByte(DevAddrW);
    if(!RTCWire.i2c_get_ack())
    {RTCWire.i2c_stop(); return false;}
    I2CSendByte(CTR3);//设置写地址10H
    RTCWire.i2c_get_ack();
    switch(unit)
    {
    case 0x01:
     temp&=~TDS0;  //计时单位设置为秒
     temp|=TDS1;

     break;
    case 0x02:
      temp|=TDS0;//计时单位设置为分
      temp|=TDS1;
      break;
    default:
      return false;
      break;
    }
    I2CSendByte(temp) ;
    RTCWire.i2c_get_ack();
    RTCWire.i2c_stop();
    WriteTimeOff();
    //写入倒计时的时间
    WriteTimeOn();
    //写控制寄存器 2
    RTCWire.i2c_start();
    I2CSendByte(DevAddrW);
    if(!RTCWire.i2c_get_ack())
    {RTCWire.i2c_stop(); return false;}
    I2CSendByte(COUNT_DOWN_TIME_SR);//设置写地址13H
    RTCWire.i2c_get_ack();
    temp=Time;
    I2CSendByte(temp) ;
    RTCWire.i2c_get_ack();
    RTCWire.i2c_stop();
    WriteTimeOff();
    //设置到计时中断
    Rtc_Intterupt_Set(Count_down);
    interrupts();
    return true;

}

/******写SD2400允许程序******/
bool WriteTimeOn(void)
{ 
  RTCWire.i2c_start();
  I2CSendByte(0x64);
        if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}

        I2CSendByte(0x10);//设置写地址10H
  RTCWire.i2c_get_ack(); 
        I2CSendByte(0x80);//置WRTC1=1
  RTCWire.i2c_get_ack();
  RTCWire.i2c_stop();
  
  RTCWire.i2c_start();
  I2CSendByte(0x64);
  RTCWire.i2c_get_ack();
        I2CSendByte(0x0F);//设置写地址0FH
  RTCWire.i2c_get_ack(); 
        I2CSendByte(0x84);//置WRTC2,WRTC3=1
  RTCWire.i2c_get_ack();
  RTCWire.i2c_stop();
        return true;
  
}


/******写SD2400禁止程序******/
bool WriteTimeOff(void)
{
  RTCWire.i2c_start();
  I2CSendByte(0x64);
        if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}
  
        I2CSendByte(0x0F);//设置写地址0FH
  RTCWire.i2c_get_ack(); 
  I2CSendByte(0x0) ;//置WRTC2,WRTC3=0
  RTCWire.i2c_get_ack();
  I2CSendByte(0x0) ;//置WRTC1=0(10H地址)
  RTCWire.i2c_get_ack();
  RTCWire.i2c_stop();
        return true;
}
/******写SD2400中断报警允许设置*****/
bool Rtc_Intterupt_Set(unsigned char IntSource)
{
    static unsigned char temp=0x80;
  //先读出控制寄存器2的值
   // temp=ReadOneByte(CTR2);
    WriteTimeOn();
    //写控制寄存器 2
    RTCWire.i2c_start();
    I2CSendByte(DevAddrW);
    if(!RTCWire.i2c_get_ack())
    {RTCWire.i2c_stop(); return false;}
    I2CSendByte(CTR2);//设置写地址10H
    RTCWire.i2c_get_ack();
    switch(IntSource)
    {
    case Dis_Int:
      temp&=0xC1;  //置INTS0=0;INTS1=0;
      break;
    case RTC_Alarm:
      temp|=0x12;  //置INTS0=1;INTS1=0;INAE=1;
      break;
    case frequency:
      temp|=0x21;  //置INTS0=0;INTS1=1;INFE=1;
      break;
    case Count_down:
     temp|=0x34;  //置INTS0=1;INTS1=1;INDE=1;
      break;
    default:
      break;
    }
    temp|=0x40;//IM=1;报警时输出250ms周期性脉冲
    //temp|=0x00;//IM=0;报警时输出低电平直至 INTAF 位清零
    I2CSendByte(temp) ;
    RTCWire.i2c_get_ack();
    RTCWire.i2c_stop();
    WriteTimeOff();
    return true;
}

unsigned char ReadOneByte(unsigned char addr)
{
  static unsigned char temp;
  //先读出控制寄存器2的值
    RTCWire.i2c_start();
    I2CSendByte(DevAddrW);
   
    if(!RTCWire.i2c_get_ack())
    {
      RTCWire.i2c_stop();
       return false;
    }
    I2CSendByte(addr);
    RTCWire.i2c_get_ack();
    //重新开始总线
    RTCWire.i2c_start();
    I2CSendByte(DevAddrR);
    if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}
    temp=I2CReceiveByte();
    RTCWire.i2c_send_ack();
    I2CReceiveByte();
    RTCWire.i2c_send_ack();
    RTCWire.i2c_stop();
    return temp;
}
 bool ClearAlarmOut(void)
 {
     unsigned char temp;
     temp=ReadOneByte(CTR1);
    //写控制寄存器 1  
     WriteTimeOn();
    RTCWire.i2c_start();
    I2CSendByte(DevAddrW);
    if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}
    I2CSendByte(CTR1);//设置写地址0FH
    RTCWire.i2c_get_ack();
    temp&=0xCF;  //置INTAF=0,INTDF清除报警
    I2CSendByte(temp) ;
    RTCWire.i2c_get_ack();
    RTCWire.i2c_stop();
    WriteTimeOff();
    return true;
 }

/*
 bool ClearAlarmOut(void)
 {
     unsigned char temp;
  //先读出控制寄存器1的值
    if(!RTCWire.i2c_start())return false;
    I2CSendByte(DevAddrR);
    if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}
    I2CSendByte(CTR1);
    RTCWire.i2c_get_ack();
    temp=I2CReceiveByte();
    RTCWire.i2c_send_nack();
    RTCWire.i2c_stop();
    //写控制寄存器 2
    if(!RTCWire.i2c_start())return false;
    I2CSendByte(DevAddrW);
    if(!RTCWire.i2c_get_ack()){RTCWire.i2c_stop(); return false;}
    I2CSendByte(CTR1);//设置写地址0FH
    RTCWire.i2c_get_ack();
    temp&=0xCF;  //置INTAF=0,INTDF清除报警
    I2CSendByte(temp) ;
    RTCWire.i2c_get_ack();
    RTCWire.i2c_stop();
    //关闭RTC中断输出功能
    Rtc_Intterupt_Set(Dis_Int);
    return true;
 }
*/

/*********延时子程序*********/
void Delay(unsigned int nn)
{
   while(nn--)
   {
     delayMicroseconds(1000);
   }
}


void localtime_h(uint32 time, UTCTimeStruct *ret_time)
  {
      static const char month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      static const char leap_year[4] = {0, 0, 1, 0};
      uint32 j = 0;
      uint32 day_count = 0;
      uint32 i = 0;
      uint32 leave_for_fouryear = 0;
      uint32 four_year_count = 0;
      uint32 temp_value = 0;
      uint32 leave_for_year_days;
      uint32 leave_for_month_days;
      //time=time-28800;
      ret_time->seconds = 0;
      ret_time->minutes = 0;
      ret_time->hour = 0;
      ret_time->day = 0;
      ret_time->year = 0;
      ret_time->day = 0;
      ret_time->month = 0;
      ret_time->seconds = time % 60;
      temp_value = time / 60;
      ret_time->minutes = temp_value % 60;
      temp_value /= 60; 
      temp_value += 8;
      ret_time->hour = temp_value % 24;
      temp_value /= 24;
      ret_time->day = (temp_value + 4) % 7;
      four_year_count = temp_value / (365 * 4 + 1);
      leave_for_fouryear = temp_value % (365 * 4 + 1);
      leave_for_year_days = leave_for_fouryear; 
     for (i = 0; i < 4; i++)
     {        
          day_count = leap_year[i] ? 366 : 365;
          
          if (leave_for_year_days <= day_count)
          {
              break;
          }
          else
          {
              leave_for_year_days -= day_count;
          }
     }
      ret_time->year = four_year_count * 4 + i ;
      //ret_time->yday = leave_for_year_days;
      leave_for_month_days = leave_for_year_days;
     for (j = 0; j < 12; j++)
     {
         if (((leap_year[i])) && (j == 1))
         {
             if (leave_for_month_days <= 29)
             {
                 break;
             }
             else if (leave_for_month_days == 29)
             {
                 i++;
                 leave_for_month_days = 0;
                 break;
             }
             else
             {
                 leave_for_month_days -= 29;
             }
             
             continue;    
         }
                 
         if (leave_for_month_days < month_days[j])
         {
             break;
         }
         else if(leave_for_month_days == month_days[j]){
             i++;
             leave_for_month_days = 0;
             break;
         }
         else
         {
             leave_for_month_days -= month_days[j];
         }                
     }
     ret_time->day = leave_for_month_days + 1;
     ret_time->month = j;
  }

 void ConvSecTimeToCalendar(UTCTimeStruct *t_tm,uint32 t)
 {
     t=(t>=28800)?(t-28800):t;
     localtime_h(t,t_tm);
     t_tm->year +=1970;
     t_tm->month +=1;
 }

