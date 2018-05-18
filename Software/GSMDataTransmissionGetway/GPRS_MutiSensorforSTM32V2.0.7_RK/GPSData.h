 /**
  ******************************************************************************
  * @file    GPS.h 
  * @author  yangzhaoguo
  * @version V1.0.0
  * @date    11/03/2010
  * @brief   GPS模块数据接口定义
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2010 yw</center></h2>
  */
#ifndef _GPSDATA_H
#define _GPSDATA_H
typedef struct{
   int year;    //年
   int month;   //月
   int day;     //日
   int hour;    //时
   int minutes;  //分
   int seconds;  //秒
}date_time;  //时间结构体
typedef struct{
    date_time D;//时间
    float latitude;   //纬度
    float longitude;  //经度
     char LocationFlag;   //定位状态，“A”,已定位，“V”未定位
     char latitudestr[16];   //纬度
     char longitudestr[16];   ////经度
     char altitudestr[16];   ////经度
    double altitude;     //海拔
    float course_over_ground;//对地方向
     char fix_Mode;
    char NS;           //南北极
    char EW;           //东西
    float  rate;       //速度
    float HDOP;//水平分量精度因子：为纬度和经度等误差平方和的开根号值。
    float PDOP;//三维位置精度因子：为纬度、经度和高程等误差平方和的开根号值
    float VDOP;//垂直分量精度因子
    float TDOP;//钟差精度因子：为接收仪内时表偏移误差值。
    char SNR;
    char view;
    int ACC;//LBS定位精度
}GPS_INFO;

void gps_parse(volatile char *line,GPS_INFO *GPS);
void lbs_parse(volatile char *line,GPS_INFO *GPS);
void show_gps(GPS_INFO *GPS);
static void UTC2BTC(date_time *GPS);
 /**
  *@brief  将GPS世界时间转换为北京时间
  *@param [in] GPS gps数据流的指针
  *@return None
  */
static void UTC2BTC(date_time *GPS)
{
   //如果秒号先出,再出时间数据,则将时间数据+1秒
   GPS->seconds++; //加一秒
   if(GPS->seconds>59){
      GPS->seconds=0;
      GPS->minutes++;
      if(GPS->minutes>59){
        GPS->minutes=0;
        GPS->hour++;
      }
  } 
  GPS->hour+=8;   //北京时间与世界时间相差8个时区，即相差8个钟
  if(GPS->hour>23)
  {
     GPS->hour-=24;
     GPS->day+=1;
     if(GPS->month==2 || GPS->month==4 || GPS->month==6 || GPS->month==9 || GPS->month==11 ){
        if(GPS->day>30){
           GPS->day=1;
           GPS->month++;
        }
     }
     else{
        if(GPS->day>31){
           GPS->day=1;
           GPS->month++;
        }
   }
   if((GPS->year % 4 ==0) && (GPS->year % 400 == 0 || GPS->year % 100 != 0)){  //判断闰年
       if(GPS->day > 29 && GPS->month ==2){   //闰年二月比平年二月多一天
          GPS->day=1;
          GPS->month++;
       }
   }
   else{
       if(GPS->day>28 &&GPS->month ==2){
         GPS->day=1;
         GPS->month++;
       }
   }
   if(GPS->month>12){
      GPS->month-=12;
      GPS->year++;
   }  
  }
}
/**
  *@brief  解释gps发出的数据流，并将解析数据放入数据结构中
  *@param [in] line gps数据流的指针
  *@param [in] GPS GPS数据结构的指针
  *@return None
  *@note    
  *  +CLBS: 0,102.655042,25.065250,550,17/06/21,09:29:08
  */
void lbs_parse(String line,GPS_INFO *GPS)
{
     String strTemp;
     char str[30]={0};
      strTemp = (line.substring(0, line.indexOf(",")));
      int Parameter1 = strTemp.toInt();
      //取经度
      line = line.substring(line.indexOf(",") + 1);  
      strTemp = (line.substring(0, line.indexOf(",")));
      //memcpy((char*)GPS->longitudestr,strTemp.toCharArray(Timebuf));
      strTemp.toCharArray(GPS->longitudestr,strTemp.length()+1);
      GPS->longitude=strTemp.toFloat();   
      Serial1.println("longitude:"+(String)GPS->longitudestr);
      //Serial1.print(GPS->longitude);
      //纬度
      line = line.substring(line.indexOf(",") + 1);  
      strTemp = (line.substring(0, line.indexOf(",")));
      strTemp.toCharArray(GPS->latitudestr,strTemp.length()+1);
      //memcpy((char*)GPS->latitudestr,strTemp.toCharArray(Timebuf));
      GPS->latitude=strTemp.toFloat();   
      //float latitude =strTemp.toFloat();
      Serial1.println("latitude:"+(String)GPS->latitudestr);
      //Serial1.print(GPS->latitude);
      //定位半径，精度
      line=line.substring(line.indexOf(",")+1);
      strTemp=(line.substring(0,line.indexOf(",")));
      int PosRadius=strTemp.toInt();
      GPS->ACC=PosRadius;
      //日期字符串
      line=line.substring(line.indexOf(",")+1);
      strTemp=(line.substring(0,line.indexOf(",")));
      char Datebuf[10]={0};
      strTemp.toCharArray(Datebuf,9);
      //Serial1.write(Datebuf,9);
      GPS->D.year=2000+(Datebuf[0]-'0')*10+(Datebuf[1]-'0');
      GPS->D.month=(Datebuf[3]-'0')*10+(Datebuf[4]-'0');
      GPS->D.day=(Datebuf[6]-'0')*10+(Datebuf[7]-'0');
      Serial1.println("Date:"+strTemp);
      //Serial1.println(strTemp);
      //取时间字符串
      line=line.substring(line.indexOf(",")+1);
      strTemp=(line.substring(0,line.indexOf(",")));
      char Timebuf[9]={0};
      strTemp.toCharArray(Timebuf,9);
      //Serial1.write(Timebuf,9);
      GPS->D.hour=(Timebuf[0]-'0')*10+(Timebuf[1]-'0');
      GPS->D.minutes=(Timebuf[3]-'0')*10+(Timebuf[4]-'0');
      GPS->D.seconds=(Timebuf[6]-'0')*10+(Timebuf[7]-'0');
      //itoa(
      Serial1.println("Time:"+strTemp);
      //Serial1.println(strTemp);
      UTC2BTC(&GPS->D);   
}

#endif
/******************* (C) COPYRIGHT 2010 YW *****END OF FILE****/

