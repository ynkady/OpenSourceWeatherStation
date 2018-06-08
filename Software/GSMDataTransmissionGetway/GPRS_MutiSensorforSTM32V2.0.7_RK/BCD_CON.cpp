/**************************************************************************************************
  Filename:       BCD_CON.c
 
  Description:    BCD Code conversion function 
**************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */
//#include "BCD_CON.h"
#include "math.h"
#include "stdlib.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
/***************************************************************************************************
 * CONSTANTS
 ***************************************************************************************************/

/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/
//基于查表实现BCD与Ascii之间的转换  
static const unsigned char bcd2ascii[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};  
static const unsigned char ascii2bcd1[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};  
static const unsigned char ascii2bcd2[6]  = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}; 

/******************************************************************************
 * FUNCTIONS
 */
/*******************************************************************************/

/******************************************************************************
 * @fn          ToBCD
 *
 * @brief       conversion tow Byte to one BCD 
 *
 * @param       n - out BCD
 *              p0- one byte point
 *              P1- tow byte point
 *              flag - num flag
 * @return      none
 */
void ToBCD(int n,unsigned char *p1,unsigned char *p0,unsigned char *flag)
{
  char ch3,ch2,ch1,ch0;
  int  m;

  if(n<0) 
  {
    *flag=1;
    m=-n;
  }
  else
  {
    *flag=0;
    m=n;
  }
  if(m>9999  )
  {
    *p1=0x99;
    *p0=0x99;
  }
  else
  {
    ch3=m/1000;             //取最高位
    ch2=(m-ch3*1000)/100;       //取次高位
    ch1=(m-ch3*1000-ch2*100)/10;
    ch0=m%10;           //取最低位
    *p1=((ch3 & 0x0f)<<4) | (ch2 & 0x0f);
    *p0=((ch1 & 0x0f)<<4) | (ch0 & 0x0f);
  
  }
}
/******************************************************************************
 * @fn          ByteTOBcd
 *
 * @brief       conversion one Byte to one BCD 
 *
 * @param       OneByte - dec

 * @return      BCD code
 */
unsigned char ByteTOBcd(unsigned char OneByte)
{
  unsigned char temp,ch1,ch2;
  ch1=OneByte/10;
  ch2=OneByte%10;
  //temp = ((OneByte>>4)&0x0F)*10 + (OneByte&0x0F);   
  temp=((ch1 & 0x0f)<<4) | (ch2 & 0x0f);
  return temp;        
}



 /*BCD 与 ASCII码转换*/  
  
/******************************************************************* 
函数名:  asc2bcd  
功能描述:将ascii码转换为bcd码 
参数:     
         bcd:转换后的BCD码 
         asc:需转换的ASCII码串 
         len:需转换的ascii码串长度 
 
返回值:  uint32  
            0：成功 
            其他:失败 
********************************************************************/    
unsigned int ASC2BCD(unsigned char *bcd, char *asc, unsigned int len)  
{  
    unsigned char c = 0;  
    unsigned char index = 0;  
    unsigned char i = 0;

   if(len%2!=0)    
   {
     len+=1;
   }
   
    len >>= 1;      
  
    for(; i < len; i++) {  
        //first BCD  
        if(*asc >= 'A' && *asc <= 'F') {  
            index = *asc - 'A';   
            c  = ascii2bcd2[index] << 4;  
        } else if(*asc >= '0' && *asc <= '9') {  
            index = *asc - '0';  
            c  = ascii2bcd1[index] << 4;  
        }  
        asc++;  
  
        //second BCD  
        if(*asc >= 'A' && *asc <= 'F') {  
            index = *asc - 'A';   
            c  |= ascii2bcd2[index];  
        } else if(*asc >= '0' && *asc <= '9') {  
            index = *asc - '0';  
            c  |= ascii2bcd1[index];  
        }  
        asc++;  
  
        *bcd++ = c;  
    } 
 /*
 int j=len-1; 
 unsigned char temp;   
  do
  {
   if(bcd[j]==0x00)
   {
     temp=bcd[0];
     bcd[0]=bcd[j];
   }
   else
   {
     break;
   }
   j--;
  }while(j>0);
  */
    return 0;  
}  

/******************************************************************* 
函数名: bcd2asc  
功能描述:将bcd码转换为ascii码串 
参数:     
         asc:转换的ASCII码串 
         bcd:需转换的BCD码 
         len:需转换的BCD码长度 
 
返回值:  uint32  
                              0：成功 
                              其他:失败 
********************************************************************/  

unsigned int  BCD2ASC ( char  *asc, const  char *bcd, unsigned int len)  
{  
    unsigned char c = 0;  
    unsigned char i;  
  
    for(i = 0; i < len; i++) {  
        //first BCD  
        c = *bcd >> 4;  
        *asc++ = bcd2ascii[c];  
  
  
        //second  
        c = *bcd & 0x0f;  
        *asc++ = bcd2ascii[c];  
        bcd++;  
    }  
  
  
    return 0;  
}  

/******************************************************************************
 * @fn          FloatStringToIntString
 *
 * @brief      把一个带小数据的字符串转换成一个整型数的字符串，并返回小数位数
 *
 * @param       fstring-小数字符串
                Istring-整型数据字符串
 *              NumOfDot-小数位数变量指针
                sringLen-小数字符串的长度
 * @return     None
 */
void FloatStringToIntString( char *fstring, char *Istring, unsigned char *NumOfDot,int sringLen)
{
  unsigned char L1,L2,Slen;
  unsigned char i,j; 
  int offset=0;
  Slen=strlen((char*)fstring);
  j=0;
  for(i=0;i<=Slen;i++)
  {

     if(fstring[i]==0x2E)//小数点
    {
      L1=i;
    }
    else if(fstring[i]==0x2C) //,号
    {
      L2=i;
      *NumOfDot=L2-L1-1;
    }
    //结束符
    else if(fstring[i]==0x00) //结束符
    {
      Istring[j++]='\0';
      break;
    }
    else
    {
      Istring[j++]=fstring[i];
    }
    
  } 
}

void ulongtoString(char buffer[],unsigned long a)
{
  int i,j;
  i=0;  
  while(a)
  {
    buffer[i++]=a%10+'0';
    a/=10;
  }
  
  for(j=0;j<=i/2;j++)
  {
     int t=buffer[j];
     buffer[j]=buffer[i-j-1];
     buffer[i-j-1]=t;
  }
 buffer[i]='\0'; 
}


// 压缩BCD码一个字符所表示的十进制数据范围为0 ~ 99,进制为100  
// 先求每个字符所表示的十进制值，然后乘以权  
//////////////////////////////////////////////////////////   
unsigned int  BCDtoDec(unsigned char *bcd, int length)  
{        
  int i, tmp;        
  unsigned int dec = 0;       
  for(i=0; i<length; i++)       
  {           
  tmp = ((bcd[i]>>4)&0x0F)*10 + (bcd[i]&0x0F);             
  dec += tmp*pow(100,length-1-i);                 
  }        
  return dec;  
} 


unsigned int inttoBCD(int m)
{
    unsigned int r=0,n=1;
    int a;
    while(m)
    {
        a=m %10;
        m=m/10;
        r=r+n*a;
        a=a<<4;
    } 
    return r;
}

/*---------------------------------------------------------------------- 
* BCD编码
* char* buff : 目标缓冲区 
* float value : 值 
* int width : 目标宽度 
* int decimal: 目标小数位数 
*--------------------------------------------------------------------*/  
void PackBCD(char* buff, float value, int width, int decimal)  
{  
    int i;  
      
    if ((float)value >= pow(10.0, width-decimal))  
    {  
        memset(buff, 0, width);  
        return;  
    }  
      
    for (i=0; i<(width-decimal)/2; i++)  
    {  
        buff[i] = (int)(value/pow(10.0, width-decimal-i*2-1)) % 10*16  
            + (int)(value/pow(10.0, width-decimal-i*2-2)) % 10;  
    }  
      
    for (i=(width-decimal)/2; i<width/2; i++)  
    {  
        buff[i] = (int)(value*pow(10, i*2+decimal-width+1)) % 10 *16  
            + (int)(value*pow(10, i*2+decimal-width+2)) % 10;  
    }  
}  
/*---------------------------------------------------------------------- 
* BCD解码 
* char* buff : BCD码 缓冲区 
* int width : 目标宽度 
* int decimal: 目标小数位数 
* 返回：转换后的浮点数
*--------------------------------------------------------------------*/  
float UnPackBCD(char* buff, int width, int decimal)  
{  
    float value = 0;  
    int i;  
      
    for (i=0; i<(width-decimal)/2; i++)  
    {  
        value+=(buff[i]/16 * pow(10, width-decimal-i*2-1)  
            + buff[i]%16 * pow(10, width-decimal-i*2-2));  
    }  
      
    for (i=(width-decimal)/2; i<width/2; i++)  
    {  
        value+=(buff[i]/16 / pow(10, i*2+decimal-width+1)  
            + buff[i]%16 / pow(10, i*2+decimal-width+2));  
    }  
      
    return value;  
}  
  
int FindDecimal(char *str)  
{     
    int len = 0, i;   
    for(i = 0; i < strlen(str); i ++)  
    {  
        if(str[i]!='.')  
        {  
            len ++;  
        }  
        else  
        {  
            break;  
        }  
    }  
    return strlen(str) - len - 1;         
} 


void long_to_bcd(unsigned long sec)
{
  unsigned char a[10];
  a[0] = sec % 10; // 获得个位
  a[1] = sec / 10 % 10; // 获得十位
  a[2] = sec / 100 % 10; // 获得百位
  a[3] = sec / 1000 % 10; // 获得千位
  a[4] = sec / 10000 % 10; // 获得万位
  a[5] = sec / 100000 % 10; // 获得十万位
  a[6] = sec / 1000000 % 10; // 获得百万位
  a[7] = sec / 10000000 % 10; // 获得千万位
  a[8] = sec / 100000000 % 10; // 获得亿位
  a[9] = sec / 1000000000 % 10; // 获得十亿位
}

