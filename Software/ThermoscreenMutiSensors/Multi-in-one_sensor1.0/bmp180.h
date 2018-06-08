/**************************************************************************************/
           /**********************DFRobot.com*****************************/
/***write by Tom Riddler          Jun.16.14***/
/***if you got any progream,please contact me terminaterfxy@hotmail.com***/

//#include <Wire.h>

#define BMP180ADD 0xEE>>1  // I2C address of BMP180  
                           //write is (0xEE)     read is (0xEF)       
extern unsigned char OSS;   

void Bmp_begin(void);

/**BMP180 satrt program**/
void BMP180start();

/***BMP180 temperature Calculate***/
short bmp180GetTemperature(unsigned int ut);

/***BMP180 pressure Calculate***/

long bmp180GetPressure(unsigned long up);
/*** Read 1 bytes from the BMP180  ***/

int bmp180Read(unsigned char address);

/*** Read 2 bytes from the BMP180 ***/
int bmp180ReadDate(unsigned char address);

/*** read uncompensated temperature value ***/
unsigned int bmp180ReadUT();

/*** Read uncompensated pressure value from BMP180 ***/
unsigned long bmp180ReadUP();
