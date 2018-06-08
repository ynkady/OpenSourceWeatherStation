void ToBCD(int n,unsigned char *p1,unsigned char *p0,unsigned char *flag);
//unsigned int BCDToInt(unsigned char byte1,unsigned char byte0);
//int TD_ASC2BCD(unsigned char *bcd, const char *asc,int len,int fmt );
void FloatStringToIntString(char *fstring,char *Istring,unsigned char *NumOfDot,int sringLen);
//unsigned int ASC2BCD(char *bcd, char *asc, int len);
unsigned int ASC2BCD(unsigned char *bcd, char *asc, unsigned int len); 
unsigned int  BCD2ASC ( char  *asc,char *bcd,int len);
void ulongtoString(char buffer[], long a);
extern unsigned int  BCDtoDec(unsigned char *bcd, int length);
unsigned int inttoBCD(int m);  
unsigned char ByteTOBcd(unsigned char OneByte);
void PackBCD(char* buff, float value, int width, int decimal);
float UnPackBCD(char* buff, int width, int decimal);   

