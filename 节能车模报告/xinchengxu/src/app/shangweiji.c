#include "shangweiji.h"
#include "common.h"
#include "uart.h"
#define UartDataNum      17
uint8  uSendBuf[UartDataNum*2]={0};
uint8 FreeCarsDataNum=UartDataNum*2;

void push(uint8 chanel,uint16 data)
{
     uSendBuf[chanel*2]=data/256;
     uSendBuf[chanel*2+1]=data%256;
}

void sendDataToScope(void)
{
  uint8 i,sum=0;
  Uart_SendChar(UARTR2,251);
  Uart_SendChar(UARTR2,109);
  Uart_SendChar(UARTR2,37);
  sum+=(251);      
  sum+=(109);
  sum+=(37);
   for(i=0;i<FreeCarsDataNum;i++)
  {
    Uart_SendChar(UARTR2,uSendBuf[i]);
    sum+=uSendBuf[i];         
  }
  Uart_SendChar(UARTR2,sum);
}
/*
void CRC16(unsigned char *Array,unsigned char *Rcvbuf,unsigned int Len)//串口示波器协议
{
  unsigned int  IX,IY,CRC;
  CRC=0xFFFF; //set all 1
  if (Len<=0)CRC= 0;
  else{
   Len-- ;
  for  (IX=0;IX<=Len;IX++)
  {
  CRC=CRC^ (unsigned int) (Array [IX] );
  for(IY=0; IY<=7; IY++)
  if ((CRC&1) !=0) CRC= (CRC>>1)^0xA001;
  else CRC=CRC>>1;  //
  }
  }
Rcvbuf [0]= (CRC & 0xff00) >>8 ;//高位置
Rcvbuf[1]= (CRC & 0x00ff) ;  //低位置
  }

void sendscope(short int ch1,short int ch2,short int ch3,short int ch4)//示波器发送函数
{
      short int temp;
      uint8 buf[10]={0};
      buf[0]=ch1&0xff;
      buf[1]=ch1>>8;
      buf[2]=ch2&0xff;
      buf[3]=ch2>>8;
      buf[4]=ch3&0xff;
      buf[5]=ch3>>8;
      buf[6]=ch4&0xff;
      buf[7]=ch4>>8;
     CRC16(buf,&buf[8],8);

     putbuff(UARTR2,buf,10);
}
*/