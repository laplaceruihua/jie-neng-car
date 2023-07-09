#include "include.h"
uint8 stop_fig=0;//num,app_pang_fig, 
char adc_1_ADresult[2];
extern float pwm,hope_speed;
struct dianganADC adc_1,adc_2,adc_3,adc_4,adc_5,adc_6;
float Servo_count_1[30] ={0.0},Servo_count_sum3=0.0,Servo_count_sum4=0.0;
float Servo_count[10] ={0.0},Servo_count_sum1=0.0,Servo_count_sum2=0.0;
extern uint8 adc_buf[10];
int shizhi_fig=0,position=0,cross_counter=0,within_fig=0,jiashi=0,xushi_fig=0,swan_fig=0,dayuanhuanflag=0;
 extern uint8 TIME0flag_50ms ;
extern int angle_p,angle_d;
ADC_info  Adc = {0};
int left_right_flag,biaozhi1=0,ramp_up=0;      //左右判定        12/3
int distence_store[30],AD[8];   //数组优化打脚,优化速度   取50组较好。   前瞻50cm， 3m/s,   花166ms，5ms约33组故取30组    1/3的前瞻即可识别出来
int distence_sum_weight,sub5_6,sub2,sub3;//,sum14_56
int distence,distence_old;//,left_right_flag1,left_right_flag2,left_right_flag3
char chuandao_flag=0;
uint32 stopcount=0;
extern int16 motor_pwm3;
uint8 Flag_Round=1,Turn_flag;
float Pulse_ave[5];
float DirectionError[2]; 
float DirectionError_dot[2]; 
/******************************/

int16 abs1(int16 x)
{
    if(x<0)  return - x;
    else     return x;
}


void uniformization(void)//归一化
{  
 
    if(AD[1] >adc_1.max) AD[1]=adc_1.max;
    else if(AD[1]<adc_1.min) AD[1]=adc_1.min;
    
    if(AD[2] >adc_2.max) AD[2]=adc_2.max;
    else if(AD[2]<adc_2.min) AD[2]=adc_2.min;
     
    if(AD[3] >adc_3.max) AD[3]=adc_3.max;
    else if(AD[3]<adc_3.min) AD[3]=adc_3.min;
    
    if(AD[4] >adc_4.max) AD[4]=adc_4.max;
    else if(AD[4]<adc_4.min) AD[4]=adc_4.min;
    
     if(AD[5] >adc_5.max) AD[5]=adc_5.max;
    else if(AD[5]<adc_5.min) AD[5]=adc_5.min;
    
     if(AD[6] >adc_6.max) AD[6]=adc_6.max;
    else if(AD[6]<adc_6.min) AD[6]=adc_6.min;
    
    
    //归一化处理  使之处于100以内以便处理
    AD[1]=(AD[1]-adc_1.min)*100/(adc_1.max-adc_1.min);
    AD[2]=(AD[2]-adc_2.min)*100/(adc_2.max-adc_2.min);
    AD[3]=(AD[3]-adc_3.min)*100/(adc_3.max-adc_3.min);
    AD[4]=(AD[4]-adc_4.min)*100/(adc_4.max-adc_4.min);
    AD[5]=(AD[5]-adc_5.min)*100/(adc_5.max-adc_5.min);
    AD[6]=(AD[6]-adc_6.min)*100/(adc_6.max-adc_6.min);
}



void Get_sensor_values(void)
{
  uint8 i=0;
  uint8 k=0;
  uint16 temp = 0;   
  for(i=1;i<=6;i++)
  {AD[i]=0;}
  
  for(i=1;i<=6;i++)
  {Adc.ad_add_val[i]=0;}  //先对其和进行清零

  for(i = 0; i <3 ;i++)
  {
    Adc.ad_avr_temp[i][1] = adc_ave(ADC_CHANNEL_AD5,ADC_10BIT,6);   
    Adc.ad_avr_temp[i][2] = adc_ave(ADC_CHANNEL_AD13,ADC_10BIT,6);    
    Adc.ad_avr_temp[i][3] = adc_ave(ADC_CHANNEL_AD4,ADC_10BIT,6);  
    Adc.ad_avr_temp[i][4] = adc_ave(ADC_CHANNEL_AD12,ADC_10BIT,6);  
    Adc.ad_avr_temp[i][5] = adc_ave(ADC_CHANNEL_AD15,ADC_10BIT,6);    
    Adc.ad_avr_temp[i][6] = adc_ave(ADC_CHANNEL_AD11,ADC_10BIT,6); 
  }
  
  
//the next is 中值滤波
 for(k = 0; k < 3; k++)
{ 
  for(i = 1;i <= 6;i++)
  {
   if(Adc.ad_avr_temp[0][i] > Adc.ad_avr_temp[2][i]) 
   {
      temp = Adc.ad_avr_temp[0][i];
      Adc.ad_avr_temp[0][i] = Adc.ad_avr_temp[2][i];
      Adc.ad_avr_temp[2][i] = temp;
   }
   if(Adc.ad_avr_temp[0][i] > Adc.ad_avr_temp[1][i])
   {
      temp = Adc.ad_avr_temp[0][i];
      Adc.ad_avr_temp[0][i] = Adc.ad_avr_temp[1][i];
      Adc.ad_avr_temp[1][i] = temp;
   }
   if(Adc.ad_avr_temp[1][i] > Adc.ad_avr_temp[2][i])
   {
      temp = Adc.ad_avr_temp[1][i];
      Adc.ad_avr_temp[1][i] = Adc.ad_avr_temp[2][i];
      Adc.ad_avr_temp[2][i] = temp;
   }   

   Adc.ad_add_val[i] +=  Adc.ad_avr_temp[1][i]; //中值和
  }
}
 
for(i = 1; i <= 6; i++)
 {
  Adc.ad_avr_val[i] = (uint16)(Adc.ad_add_val[i]*0.333);  //均值
 }
   AD[1]=Adc.ad_avr_val[1];  
   AD[2]=Adc.ad_avr_val[2]; 
   AD[3]=Adc.ad_avr_val[3]; 
   AD[4]=Adc.ad_avr_val[4]; 
   AD[5]=Adc.ad_avr_val[5]; 
   AD[6]=Adc.ad_avr_val[6];                     
   //uniformization();   //归一化             
} 


void data_processing(void)
{
  
   static float DirectionErrorTemp[2][8];
   uint8 sub0,sub1;
   /*获得传感器数据*/     
   Get_sensor_values();
   if(AD[4]==0&&AD[1]==0)
   {
      FTM_PWM_Duty(CFTM2, FTM_CH0,0);
      FTM_PWM_Duty(CFTM2, FTM_CH1,10000);
   }

  sub0=AD[4]+AD[1];
  sub1=AD[2]+AD[5];
  if(sub0==0) sub0=1;
  if(sub1==0) sub1=1;
  
    
  DirectionError[0]=(float)(AD[1]-AD[4])/(sub0);
  DirectionError[0] = (DirectionError[0]>= 1? 1:DirectionError[0]);	//偏差限幅
  DirectionError[0] = (DirectionError[0]<=-1?-1:DirectionError[0]);
  
  DirectionError[1]=(float)(AD[5]-AD[2])/(sub1);
  DirectionError[1] = (DirectionError[1]>= 1? 1:DirectionError[1]);	//偏差限幅
  DirectionError[1] = (DirectionError[1]<=-1?-1:DirectionError[1]);
  
  DirectionErrorTemp[0][4] = DirectionErrorTemp[0][3];
  DirectionErrorTemp[0][3] = DirectionErrorTemp[0][2];
  DirectionErrorTemp[0][2] = DirectionErrorTemp[0][1];
  DirectionErrorTemp[0][1] = DirectionErrorTemp[0][0];
  DirectionErrorTemp[0][0] = DirectionError[0];
  DirectionError_dot[0] = 5*(DirectionErrorTemp[0][0]-DirectionErrorTemp[0][3]);//水平电感的偏差微分
  DirectionError_dot[0] = (DirectionError_dot[0]>1? 1:DirectionError_dot[0]);//偏差微分限幅
  DirectionError_dot[0] = (DirectionError_dot[0]<-1?-1:DirectionError_dot[0]);

  DirectionErrorTemp[1][4] = DirectionErrorTemp[1][3];
  DirectionErrorTemp[1][3] = DirectionErrorTemp[1][2];
  DirectionErrorTemp[1][2] = DirectionErrorTemp[1][1];
  DirectionErrorTemp[1][1] = DirectionErrorTemp[1][0];
  DirectionErrorTemp[1][0] = DirectionError[1];
  DirectionError_dot[1] = 1.5*(DirectionErrorTemp[1][0]-DirectionErrorTemp[1][3]);//斜电感的偏差微分
  DirectionError_dot[1] = (DirectionError_dot[1]> 1? 1:DirectionError_dot[1]);//偏差微分限幅
  DirectionError_dot[1] = (DirectionError_dot[1]<-1?-1:DirectionError_dot[1]);
  
  if((DirectionError[0]>0.1)||(DirectionError[0]<-0.1))
  {
  left_right_flag=1;
  }
  else 
  {
    left_right_flag=2;
  }
  
  /*
   if(AD[5]>37&&Flag_Round)
  {
    Turn_flag=1;
    Flag_Round=0;
  }
  */
}
 /***********编码器********/
void get_maichong(void)//编码器
{
  int16 pulse=0;
   uint8 i;
      Pulse_ave[0] =FTM_count_get(CFTM0);//保存脉冲计数器计算值
      FTM_count_clean(CFTM0); //清空脉冲计数器计算值（马上清空，这样才能保证计数值准确)
        Pulse_ave[4]=Pulse_ave[3];
        Pulse_ave[3]=Pulse_ave[2];
        Pulse_ave[2]=Pulse_ave[1];
        Pulse_ave[1]=Pulse_ave[0];
        for(i=0;i<4;i++)
        pulse+=Pulse_ave[i];
        pulse=pulse/4;
        Servo_count_1[0]=pulse;
        for(uint8 i=5;i>0;i--)
        {
          Servo_count_1[i]=Servo_count_1[i-1];
         }
}

/***********停车********/

void stop_detec(void)//停车
{
   if(stop_fig==1)
   {
      if(PTB2_IN==0)   //干簧管
      {
         stopcount++;
         if(stopcount>2)  //次数可以改  是干簧管的触发次数
         {
            for(uint16 i=0;i<400;i++)
            {                 
              FTM_PWM_Duty(CFTM2, FTM_CH0,35000);
              FTM_PWM_Duty(CFTM2, FTM_CH1,0);//单电机双通道控制正反转   反转停车
                  /*获得传感器数据*/
                 Get_sensor_values();
                 
                 /*数据处理,判断赛道*/
                 data_processing();
          
                 /*舵机控制*/  
                 steering_control();
            }  
          
            for(;;) 
            {
              FTM_PWM_Duty(CFTM2, FTM_CH0,35000);
              FTM_PWM_Duty(CFTM2, FTM_CH1,0);
                 /*获得传感器数据*/
                 Get_sensor_values();
                 
                /*数据处理,判断赛道*/
                 data_processing();
          
                 /*舵机控制*/  
                 steering_control();
             }
         }
      }
   }
}
/******************************/
void XH_loop(void)
{
     
    uint8 i;
    unsigned char  txt_t[30]="X:";

    if(TIME0flag_2ms == 1)
    { 
       Uart_SendChar(UARTR2,0xff);  
       push(0,cross_counter);
       for(i=6;i>0;i--)
       {
         sprintf((char*)txt_t,"AD%d:%04d",i,AD[i]);
         LCD_P6x8Str(0,i,txt_t);
         push(i,AD[i]);
       } 
       push(7,(int16)pwm);
       push(8,left_right_flag);
       push(9,(uint16)Servo_count_1[0]);
       push(10,(int16)hope_speed);
       push(11,(int16)error_motor1[0]);
       push(12,(int16)motor_pwm3);
       
       sendDataToScope();  
       TIME0flag_2ms = 0;  
    } 
    
}   