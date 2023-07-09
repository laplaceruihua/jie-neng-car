#include "include.h"
uint8 stop_fig=0;//num,app_pang_fig, 
char adc_1_ADresult[2];
extern float pwm;
struct dianganADC adc_1,adc_2,adc_3,adc_4,adc_5,adc_6;
extern uint8 LPT_INT_count ;
float Servo_count_1[30] ={0.0},Servo_count_sum3=0.0,Servo_count_sum4=0.0;
float Servo_count[10] ={0.0},Servo_count_sum1=0.0,Servo_count_sum2=0.0;
extern uint8 adc_buf[10];
int shizhi_fig=0,position=0,cross_counter=0,within_fig=0,jiashi=0,xushi_fig=0,swan_fig=0;
 
extern int angle_p,angle_d;
ADC_info  Adc = {0};
int left_right_flag,biaozhi1=0,ramp_up=0;      //�����ж�        12/3
int distence_store[30],AD[8];   //�����Ż����,�Ż��ٶ�   ȡ50��Ϻá�   ǰհ50cm�� 3m/s,   ��166ms��5msԼ33���ȡ30��    1/3��ǰհ����ʶ�����
int distence_sum_weight,sub5_6,sub2,sub3;//,sum14_56
int distence,distence_old;//,left_right_flag1,left_right_flag2,left_right_flag3
char chuandao_flag=0;
uint32 stopcount=0;
extern int16 motor_pwm3;
/******************************/

int16 abs1(int16 x)
{
    if(x<0)  return - x;
    else     return x;
}

void time_delay_ms(uint16_t ms)   //д���ٱ��Ƕ���
{  
  while(ms--)
  {
    delay_us(1000);
  }
}

void uniformization(void)//��һ��
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
    
    
    //��һ������  ʹ֮����100�����Ա㴦��
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
  {Adc.ad_add_val[i]=0;}  //�ȶ���ͽ�������

  for(i = 0; i <3 ;i++)
  {
    Adc.ad_avr_temp[i][1] = adc_ave(ADC_CHANNEL_AD5,ADC_10BIT,6);   
    Adc.ad_avr_temp[i][2] = adc_ave(ADC_CHANNEL_AD13,ADC_10BIT,6);    
    Adc.ad_avr_temp[i][3] = adc_ave(ADC_CHANNEL_AD4,ADC_10BIT,6);  
    Adc.ad_avr_temp[i][4] = adc_ave(ADC_CHANNEL_AD12,ADC_10BIT,6);  
    Adc.ad_avr_temp[i][5] = adc_ave(ADC_CHANNEL_AD15,ADC_10BIT,6);    
    Adc.ad_avr_temp[i][6] = adc_ave(ADC_CHANNEL_AD14,ADC_10BIT,6); 
  }
  
  
//the next is ��ֵ�˲�
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

   Adc.ad_add_val[i] +=  Adc.ad_avr_temp[1][i]; //��ֵ��
  }
}
 
for(i = 1; i <= 6; i++)
 {
  Adc.ad_avr_val[i] = (uint16)(Adc.ad_add_val[i]*0.333);  //��ֵ
 }
   AD[1]=Adc.ad_avr_val[1];  
   AD[2]=Adc.ad_avr_val[2]; 
   AD[3]=Adc.ad_avr_val[3]; 
   AD[4]=Adc.ad_avr_val[4]; 
   AD[5]=Adc.ad_avr_val[5]; 
   AD[6]=Adc.ad_avr_val[6];                     
   uniformization();   //��һ��             
} 


void data_processing(void)
{
   int16 i;
  // static int left_right_flag_old;
   
    /*��ô���������*/     
   Get_sensor_values();
   
   sub3=AD[1]-AD[4]; 
   
   
   
   
  /************ֱ�������**************/ 
  if(AD[4]>60||AD[1]>60) 
  {    
      if((abs1(sub3)<20)||(AD[5]<20&&AD[6]<20))
      {
        left_right_flag=2;    //ֱ��
       }  
      if(AD[5]>20||AD[6]>20)
      {
          if(AD[2]-AD[3]>5)
             left_right_flag=3;  //��ת
          else if(AD[3]-AD[2]>5)
             left_right_flag=1;   //��ת
       }
   }
 else if(AD[4]<60&&AD[1]<60)    //ֱ��ʮ�ַ�������������б��У���ֱ��У�ˮƽ��У���ͣ�ֵ�ۺϴ���      
  {
       if(AD[2]-AD[3]>5)    
          left_right_flag=3;
       else if(AD[3]-AD[2]>5)
          left_right_flag=1; 
  }

  
    /************ʮ����һ**************/

  if((AD[5]+AD[6])>140)
    {
       left_right_flag=2;
       shizhi_fig=1;
    }
  
     /************����**************/
  
  if(((AD[2]+AD[3])>170)&&((AD[5]+AD[6])<50))
  {
    ramp_up=1;
  }
 
 if(ramp_up==1)
 {
   left_right_flag=2;
 }
 
      /************Բ��**************/
 
 
 
 
 
 
 
 
       /************���ݴ���**************/
 
  if(left_right_flag==2)                           // 10cm
   {
     distence=-(int16)((AD[1]-AD[4])*3/2);   //ֱ�� ���� СS��
   } 
  else if(left_right_flag==1)                                                      // +5/+5/+10    ������
   {
    if(AD[4]>=70)
     {
       distence=-30;
     }
    else if(AD[4]>10&&AD[4]<70)
      {
        distence=-30-(int16)(12000/(50+AD[4])-100)*3; 
      }
    else if(AD[4]<=10)
      {
        distence=-330; 
      }
   }
   
  else if(left_right_flag==3)
   {
      if(AD[1]>=70)
      {
        distence=30;   
      }
      else if(AD[1]>10&&AD[1]<70)
      {
        distence=30+(int16)(12000/(50+AD[1])-100)*3; 
      }
      else if(AD[1]<=10)
      {
        distence=330;
      }
   } 
  distence_old=distence;
      
  for(i=28;i>=0;i--)   //�����ǵݼ���
   {
    distence_store[i+1]=distence_store[i];
   } 
   distence_store[0]=distence_old;
   
   distence_sum_weight=distence_store[0]*15/25+distence_store[1]*4/25+distence_store[2]*3/25+distence_store[3]*2/25+distence_store[4]/25;
}
 
/***********����distence���������������״̬********/
 void speed_analy(void)
{
 int i=0; 
 int32 distence_abs_sum[4]={0,0,0,0};   //����֮�ͣ�ǰ10����10,��10 ÿ����������ڲ�����
 
  for(i=0;i<30;i++)
   {
    distence_abs_sum[0]=distence_abs_sum[0]+abs1(distence_store[i]);
   }
  
  distence_abs_sum[0]=distence_abs_sum[0]/30;

  if(distence_abs_sum[0]<45)
  {
    within_fig=1;
  }
  else
  {
    within_fig=0;
  }   
}


 /***********������********/
void get_maichong(void)//������
{
  
  for(uint8 i=5;i>0;i--)
   {
     Servo_count_1[i]=Servo_count_1[i-1];
   }  
      Servo_count_1[0] =FTM_count_get(CFTM0);                                           //�����������������ֵ
      FTM_count_clean(CFTM0);                                             //����������������ֵ��������գ��������ܱ�֤����ֵ׼ȷ��
      Servo_count_1[0] = (LPT_INT_count * LIN_COUT + Servo_count_1[0]);        //���10ms���������   ��������ת��Ϊpwm
      LPT_INT_count = 0;      
}

/***********ͣ��********/

void stop_detec(void)//ͣ��
{
   if(stop_fig==1)
   {
      if(PTB2_IN==0)   //�ɻɹ�
      {
         stopcount++;
         if(stopcount>2)  //�������Ը�  �ǸɻɹܵĴ�������
         {
            for(uint16 i=0;i<400;i++)
            {                 
              FTM_PWM_Duty(CFTM2, FTM_CH1,200);
              FTM_PWM_Duty(CFTM2, FTM_CH2,0);//�����˫ͨ����������ת   ��תͣ��
                  /*��ô���������*/
                 Get_sensor_values();
                 
                 /*���ݴ���,�ж�����*/
                 data_processing();
          
                 /*�������*/  
                 steering_control();
            }  
           
            for(;;) 
            {
               FTM_PWM_Duty(CFTM2, FTM_CH1,0);
              FTM_PWM_Duty(CFTM2, FTM_CH2,0);
                 /*��ô���������*/
                 Get_sensor_values();
                 
                 /*���ݴ���,�ж�����*/
                 data_processing();
          
                 /*�������*/  
                 steering_control();
             }
         }
      }
   }
}
/******************************/ 
 void XH_loop(void)
{
    /* 50ms����ִ�д����*/
    if(TIME0flag_2ms == 1)
    { 
     Uart_SendChar(UARTR2,0xff);
     push(0,cross_counter);
     push(1,AD[1]);
     push(4,AD[4]);
     push(2,AD[5]);
     push(3,AD[6]);    
     push(5,AD[2]);
     push(6,AD[3]);
     push(7,(int16)pwm);
     push(8,left_right_flag);
     push(9,(uint16)Servo_count_1[0]);
     
     sendDataToScope();  
     TIME0flag_2ms = 0;  
    }    
}   