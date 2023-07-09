#include "include.h"
#define  MIDSTRING  100      //�����ֵ
#define  chu_2  1500  //���
extern char fig,within_fig,devi_fig;//appear_fig,
extern uint8 middle_value,criti_value; 

float pwm,kdc=-1.0f;

float Critil_devia=0;
float Kp;                     //����PD
float Kd;
int angle_p,angle_p1;//36            //��ͨP��׼ֵ     ��Ӧȫ��
int angle_d;//6
int m3=3;     //ƫ���С���P����      ��ǿPЧ��
int m4=9;     //ƫ��仯�ʶ����̬D    ��ǿDЧ��
int m5=40; 
extern int distence_sum_weight,distence,biaozhi;
extern int distence_store[30];
extern int left_right_flag,left_right_flag_old,shizhi_fig,straight_fig,jiashi,swan_fig;
extern char chuandao_flag;
extern int zhidao_flag,position,left_right_flag1,cross_counter;
int Ramp_up_flag=0;
int Ramp_down_flag=0;
uint16 add_dat;
//�������
float error_motor1[5],err2_err,err3_err;  //�Ķ�float
float last_err2err;    //�Ķ�float
int motor_pwm=0;    //�Ķ�float
float m_mpc=0.0,m_mic=0.0,m_mdc=0.0;
extern int16 hope_speed_zhidao,hope_speed_wandao,hope_speed_yuanhuan,hope_speed_shizi;
float left_pwm_num=0.0;
int16 hope_speed;
float SpeedPWMKP=0,SpeedPWMKI=0,SpeedPWMKD=0;
int16 motor_pwm3=0;


int16 abs(int16 x)
{
    if(x<0)  return - x;
    else     return x;
}




/************�������******************/
void steering_control(void)                 
{   

      angle_p=45;
      angle_d=330;  
     
   if(left_right_flag==2)
    { 
      angle_p=35;  
      angle_d=10;
    }

   if(cross_counter==1)
    {
      angle_p=55; 
      angle_d=330;
    }

      
   Kp=angle_p+(float)(abs(distence_sum_weight)*m3/100);   
   Kd=angle_d+(float)(abs(distence_store[0]-distence_store[15])*m4/100)+30*m5/(Kp+10);    
   pwm=(int16)((distence_sum_weight*Kp/10+(distence_store[0]-distence_store[10])* Kd/10)); 
 

                  
   if(pwm>20) 
      pwm=20;
   if(pwm<-20)
      pwm=-20;

  FTM_PWM_Duty(CFTM1, FTM_CH1,MIDSTRING-(int)pwm);
 
}

/*****************�������******************/

void motor_control(void)  
{
  
//ֱ���ٶ�  
  if(left_right_flag==2||within_fig==1)  
   {
     hope_speed=hope_speed_zhidao;
   }
//����ٶ�
  if(left_right_flag==3||left_right_flag==1)
   {
     hope_speed=hope_speed_wandao;
   }
  
 //Բ��
  if(cross_counter==1)
   {
     hope_speed=hope_speed_yuanhuan;
   } 
      
     Servo_count_1[0]=Servo_count_1[0]*0.4+Servo_count_1[1]*0.3+Servo_count_1[2]*0.2+Servo_count_1[3]*0.1;
     
     for(uint8 i=3;i>0;i--)
      {
        error_motor1[i]=error_motor1[i-1];       
      } 
     
     error_motor1[0]=hope_speed- Servo_count_1[0];      //ƫ��=����ֵ-��ǰֵ��      
     err2_err=error_motor1[0]-error_motor1[1];          //ƫ���ƫ��=���ڵ�ƫ��-�ϴε�ƫ�      
     err3_err=err2_err-last_err2err;                    //��һ��ƫ���ƫ���ǰһ��ƫ���ƫ��Ĳ�       
     last_err2err=err2_err;                             //�洢ƫ���ƫ��
    
                                                    
    SpeedPWMKP = m_mpc*err2_err;           //KP 
  
    SpeedPWMKI = m_mic* error_motor1[0];   //KI  
      
    SpeedPWMKD = m_mdc* err3_err;          //KD 
 
   
   motor_pwm3+= (int16)(SpeedPWMKD+SpeedPWMKI+SpeedPWMKP) ;
     
  
   if(motor_pwm3>300)  
      motor_pwm3=300;
   else if(motor_pwm3<0)
      motor_pwm3=0; 
   
   FTM_PWM_Duty(CFTM2, FTM_CH1,0);
   FTM_PWM_Duty(CFTM2, FTM_CH2,(uint16)motor_pwm3);

}









