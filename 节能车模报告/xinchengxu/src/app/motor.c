#include "include.h"
#define  MIDSTRING  9750      //舵机中值
#define  chu_2  1500  //弯道
extern char fig,within_fig,devi_fig;//appear_fig,
extern uint8 middle_value,criti_value;
extern int AD[8];
float pwm,kdc=-1.0f;
float Critil_devia=0;
float Kp;                     //最终PD
float Kd;
int angle_p,angle_p1;//36            //普通P基准值     适应全局
int angle_d;//6
int m3=3;     //偏差大小舵机P调整      加强P效果
int m4=9;     //偏差变化率舵机动态D    加强D效果
int m5=40;
extern int distence_sum_weight,distence,biaozhi;
extern int distence_store[30];
extern int left_right_flag,left_right_flag_old,shizhi_fig,straight_fig,jiashi,swan_fig;
extern char chuandao_flag;
extern int zhidao_flag,position,left_right_flag1,cross_counter,dayuanhuanflag;
int Ramp_up_flag=0;
int Ramp_down_flag=0;
uint16 add_dat;
//电机参数
float error_motor1[5],err2_err,err3_err;  //改动float
float last_err2err;    //改动float
float m_mpc=0.0,m_mic=0.0,m_mdc=0.0;
extern float hope_speed_zhidao,hope_speed_youwandao,hope_speed_yuanhuan,hope_speed_shizi;
float left_pwm_num=0.0;
float hope_speed;
float SpeedPWMKP=0,SpeedPWMKI=0,SpeedPWMKD=0;
uint16 motor_pwm3=0;
extern float DirectionControl_Out;
float dir_Control_D ;
float dir_Control_P ;
float DirectionControl_Out;
extern float DirectionError_dot[2]; 
extern float DirectionError[2];
float DirControl_P_zhi ;
float DirControl_D_zhi ;
float DirControl_P_wan ;
float DirControl_D_wan ;
float DirControl_P_huan ;
float DirControl_D_huan ;
extern uint8 Flag_Round,Turn_flag;
int16 abs(int16 x)
{
    if(x<0)  return - x;
    else     return x;
}




/************舵机控制******************/
void steering_control(void)                 
{  
   if(Flag_Round==0)
  {
      dir_Control_P=DirControl_P_huan ;
    
      dir_Control_D= DirControl_D_huan ;
  }
  else
  {
    if(left_right_flag==1)
    {
      dir_Control_P=DirControl_P_wan ;
      dir_Control_D=DirControl_D_wan ;
    }
    else if(left_right_flag==2)
    {
      dir_Control_P=DirControl_P_zhi ;
      dir_Control_D=DirControl_D_zhi ;  
    }
  }
 
  if( Turn_flag )
  {
    DirectionControl_Out = DirectionError[0]*980 + DirectionError_dot[0]*605;
    pwm= DirectionControl_Out ;
  }
  else
  {  
    DirectionControl_Out = DirectionError[1]*dir_Control_P + DirectionError_dot[1]*dir_Control_D;
    pwm=DirectionControl_Out ;
  }
             
   if(pwm>1250) 
     pwm=1250;
   if(pwm<-1250)
     pwm=-1250;
  FTM_PWM_Duty(CFTM1, FTM_CH1,MIDSTRING+(int)pwm);
}

/*****************电机控制******************/

void motor_control(void)  
{
  
 
  if(AD[6]>70)
  {
    //直道速度  
     if(left_right_flag==2)  
       {
         hope_speed=120;
       }
    //弯道速度
      if(left_right_flag==1)
       {
         hope_speed=100;
       }
  }
  
    if(AD[6]>50)
  {
    //直道速度  
     if(left_right_flag==2)  
       {
         hope_speed=110;
       }
    //弯道速度
      if(left_right_flag==1)
       {
         hope_speed=90;
       }
  }
  
    if(AD[6]<50)
  {
    //直道速度  
     if(left_right_flag==2)  
       {
         hope_speed=90;
       }
    //弯道速度
      if(left_right_flag==1)
       {
         hope_speed=70;
       }
  }
  

  
  
     Servo_count_1[0]=Servo_count_1[0]*0.4+Servo_count_1[1]*0.3+Servo_count_1[2]*0.2+Servo_count_1[3]*0.1;
     
     for(uint8 i=3;i>0;i--)
      {
        error_motor1[i]=error_motor1[i-1];       
      } 
     
     error_motor1[0]=hope_speed- Servo_count_1[0];      //偏差=期望值-当前值；      
     err2_err=error_motor1[0]-error_motor1[1];          //偏差的偏差=现在的偏差-上次的偏差；      
     err3_err=err2_err-last_err2err;                    //这一次偏差的偏差和前一次偏差的偏差的差       
     last_err2err=err2_err;                             //存储偏差的偏差
    
                                                    
    SpeedPWMKP = m_mic* err2_err;           //KP 
  
    SpeedPWMKI = m_mpc* error_motor1[0];   //KI  
      
    SpeedPWMKD = m_mdc* err3_err;          //KD 
 
   
   motor_pwm3+= (int16)(SpeedPWMKD+SpeedPWMKI+SpeedPWMKP) ;
 

   if(motor_pwm3>50000)  
      motor_pwm3=50000;
   else if(motor_pwm3<1)
      motor_pwm3=1; 
  

   
   FTM_PWM_Duty(CFTM2, FTM_CH0,(uint16)motor_pwm3);
   FTM_PWM_Duty(CFTM2, FTM_CH1,0);


/*
  FTM_PWM_Duty(CFTM2, FTM_CH0,30000);
  FTM_PWM_Duty(CFTM2, FTM_CH1,0); 
 */ 
}









