#ifndef _headfile_h
#define _headfile_h

#include <stdarg.h>
#include <stdio.h>
#include <SKEAZ1284.h>
#include "sysinit.h"
#include "systick.h"
#include "common.h"
#include "ics.h"
#include "rtc.h"
#include "io.h"
#include "uart.h"
#include "sim.h"
#include "gpio.h"       //IO口操作
#include "pit.h"
#include "adc.h"
#include "uart.h"
#include "ftm.h"
#include "kbi.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "acmp.h"
#include "mscan.h"
#include "pwt.h"
#include "wdog.h"
#include "pmc.h"
#include "adc.h"
#include "adc.h"


//用户自定义头文件   
#include "LQ12864.h"
#include "isr.h"   
#include "LQLED.h"
#include "LQKEY.h"  
#include "LQ9AX.h" 
#include "LQI2C.h"
#include "MPUIIC.h"
#include "MPU6050.h"
#include "Serial_oscilloscope.h"
#include "8700_2100.h"


#include  "init.h"      //外设初始化
#include  "loop.h"
#include  "shangweiji.h"
#include  "motor.h"     //电机模块


extern uint8 TIME0flag_2ms   ;
extern char angel_duty;
extern float kpc,kdc;
extern float position1[100],PianCha[5],ADx1[4];
extern uint16 positiony;
extern float kpc,kdc,jj,kk,dd,zz,kdw;
extern float m_mpc,m_mic,m_mdc;
extern int chu,sss,s2,s3;
extern float Servo_count_1[30],Servo_count[10] ;
extern float Servo_count_sum1,Servo_count_sum2,Servo_count_sum3,Servo_count_sum4;
extern float error_motor1[5],err2_err,er3_err;
extern int  motor_pwm;
extern float hope_speed_zhidao,hope_speed_youwandao;
extern  uint8 stop_fig;






#endif