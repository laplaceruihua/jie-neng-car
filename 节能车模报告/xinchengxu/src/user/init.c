#include "include.h"

extern struct dianganADC adc_1,adc_2,adc_3,adc_4,adc_5,adc_6;     //小心重复函数名
extern float d1,e1,f1,d,e,f;
extern char yejing_fig;
uint16 adc_buf[10];
int16 hope_speed_zhidao=0,hope_speed_wandao=0,hope_speed_yuanhuan=0,hope_speed_shizi=0;
extern uint8 aget[3];
void peripheral_init(void)
{                //蓝牙串口初始化       初始化uartr2,重映射引脚PTI1 TX ,PTI0 RX 串口频率9600
    uart_init(UARTR2,Remap,9600);
    
                 //干簧管终点检测       0输入1输出   0低电平1高电平 使用预留io口ad7 b2
    gpio_init(PTB2, 0, 1);
      
                 //拨码开关
    gpio_init (PTH4,GPI,HIGH);
    gpio_init (PTC0,GPI,HIGH);
    gpio_init (PTC1,GPI,HIGH);
    gpio_init (PTI5,GPI,HIGH);
                 //拨码开关没有外部上拉，所以使能内部上拉
    PORT->PUE2 |= (uint32)(1<<PTn(PTH4));//使能内部上拉4
    PORT->PUE2 |= (uint32)(1<<PTn(PTC0));//使能内部上拉3
    PORT->PUE1 |= (uint32)(1<<PTn(PTC1));//使能内部上拉2
    PORT->PUE1 |= (uint32)(1<<PTn(PTI5));//使能内部上拉 1    
    
                    //电机初始化
    FTM_PWM_init(CFTM2, FTM_CH1,FTM_PTH1, 12500, 0);//PWM2 PTH1
    FTM_PWM_init(CFTM2, FTM_CH2,FTM_PTD0, 12500, 0);//PWM2 PTD0
                    //舵机初始化
    FTM_PWM_init(CFTM1, FTM_CH1,FTM_PTE7, 100, 100);//PWM1 PTE7    
                    //编码器初始化
    FTM_count_init(CFTM0);
     
                    //adc初始化
     ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD4,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD5,ADC_12BIT);
     
                    //定时器中断
     PIT_Init(PIT_CHANNEL0,1);//定时1MS
     PIT_Init(PIT_CHANNEL1,500);//定时500MS   
}





void speed_init(void)
{
  
   if(PTH4_IN==0)      
   {
      hope_speed_zhidao=115;//pwm
      hope_speed_wandao=85;
      hope_speed_yuanhuan=70;   
   } 

   if(PTC0_IN==0)  
   {
      hope_speed_zhidao=115;
      hope_speed_wandao=85;
      hope_speed_yuanhuan=70; 
   }
   
   if(PTC1_IN==0)    
   {      
      hope_speed_zhidao=115;
      hope_speed_wandao=85;
      hope_speed_yuanhuan=70;
   }
   
   if(PTI5_IN==0)
   {
      hope_speed_zhidao=115;
      hope_speed_wandao=85;
      hope_speed_yuanhuan=70;
   } 
}

void parameter_init(void)
{
   speed_init();
   
   m_mic=90.0;  
   m_mpc=30.0;  
   m_mdc=5.0;
   
    adc_1.max=620;     //可以改根据放大倍数 改
    adc_2.max=610;
    adc_3.max=610;
    adc_4.max=620;
    adc_5.max=615;
    adc_6.max=615;

    adc_1.min=7;
    adc_2.min=7;
    adc_3.min=7;
    adc_4.min=7;
    adc_5.min=7;
    adc_6.min=7;    
    
}