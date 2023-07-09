#include "include.h"

extern struct dianganADC adc_1,adc_2,adc_3,adc_4,adc_5,adc_6;     //С���ظ�������
extern float d1,e1,f1,d,e,f;
extern char yejing_fig;
uint16 adc_buf[10];
int16 hope_speed_zhidao=0,hope_speed_wandao=0,hope_speed_yuanhuan=0,hope_speed_shizi=0;
extern uint8 aget[3];
void peripheral_init(void)
{                //�������ڳ�ʼ��       ��ʼ��uartr2,��ӳ������PTI1 TX ,PTI0 RX ����Ƶ��9600
    uart_init(UARTR2,Remap,9600);
    
                 //�ɻɹ��յ���       0����1���   0�͵�ƽ1�ߵ�ƽ ʹ��Ԥ��io��ad7 b2
    gpio_init(PTB2, 0, 1);
      
                 //���뿪��
    gpio_init (PTH4,GPI,HIGH);
    gpio_init (PTC0,GPI,HIGH);
    gpio_init (PTC1,GPI,HIGH);
    gpio_init (PTI5,GPI,HIGH);
                 //���뿪��û���ⲿ����������ʹ���ڲ�����
    PORT->PUE2 |= (uint32)(1<<PTn(PTH4));//ʹ���ڲ�����4
    PORT->PUE2 |= (uint32)(1<<PTn(PTC0));//ʹ���ڲ�����3
    PORT->PUE1 |= (uint32)(1<<PTn(PTC1));//ʹ���ڲ�����2
    PORT->PUE1 |= (uint32)(1<<PTn(PTI5));//ʹ���ڲ����� 1    
    
                    //�����ʼ��
    FTM_PWM_init(CFTM2, FTM_CH1,FTM_PTH1, 12500, 0);//PWM2 PTH1
    FTM_PWM_init(CFTM2, FTM_CH2,FTM_PTD0, 12500, 0);//PWM2 PTD0
                    //�����ʼ��
    FTM_PWM_init(CFTM1, FTM_CH1,FTM_PTE7, 100, 100);//PWM1 PTE7    
                    //��������ʼ��
    FTM_count_init(CFTM0);
     
                    //adc��ʼ��
     ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD4,ADC_12BIT);
     ADC_Init(ADC_CHANNEL_AD5,ADC_12BIT);
     
                    //��ʱ���ж�
     PIT_Init(PIT_CHANNEL0,1);//��ʱ1MS
     PIT_Init(PIT_CHANNEL1,500);//��ʱ500MS   
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
   
    adc_1.max=620;     //���Ըĸ��ݷŴ��� ��
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