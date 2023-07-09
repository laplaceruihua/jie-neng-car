#include "include.h"
extern float DirControl_P_zhi ;
extern float DirControl_D_zhi ;
extern float DirControl_P_wan ;
extern float DirControl_D_wan ;
extern float DirControl_P_huan ;
extern float DirControl_D_huan ;
extern struct dianganADC adc_1,adc_2,adc_3,adc_4,adc_5,adc_6;     //С���ظ�������
extern float d1,e1,f1,d,e,f;
extern char yejing_fig;
extern int AD[8];
uint16 adc_buf[10];
float hope_speed_zhidao=0.0,hope_speed_youwandao=0.0,hope_speed_yuanhuan=0.0,hope_speed_shizi=0.0;
extern uint8 aget[3];
void peripheral_init(void)
{                //�������ڳ�ʼ��       ��ʼ��uartr2,��ӳ������PTI1 TX ,PTI0 RX ����Ƶ��9600
    uart_init(UARTR2,Remap,9600);
  
    /*led��ʼ��*/
    
     LED_Init();
    LED_Ctrl(LEDALL, LEDOFF); 
    
    
    
     /* OLED ��ʼ�� */
    LCD_Init();         //oled   
    
                 //�ɻɹ��յ���       0����1���   0�͵�ƽ1�ߵ�ƽ ʹ��Ԥ��io��ad7 b2
    gpio_init(PTB2, 0, 1);
      
                 //���뿪��
    gpio_init (PTH4,GPI,HIGH);
    gpio_init (PTC0,GPI,HIGH);
    gpio_init (PTC1,GPI,HIGH);
    gpio_init (PTI5,GPI,HIGH);  
    
                    //�����ʼ��
  FTM_PWM_init(CFTM2, FTM_CH0,FTM_PTH0, 12500,0);//PWM2 PTH1
  FTM_PWM_init(CFTM2, FTM_CH1,FTM_PTD0, 12500,0);//PWM2 PTD0
                    //�����ʼ��
    FTM_PWM_init(CFTM1, FTM_CH1,FTM_PTE7, 100, 9750);//PWM1 PTE7    
                    //��������ʼ��
    FTM_count_init(CFTM0);
     
                    //adc��ʼ��
     ADC_Init(ADC_CHANNEL_AD12,ADC_10BIT);
     ADC_Init(ADC_CHANNEL_AD13,ADC_10BIT);
     ADC_Init(ADC_CHANNEL_AD14,ADC_10BIT);
     ADC_Init(ADC_CHANNEL_AD15,ADC_10BIT);
     ADC_Init(ADC_CHANNEL_AD4,ADC_10BIT);
     ADC_Init(ADC_CHANNEL_AD5,ADC_10BIT);
     
                    //��ʱ���ж�
     PIT_Init(PIT_CHANNEL0,1);//��ʱ1MS
     PIT_Init(PIT_CHANNEL1,500);//��ʱ500MS   
}





void speed_init(void)
{ 
  DirControl_D_zhi =0;
  DirControl_P_zhi =10000;
  DirControl_D_wan =0;
  DirControl_P_wan =10000;
  DirControl_D_huan=0 ;
  DirControl_P_huan=1010 ;
  
  m_mpc=1100.0;  
  m_mic=0.7*AD[6];  
  m_mdc=20.0;
  
   if(PTH4_IN==0)      
   {
  hope_speed_zhidao=130.0;
  hope_speed_youwandao=80.0;
   m_mic=120.0;  
   m_mpc=30.0;  
   m_mdc=5.0; 
    
   } 

   if(PTC0_IN==0)  
   {
  hope_speed_zhidao=130.0;
  hope_speed_youwandao=80.0;
      
   m_mic=120.0;  
   m_mpc=30.0;  
   m_mdc=5.0; 
   
   }
   
   if(PTC1_IN==0)    
   {      
  hope_speed_zhidao=130.0;
  hope_speed_youwandao=80.0;
      
   m_mic=120.0;  
   m_mpc=30.0;  
   m_mdc=5.0; 
  
   }
   
   if(PTI5_IN==0)
   {
  hope_speed_zhidao=130.0;
  hope_speed_youwandao=80.0;
   m_mic=120.0;  
   m_mpc=30.0;  
   m_mdc=5.0;  
   }
}

void parameter_init(void)
{
   speed_init();
   

   
    adc_1.max=255;     //���Ըĸ��ݷŴ��� ��
    adc_2.max=420;              //175
    adc_3.max=411;              //189
    adc_4.max=255;
    adc_5.max=537;
    adc_6.max=500;

    adc_1.min=0;
    adc_2.min=0;
    adc_3.min=0;
    adc_4.min=0;
    adc_5.min=0;
    adc_6.min=0;    
    
}