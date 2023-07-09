#include "isr.h"
#include "common.h"
#include "include.h"

uint8 TIME0flag_2ms  = 0 ;


uint16  TimeCount = 0 ;
uint16  TimeCount1 = 0 ;
uint16  TimeCount2 = 0 ;
uint16  TimeCount3 = 0 ;
uint16  TimeCount4=0;
uint16  TimeCount5=0;


extern int within_fig,shizhi_fig,ramp_up;
extern float m_mpc,m_mic,m_mdc;

uint16 num_num=0, num_maxfig_1=0,num_maxfig_2=0,interrupt_fig=0,interrupt_fig_1=0;
uint16 adc_1_ADCmax[10]={0},adc_2_ADCmax[10]={0};
uint8 adc_buffer[5]={0};
uint8 num222=0;
extern uint8 Flag_Round,Turn_flag;
char data_receive[20];



//����0�����жϷ�������
void UART0_ISR(void)
{
  uint8_t ReData;
  uint8_t txt[20];
  
  DisableInterrupts ;//�����ж�
  
  ReData = Uart_GetChar(UARTR0);
  sprintf((char*)txt,"UART0_RX: %c \n",ReData);  
  Uart_SendString(UARTR0,txt);
  
  EnableInterrupts;   //�����ж�
}


//����1�����жϷ�������
void UART1_ISR(void)
{
  uint8_t ReData;
  uint8_t txt[20];
  
  DisableInterrupts ;//�����ж�
  
  ReData = Uart_GetChar(UARTR1);
  sprintf((char*)txt,"UART1_RX: %c \n",ReData);  
  Uart_SendString(UARTR1,txt);
  
  EnableInterrupts;   //�����ж�
}

//����2�����жϷ�������
void UART2_ISR(void)
{
  uint8 ch;
  
   ch = Uart_GetChar(UARTR2);
   data_receive[num222]=ch;
   num222++;
   if(data_receive[0]=='#')  //��ʶͷ
    {   
      if(ch=='$')
      {  if(data_receive[1]=='P')  //�����ֵ
         { 
           m_mpc=(data_receive[2]-48)*100+(data_receive[3]-48)*10+(data_receive[4]-48);
         }
         if(data_receive[1]=='I')  //�ٶȲ���
         {  m_mic= (data_receive[2]-48)*100+(data_receive[3]-48)*10+(data_receive[4]-48);
         }
         if(data_receive[1]=='D')  // ����CCD�ϴ�
         { 
           m_mdc=(data_receive[2]-48)*100+(data_receive[3]-48)*10+(data_receive[4]-48);
         }       
         for(num222=0;num222<10;num222++)
           data_receive[num222]=0x00;
         num222=0;
       }
       if(num222>12)
         num222=0;
    }
    else
    {  for(num222=0;num222<10;num222++)
         data_receive[num222]=0x00;
       num222=0;
    } 
}

//��ʱ��0�жϺ���
void PIT0_ISR(void)
{
  PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;//����жϱ�־λ
      TimeCount ++;
   
      if(Flag_Round==0)
    {
      TimeCount4++;
      if(TimeCount4 % 5000==0)
      {
        Flag_Round=1;
        TimeCount4=0;
      }  
    }
    
     if( Turn_flag==1)
    {
      TimeCount5++;
      if(TimeCount5 % 500==0)
      {
        Turn_flag=0;
        TimeCount5=0;
      }  
    }
   

  if(TimeCount%1 == 0 )
   {
       /*��ô���������*/
     Get_sensor_values();  
       /*ͣ�����*/
     stop_detec();     
   }

   if(TimeCount%2 == 0 )
    {

      /*���ݴ���,�ж�����*/
     data_processing();
 
    } 
  
  if(TimeCount%3== 0 )
    {  
       /*�������*/
      steering_control(); 
    } 
   
  if(TimeCount%4 == 0 )
    {  
      if(PTH4_IN==0)
        TIME0flag_2ms = 0;
    else
        TIME0flag_2ms = 1; 
     
    }
  if(TimeCount%5== 0 )
    {  
        
        /*��ȡ�ٶ�*/
        get_maichong(); 
      
        /*�������*/
        motor_control();
   
    }   
   
  if(TimeCount == 8000) 
  {
    TimeCount = 0 ;
    stop_fig=0;   //ͣ����־
  }  
  
}

//��ʱ��1�жϺ���
void PIT1_ISR(void)
{ 
  PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;//����жϱ�־λ
  if(ramp_up==1)
    {
      TimeCount2++;
      if(TimeCount2 % 5==0)
      {
        ramp_up=0;
        TimeCount2=0;
      }  
    }
}



//KBI0�жϺ���
void KBI0_Isr(void)	
{  
  KBI0->SC |= KBI_SC_KBACK_MASK;       /* clear interrupt flag */
  uint16_t n = PTn(KBI_PTB5) ;   //PTA0���Ŵ����ж� 
  if(KBI0->SP &(1<<n))
  {
    //�û����� 
    LED_Ctrl(LED0, LEDRVS);             
  } 
}

//KBI1�жϺ���
void KBI1_Isr(void)	
{  
  KBI1->SC |= KBI_SC_KBACK_MASK;                /* clear interrupt flag */
  
  uint16_t n = PTn(KBI_PTH2) ;   //PTH2���Ŵ����ж� 
  if(KBI1->SP &(1<<n))
  {
    //�û����� 
    LED_Ctrl(LED1, LEDRVS);             
  }
}





/*************************************************************************
*                             �º�
*
*  �������ƣ�HardFault_Handler
*  ����˵����Ӳ���Ϸ��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-4    �Ѳ���
*  ��    ע��������LED��˸��ָʾ������Ӳ���Ϸ�
*************************************************************************/
void HardFault_Handler(void)
{
    while (1)
    {
        printf("\n****Ӳ���Ϸô���!!!*****\r\n\n");
    }
}




/*****************************************************************************//*!
*
* @brief  FTM0_Isr interrupt service routine.
*        
* @param  none.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM0_IRQHandler(void)
{
  /* clear the flag */
  FTMx[CFTM0]->SC &= ~FTM_SC_TOF_MASK;
  
}

/*****************************************************************************//*!
*
* @brief  FTM1_Isr interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM1_IRQHandler(void)
{
  /* clear the flag */
  FTMx[CFTM1]->SC &= ~FTM_SC_TOF_MASK;
}

/*****************************************************************************//*!
*
* @brief  FTM2_Isr interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/

void FTM2_IRQHandler(void)
{
  /* clear the flag */
  FTMx[CFTM2]->SC &= ~FTM_SC_TOF_MASK;
}
