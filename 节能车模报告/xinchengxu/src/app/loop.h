#ifndef __LOOP_H_
#define __LOOP_H_

#define  MIDSTRING    9750      //�����ֵ
#define  left_limit    8500     //�������ֵ
#define  right_limit  11000      //����Ҽ���ֵ
extern float DirectionError[2];
struct dianganADC
{
	uint16 max;            //���ֵ
	uint16 min;            //��Сֵ      
        uint16 ADresult[20];    //��������
	uint16 data;
        uint16 hubuzhi;
        uint16 display;
        uint16 init[500];
};


typedef struct
{

 uint16 ad_mid_val[10]; //AD������ֵ 
 uint32 ad_add_val[10]; //AD������ֵ��
 uint16 ad_avr_val[10]; //AD����ƽ��ֵ
 uint16 ad_max_val[10]; //AD���ֵ
 uint16 ad_max_temp[10];
 uint16 ad_int_val[10][3];
 uint16 ad_avr_temp[5][10];
}ADC_info;

void XH_loop(void);
void stop_detec();
void Get_sensor_values();
void data_processing();
void speed_analy();
void get_maichong();
#endif  //__INIT_H_