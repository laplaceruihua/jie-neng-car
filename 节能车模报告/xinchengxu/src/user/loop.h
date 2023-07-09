#ifndef __LOOP_H_
#define __LOOP_H_

#define  MIDSTRING    100      //舵机中值
#define  left_limit   80       //舵机左极限值
#define  right_limit  120      //舵机右极限值
#define  LIN_COUT     20000    //编码器转换系数

struct dianganADC
{
	uint16 max;            //最大值
	uint16 min;            //最小值      
        uint16 ADresult[20];    //储存数据
	uint16 data;
        uint16 hubuzhi;
        uint16 display;
        uint16 init[500];
};


typedef struct
{

 uint16 ad_mid_val[10]; //AD采样中值 
 uint32 ad_add_val[10]; //AD采样中值和
 uint16 ad_avr_val[10]; //AD采样平均值
 uint16 ad_max_val[10]; //AD最大值
 uint16 ad_max_temp[10];
 uint16 ad_int_val[10][3];
 uint16 ad_avr_temp[5][10];
}ADC_info;

void XH_loop(void);
void delay_ms(uint16 MS);
void stop_detec();
void Get_sensor_values();
void data_processing();
void speed_analy();
void get_maichong();
#endif  //__INIT_H_