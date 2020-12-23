#ifndef ADS_H
#define ADS_H
#define SampleTimes 20
#define Num_remove 3  //滤波时去除的上下点数
#define Num_AD 8  //用于采集的电感的个数
#define MinLVGot 5  //阙值，小于则赋为此值
#include "sc_adc.h"
#include<stdint.h>
#include<math.h>
uint32_t adc_allget(int num);  ////获取电感数值
void LV_get(void);  //获取flag赋值为1的电感值
void LV_cal(void);  //采集数据处理
void bubble_sort_quicker(uint32_t arr[SampleTimes], int n);  //冒泡排序
void Protect(void);  //丢线保护
float ADC_main(void);   //电磁主函数
void Normal(void);  //归一化操作
#endif
