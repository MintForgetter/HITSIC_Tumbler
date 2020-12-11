#ifndef _IMAGE_H
#define _IMAGE_H
#include <stdio.h>
#include <stdlib.h>
//#include "cv.h"
//#include "highgui.h"
#include <math.h>
#include "image.h"

#define MISS 255
#define CAMERA_H  120                            //图片高度
#define CAMERA_W  188                            //图片宽度
#define FAR_LINE 1//图像处理上边界
#define NEAR_LINE 113//图像处理下边界
#define LEFT_SIDE 0//图像处理左边界
#define RIGHT_SIDE 187//图像处理右边界
#define MISS 255
#define white_num_MAX 10//每行最多允许白条数

/////////////////////////////
#define black 0
#define white 1
#define blue  2
#define green 3
#define red   4
#define gray  5
#define purple 6
///////////////////////////

extern uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
extern uint8_t image_Buffer_0[CAMERA_H][CAMERA_W];
extern uint8_t* fullBuffer;//指向灰度图的首地址

void head_clear(void);
//void THRE(void);
int find_f(int a);
void search_white_range();
void find_all_connect();
void find_road();
uint8_t find_continue(uint8_t i_start, uint8_t j_start);
void ordinary_two_line(void);
uint8_t image_main(uint8_t midline);
void get_mid_line(void);

void my_memset(uint8_t* ptr, uint8_t num, uint8_t size);
void fix_un();
float K_line(int Begin, int Final, int sign);
int IsRoadSide(int x, int y, int h);
void head_clear_fix();
void midline_fix();
int IsCross(int x, int y, int Num, int Sign);
int Black_White();
void Protect();
void Zc_double();
int midline_wrong(int midline);
int Cross_begin();
int zc_mid(uint8_t midline);
int IsWide(int begin,int sign);
void find_K(int sign);
#endif //
