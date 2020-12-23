#ifndef _PID_H_
#define _PID_H_
#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "sc_host.h"

void PID_Init(void);
void PID_MenuInit(menu_list_t *menuList);
void Updatering(struct PID* pid_ring,float parameter,int sign);
void Ang_Init(void);
void Ang_Filter(uint32_t time);
void Ang_ring(void);
void Spd_ring(void);
void Spd_Filter(void);
void Dir_ring(void);
void PWM(float pwm_L,float pwm_R);
void Wifi(void);
void Stop(void);
void Start_init(void);

struct PID
{
    float kp,ki,kd;
    float Curr,Prev,Diff,Inte;
};

extern inv::mpu6050_t imu_6050;
#define PI  3.1415926f
#define ANGS  5U ///<直立环更新时间
#define SPDS  20U
#define DIRS  5U
#define ADCS  5U
#define G   9.4f ///<重力加速度
#define To_ang(x)     (x * (180.0f / PI)) ///<弧度转角度
#define To_rad(x)     (x*(PI / 180.0f))///<弧度转角度
#define FTMVALUE      5660.0f
#define FILWINDOW      200.0f
#define FILCOUNT       (int)(FILWINDOW/SPDS)

#endif //!_PID_H_

