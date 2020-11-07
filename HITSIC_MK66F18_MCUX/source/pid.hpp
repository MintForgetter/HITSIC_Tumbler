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

void PID_MenuInit(menu_list_t *menuList);
void PID_FilterInit(void);
void PID_Filter(uint32_t time);
float Caculate_output(void);
void UpdateAng(float increangle);
void Angring(void);
void PWM(void);

struct PID
{
    float kp,ki,kd;
    float Cur,Pre,Dif;
};

extern inv::mpu6050_t imu_6050;
#define PI  3.1415926f
#define ANGS  5U ///<更新时间
#define G   9.1f ///<重力加速度
#define To_ang(x)     (x * (180.0f / PI)) ///<弧度转角度


#endif //!_PID_H_

