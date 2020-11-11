/*
 * pid.h
 *
 *  Created on: 2020年11月12日
 *      Author: Dell
 */

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

void PID_MenuInit(menu_list_t *menuList);
void PID_FilterInit(void);
void PID_FilterUpdate(uint32_t updatetime);
float Caculate_output(void);
void UpdateAng(struct pidCtrl_t* pid_angFilter,float increangle);
void Angring(void);
void Return_Ang(void);
void PWM(void);

extern inv::mpu6050_t imu_6050;
#define PI  3.1415926f
#define ANGS  5U
#define G   9.80f
#define To_rad(x)     (x * (PI / 180.0f))
#define To_ang(x)     (x * (180.0f / PI))


#endif //!_PID_H_
