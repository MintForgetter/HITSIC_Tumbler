#include "pid.hpp"
/**
 * @brief   IMU传感器原始数据
 * @ {
 */
float pid_accl[3] = {0.0f, 0.0f, 0.0f};
float pid_gyro[3] = {0.0f, 0.0f, 0.0f};
/**
 * @ }
 */

float &pid_acc = pid_accl[0];
float &pid_gyr = pid_gyro[1];

float pid_AngA = 0.0f, pid_AngG = 0.0f; ///< 由加速度计、角速度计测出的角度
float pid_Anginit = 0.0f; ///<初始化角
float Tg = 1.0f / 0.8f; ///<K分之一

/**
 * @brief   角度滤波相关变量。
 * @note    借用结构体保存当前值、历史值、微分值。
 * @ {
 */
struct PID pid_ang =
{
    .kp = 10.0f, .ki = 0.0f, .kd = 47.0f,
    .Cur = 0.0f,.Pre = 0.0f,.Dif=0.0f,
};
/**
 * @ }
 */

float &pid_Angle = pid_ang.Cur; ///<融合角

float angset = 62.0f; ///<设定平衡角
float angoutput=0.0f; ///<直立环pwm输出

void PID_MenuInit(menu_list_t *menuList)
{
    static menu_list_t *pidMenuList =
            MENU_ListConstruct("Control", 32, menuList);
    assert(pidMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, pidMenuList, "Control", 0, 0));

    {
        MENU_ListInsert(pidMenuList, MENU_ItemConstruct(nullType, NULL, "ANG", 0, 0));

        MENU_ListInsert(pidMenuList, MENU_ItemConstruct(varfType, &angset, "angSet", 9U,
                menuItem_data_region));
        MENU_ListInsert(pidMenuList, MENU_ItemConstruct(varfType, &pid_ang.kp, "ang.kp", 10U,
                menuItem_data_region));
        MENU_ListInsert(pidMenuList, MENU_ItemConstruct(varfType, &pid_ang.ki, "ang.ki", 11U,
                menuItem_data_region));
        MENU_ListInsert(pidMenuList, MENU_ItemConstruct(varfType, &pid_ang.kd, "ang.kd", 12U,
                menuItem_data_region));
        MENU_ListInsert(pidMenuList, MENU_ItemConstruct(varfType, &angoutput, "ang.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
    }

    static menu_list_t *filterMenuList =
                MENU_ListConstruct("Filter", 32, menuList);
    assert(filterMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, filterMenuList, "Filter", 0, 0));
    {
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(nullType, NULL, "RAW", 0, 0));

        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_accl[0], "accl.x", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_accl[1], "accl.y", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_accl[2], "accl.z", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_gyro[0], "gyro.x", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_gyro[1], "gyro.y", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_gyro[2], "gyro.z", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(nullType, NULL, "FILT", 0, 0));

        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_Anginit, "AngInit", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_AngA, "Ang.Accl", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_AngG, "Ang.Gyro", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &pid_Angle, "AngFinal", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &Tg, "Tg", 10U,
                menuItem_data_global));

    }
}


/* ******************** 滤波器 ******************** */
void PID_FilterInit(void) ///<初始化
{
    const uint32_t sampleTime = 1024;
    float intergration = 0.0f;
    for(uint32_t i = 0; i < sampleTime; ++i){
        imu_6050.ReadSensorBlocking();
        imu_6050.Convert(&pid_accl[0], &pid_accl[1], &pid_accl[2], &pid_gyro[0], &pid_gyro[1], &pid_gyro[2]);
        PID_Filter(1U);
        intergration += pid_AngA;
        SDK_DelayAtLeastUs(1000,CLOCK_GetFreq(kCLOCK_CoreSysClk));
    }
    pid_Anginit = intergration / ((float)sampleTime);
    pid_Angle = pid_Anginit;
    pid_AngG = pid_Anginit;
}



void PID_Filter(uint32_t time)
{
    float Ts = float(time) * 0.001f;
    float accx = pid_acc;
    accx = (accx > G - 0.001) ? (G - 0.001) : accx; ///<加速度限制
    accx = (accx < - (G - 0.001)) ? (- (G - 0.001)) : accx;
    pid_AngA=To_ang(- asin(accx / G));
    pid_AngG+=pid_gyr*Ts;
    /** 滤波运算 */
    float increangle = (pid_gyr + ((pid_AngA - pid_Angle) * Tg)) * Ts; ///<增量
    UpdateAng(increangle); ///<更新
}
/* *********************************************** */

void UpdateAng(float increangle) ///<更新角参数
{
    pid_ang.Pre = pid_ang.Cur;
    pid_ang.Cur = pid_ang.Pre + increangle;
    pid_ang.Dif = increangle;

}

/* ******************** 直立环 ******************** */

void Angring(void)
{
    imu_6050.ReadSensorBlocking();
    imu_6050.Convert(pid_accl, pid_accl+1, pid_accl+2, pid_gyro, pid_gyro+1, pid_gyro+2);
    PID_Filter(ANGS);
    angoutput=Caculate_output();
    angoutput = angoutput < 100.0f ? angoutput : 100.0f; ///<限幅
    angoutput = angoutput > -100.0f ? angoutput : -100.0f;
    PWM();
}

/* *********************************************** */

float Caculate_output(void) ///<PD计算pwm
{
    return (angset-pid_Angle)*pid_ang.kp-pid_ang.Dif*pid_ang.kd;
}


void PWM(void) ///<控制电机
{
    if(angoutput>0)
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U, angoutput);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, angoutput);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U, -angoutput);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, -angoutput);
    }
}

