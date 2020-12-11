#include "pid.hpp"

/**
 * @brief   控制环PITMGR任务句柄
 * @ {
 */
pitMgr_t* Ang_Handle = nullptr;
pitMgr_t* Spd_Handle = nullptr;
pitMgr_t* Dir_Handle = nullptr;
/**
 * @ }
 */

/**
 * @brief   IMU传感器原始数据
 * @ {
 */
float pid_accl[3] = {0.0f, 0.0f, 0.0f};
float pid_gyro[3] = {0.0f, 0.0f, 0.0f};
/**
 * @ }
 */

int32_t Ang_con[3] = {0, 0, 1};
int32_t Spd_con[3] = {0, 0, 1};
int32_t Dir_con[3] = {0, 0, 1};
int32_t gear[3] = {1, 0, 4};

float &pid_acc = pid_accl[0];///<x轴加速度计
float &pid_gyr = pid_gyro[1];///<y轴角速度

float pid_AngA = 0.0f, pid_AngG = 0.0f; ///< 由加速度计、角速度计测出的角度
float pid_Anginit = 0.0f; ///<初始化角
float Tg = 1.0f / 0.8f; ///<K分之一

/**
 * @brief   角度滤波相关变量。
 * @note    借用结构体保存当前值、历史值、微分值、积分值。
 * @ {
 */
struct PID pid_ang =
{
    .kp = 10.0f, .ki = 0.0f, .kd = 0.235f,
    .Curr = 0.0f,.Prev = 0.0f,.Diff=0.0f,.Inte=0.0f
};
/**
 * @ }
 */

/**
 * @brief   速度滤波相关变量。
 * @note    借用结构体保存当前值、历史值、微分值、积分值。
 * @ {
 */
struct PID pid_spd =
{
    .kp = 10.0f, .ki = 0.0f, .kd = 47.0f,
    .Curr = 0.0f,.Prev = 0.0f,.Diff=0.0f,.Inte=0.0f
};
/**
 * @ }
 */
/**
 * @brief   转向滤波相关变量。
 * @note    借用结构体保存当前值、历史值、微分值、积分值。
 * @ {
 */
struct PID pid_wdst =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .Curr = 0.0f,.Prev = 0.0f,.Diff=0.0f,.Inte=0.0f
};

struct PID pid_wyaw =
{
    .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
    .Curr = 0.0f,.Prev = 0.0f,.Diff=0.0f,.Inte=0.0f
};
/**
 * @ }
 */

float &pid_Angle = pid_ang.Curr; ///<融合角

float angset=63.5f; ///<设定平衡角
float angoutput=0.0f; ///<直立环pwm输出
float spdset=0.0f; ///<设定速度
float spdcolL=0.0f,spdcolR=0.0f;///<左右电机速度
float ftmL=0.0f,ftmR=0.0f;///<左右ftm脉冲
float spdcolA=0.0f;///<平均速度
float spdrawop=0.0f;///<速度环原始输出
float spdfilop=0.0f;///<速度环滤波输出
float spdop[FILCOUNT]={0};///<滤波数组
float pic_dif=0.0f;///<中线偏差
float pic_kp=0.0f;///<中线kp
float pwm_diff=0.0f;///<pwm偏差
extern int32_t midline;
extern int32_t imageTH;
extern int32_t threshold;
extern uint8_t zcross_sign;
extern uint8_t protect_sign;
extern int32_t zcross;
extern int32_t wifi;
extern int32_t img_upload;
extern int32_t zcmid;
int32_t count=0;
int32_t tcount=20;
int32_t protect=0;
int32_t init=0;

void PID_Init(void)
{
    Spd_Handle = pitMgr_t::insert(SPDS, 2U, Spd_ring, pitMgr_t::enable);
    assert(Spd_Handle);
    Dir_Handle = pitMgr_t::insert(DIRS, 3U, Dir_ring, pitMgr_t::enable);
    assert(Dir_Handle);
    Ang_Handle = pitMgr_t::insert(ANGS, 4U, Ang_ring, pitMgr_t::enable);
    assert(Ang_Handle);
}

void PID_MenuInit(menu_list_t *menuList)
{
    static menu_list_t *conMenuList =
            MENU_ListConstruct("Control", 15, menuList);
    assert(conMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, conMenuList, "Control", 0, 0));
    {
        MENU_ListInsert(conMenuList, MENU_ItemConstruct(variType, &gear[0], "gear", 17U,
                menuItem_data_region | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(conMenuList, MENU_ItemConstruct(variType, &init, "init", 27U,
                menuItem_data_region));
        MENU_ListInsert(conMenuList, MENU_ItemConstruct(variType, &tcount, "delay", 22U,
                menuItem_data_region ));
        MENU_ListInsert(conMenuList, MENU_ItemConstruct(variType, &Ang_con[0], "ang.con", 18U,
                menuItem_data_region | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(conMenuList, MENU_ItemConstruct(variType, &Spd_con[0], "spd.con", 19U,
                menuItem_data_region | menuItem_dataExt_HasMinMax));
        MENU_ListInsert(conMenuList, MENU_ItemConstruct(variType, &Dir_con[0], "dir.con", 20U,
                menuItem_data_region | menuItem_dataExt_HasMinMax));
    }
    static menu_list_t *angMenuList =
            MENU_ListConstruct("Ang.ring", 15, menuList);
    assert(angMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, angMenuList, "Ang.ring", 0, 0));

    {
        MENU_ListInsert(angMenuList, MENU_ItemConstruct(nullType, NULL, "ANG", 0, 0));
        MENU_ListInsert(angMenuList, MENU_ItemConstruct(varfType, &angset, "angSet", 1U,
                menuItem_data_region));
        MENU_ListInsert(angMenuList, MENU_ItemConstruct(varfType, &pid_ang.kp, "ang.kp", 2U,
                menuItem_data_region));
        MENU_ListInsert(angMenuList, MENU_ItemConstruct(varfType, &pid_ang.ki, "ang.ki", 3U,
                menuItem_data_region));
        MENU_ListInsert(angMenuList, MENU_ItemConstruct(varfType, &pid_ang.kd, "ang.kd", 4U,
                menuItem_data_region));
        MENU_ListInsert(angMenuList, MENU_ItemConstruct(varfType, &angoutput, "ang.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
    }
    static menu_list_t *spdMenuList =
            MENU_ListConstruct("Spd.ring", 15, menuList);
    assert(spdMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, spdMenuList, "Spd.ring", 0, 0));
    {

        MENU_ListInsert(spdMenuList, MENU_ItemConstruct(nullType, NULL, "SPD", 0, 0));
        MENU_ListInsert(spdMenuList, MENU_ItemConstruct(varfType, &spdset, "spdSet", 5U,
                menuItem_data_region));
        MENU_ListInsert(spdMenuList, MENU_ItemConstruct(varfType, &pid_spd.kp, "spd.kp", 6U,
                menuItem_data_region));
        MENU_ListInsert(spdMenuList, MENU_ItemConstruct(varfType, &pid_spd.ki, "spd.ki", 7U,
                menuItem_data_region));
        MENU_ListInsert(spdMenuList, MENU_ItemConstruct(varfType, &pid_spd.kd, "spd.kd", 8U,
                menuItem_data_region));
        MENU_ListInsert(spdMenuList, MENU_ItemConstruct(varfType, &spdfilop, "spd.out", 0U,
                menuItem_data_NoSave | menuItem_data_NoLoad));
    }

    static menu_list_t *dirMenuList =
            MENU_ListConstruct("Dir.ring", 15, menuList);
    assert(dirMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, dirMenuList, "Dir.ring", 0, 0));
    {
        MENU_ListInsert(dirMenuList, MENU_ItemConstruct(nullType, NULL, "DIR", 0, 0));
        MENU_ListInsert(dirMenuList, MENU_ItemConstruct(varfType, &pic_kp, "pic.kp", 0U,
                menuItem_data_region));
        MENU_ListInsert(dirMenuList, MENU_ItemConstruct(varfType, &pid_wdst.kp, "dir.kp", 9U,
                menuItem_data_region));
        MENU_ListInsert(dirMenuList, MENU_ItemConstruct(varfType, &pid_wdst.ki, "dir.ki", 10U,
                menuItem_data_region));
        MENU_ListInsert(dirMenuList, MENU_ItemConstruct(varfType, &pid_wdst.kd, "dir.kd", 11U,
                menuItem_data_region));
        MENU_ListInsert(dirMenuList, MENU_ItemConstruct(varfType, &pwm_diff, "pwm.diff", 12U,
                menuItem_data_region));
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
        MENU_ListInsert(filterMenuList, MENU_ItemConstruct(varfType, &Tg, "Tg", 13U,
                menuItem_data_global));

    }
    static menu_list_t *imgMenuList =
                MENU_ListConstruct("IMG", 15, menuList);
    assert(imgMenuList);
    MENU_ListInsert(menuList, MENU_ItemConstruct(menuType, imgMenuList, "IMG", 0, 0));
    {
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(nullType, NULL, "IMG", 0, 0));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &imageTH, "imgTH", 14U,
                menuItem_data_region));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &midline, "midline", 15U,
                menuItem_data_region));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &threshold, "threshold", 16U,
                menuItem_data_region));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &zcross, "ZCross", 21U,
                        menuItem_data_region));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &protect, "protect", 23U,
                        menuItem_data_region));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &img_upload, "IMG_UPLOAD", 24U,
                                menuItem_data_region));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &wifi, "WIFI", 25U,
                                menuItem_data_region));
        MENU_ListInsert(imgMenuList, MENU_ItemConstruct(variType, &zcmid, "Zcmid", 26U,
                                menuItem_data_region));
    }

}


/* ******************** 滤波器 ******************** */
void Ang_Init(void) ///<初始化
{
    const uint32_t Itime = 1024;
    float intergration = 0.0f;
    for(uint32_t i = 0; i < Itime; ++i){
        imu_6050.ReadSensorBlocking();
        imu_6050.Convert(&pid_accl[0], &pid_accl[1], &pid_accl[2], &pid_gyro[0], &pid_gyro[1], &pid_gyro[2]);
        Ang_Filter(1U);
        intergration += pid_AngA;
        SDK_DelayAtLeastUs(1000,CLOCK_GetFreq(kCLOCK_CoreSysClk));
    }
    pid_Anginit = intergration / ((float)Itime);
    pid_Angle = pid_Anginit;
    pid_AngG = pid_Anginit;
}



void Ang_Filter(uint32_t Ftime)
{
    float Ts = float(Ftime) * 0.001f;
    float accx = pid_acc;
    accx = (accx > G - 0.001) ? (G - 0.001) : accx; ///<加速度限制
    accx = (accx < - (G - 0.001)) ? (- (G - 0.001)) : accx;
    pid_AngA=To_ang(- asin(accx / G));
    pid_AngG+=pid_gyr*Ts;
    /** 滤波运算 */
    float increangle = (pid_gyr + ((pid_AngA - pid_Angle) * Tg)) * Ts; ///<增量
    Updatering(&pid_ang,increangle,1); ///<更新
}
/* *********************************************** */

void Updatering(struct PID* pid_ring,float parameter,int sign) ///<更新参数
{
    switch(sign)
    {
    case 1:///<增量更新
        pid_ring->Prev = pid_ring->Curr;
        pid_ring->Curr = pid_ring->Prev + parameter;
        pid_ring->Inte += pid_ring->Curr;
        pid_ring->Diff = parameter;
        break;
    case 2:///<新值更新
        pid_ring->Prev = pid_ring->Curr;
        pid_ring->Curr = parameter;
        pid_ring->Inte += pid_ring->Curr;
        pid_ring->Diff = (pid_ring->Curr-pid_ring->Prev);
        break;
    }


}

/* ******************** 直立环 ******************** */

void Ang_ring(void)
{
    imu_6050.ReadSensorBlocking();///<读参
    imu_6050.Convert(pid_accl, pid_accl+1, pid_accl+2, pid_gyro, pid_gyro+1, pid_gyro+2);
    Ang_Filter(ANGS);
    if(Ang_con[0]==1)
    {
        angoutput=(angset+spdfilop-pid_Angle)*pid_ang.kp+(spdfilop-pid_ang.Diff)*pid_ang.kd;///<计算角度环输出
        angoutput=angoutput<100.0f?angoutput:100.0f;///<限幅
        angoutput=angoutput>-100.0f?angoutput:-100.0f;
    }
    else
    {
        angoutput=0.0f;
    }
    PWM(angoutput+pwm_diff,angoutput-pwm_diff);
    Stop();
}

/* *********************************************** */


/* ******************** 速度环 ******************** */

void Spd_ring(void)
{
    spdcolL=-((float)SCFTM_GetSpeed(FTM2)) /(FTMVALUE*SPDS*0.001);///<速度获取
    SCFTM_ClearSpeed(FTM2);
    spdcolR=((float)SCFTM_GetSpeed(FTM1)) /(FTMVALUE*SPDS*0.001);
    SCFTM_ClearSpeed(FTM1);
    spdcolA=(spdcolL+spdcolR)/2;///<平均速度
    float increspeed=(spdset-spdcolA);
    if(Spd_con[0]==1)
    {
        Updatering(&pid_spd,increspeed,2);
        spdrawop=pid_spd.Curr*pid_spd.kp+pid_spd.Inte*pid_spd.ki;///<PD控制
        Spd_Filter();
    }
    else
    {
        spdfilop=0.0f;
    }
}

/* *********************************************** */


/* ******************** 滑动滤波 ******************** */
void Spd_Filter(void)
{
    float filopsum=0.0f;
    for(int i=0;i<FILCOUNT-1;i++)
    {
        spdop[i+1]=spdop[i];
        filopsum+=spdop[i];
    }
    spdop[FILCOUNT-1]=spdrawop;
    filopsum+=spdop[FILCOUNT-1];
    spdfilop=filopsum/(FILCOUNT);

}

/* *********************************************** */

/* ******************** 转向环 ******************** */

void Dir_ring(void)
{
    float w=0.0f;
    if(pid_gyro[2]>=0)
        w=sqrt(pow(pid_gyro[0],2)+pow(pid_gyro[2],2));
    else
        w=-sqrt(pow(pid_gyro[0],2)+pow(pid_gyro[2],2));
    w=To_rad(w);
    pic_dif=pic_dif<94?pic_dif:94;
    pic_dif=pic_dif>-94?pic_dif:-94;
    if(Dir_con[0]==1)
    {
        Updatering(&pid_wdst,pic_dif*pic_kp*spdcolA,2);///<更新参数
        Updatering(&pid_wyaw,w,2);
        pwm_diff=(pid_wdst.Curr-pid_wyaw.Curr)*pid_wdst.kp+(pid_wdst.Diff-pid_wyaw.Diff)*pid_wdst.kd;///<转向偏差输出
        pwm_diff=pwm_diff<30?pwm_diff:30;///<限幅
        pwm_diff=pwm_diff>-30?pwm_diff:-30;
    }
    else
    {
        pwm_diff=0.0f;
    }

}

/* *********************************************** */

void PWM(float pwm_L,float pwm_R) ///<控制电机
{
    pwm_L=pwm_L<100.0f?pwm_L:100.0f;///<限幅
    pwm_L=pwm_L>-100.0f?pwm_L:-100.0f;
    pwm_R=pwm_R<100.0f?pwm_R:100.0f;
    pwm_R=pwm_R>-100.0f?pwm_R:-100.0f;

    if(pwm_L < 0.0f && pwm_R > 0.0f)///<反转保护
    {
        pwm_L = 0.0f;
    }
    if(pwm_L > 0.0f && pwm_R < 0.0f)
    {
        pwm_R = 0.0f;
    }
    if(pwm_L>0)
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U, pwm_L);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_1, 20000U, -pwm_L);
    }
    if(pwm_R>0)
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, pwm_R);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(FTM0, kFTM_Chnl_3, 20000U, -pwm_R);
    }
}

void Stop(void)
{
    if(protect==1)
    {
        if(protect_sign==1)
        {
            Ang_con[0]=0;
            Spd_con[0]=0;
            Dir_con[0]=0;

        }
    }
    if(zcross==1)
    {
        if(zcross_sign==4&&protect_sign==0)
        {
           count++;
        }
        if(count==tcount)
        {
            Ang_con[0]=0;
            Spd_con[0]=0;
            Dir_con[0]=0;
        }
    }
}

void Start_init(void)
{
    if(init==1)
    {
        zcross_sign=0;
        protect_sign=0;
        Ang_con[0]=1;
        Spd_con[0]=0;
        Dir_con[0]=0;
        SDK_DelayAtLeastUs(1000000, 180000000);
        zcross_sign=0;
        protect_sign=0;
        Ang_con[0]=1;
        Spd_con[0]=1;
        Dir_con[0]=1;
        switch(gear[0])
         {
         case 0:
             break;
         case 1:
             spdset=3.00;pid_spd.kp=-10.0;
             pic_kp=0.10;pid_wdst.kp=-5.90;
             midline=54;
             break;
         case 2:
             spdset=3.60;pid_spd.kp=-8.0;
             pic_kp=0.08;pid_wdst.kp=-6.80;
             midline=48;
             break;
         case 3:
             spdset=3.80;pid_spd.kp=-8.0;
             pic_kp=0.08;pid_wdst.kp=-7.10;
             midline=46;
             break;
         case 4:
             spdset=3.80;pid_spd.kp=-8.0;
             pic_kp=0.08;pid_wdst.kp=-7.30;
             midline=42;
             break;

         }
    }
}


/* ******************** WIFI模块 ******************** */
void Wifi(void)
{
    float mydata[8]={pwm_diff,angoutput,spdcolA,pic_dif,pid_wdst.Curr,pid_wyaw.Curr,zcross_sign,protect_sign};
    SCHOST_VarUpload(mydata,8);
}

/* *********************************************** */
