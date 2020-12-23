#include "ADC.h"

const uint8_t channels[8] = {16, 23, 17, 18, 10, 11, 12, 13};
uint32_t adc_flag[8] = {0, 0, 0, 0, 1, 0, 1, 0};

uint32_t LV_Temp[Num_AD][SampleTimes]={0};
float LV[Num_AD]={0};
float AD[Num_AD]={0};
uint8_t protect_flag=0;
float k_er=400;  //计算差比系数
float k_nor=100;  //归一化系数
uint32_t pro_flag=0; //方案选择，0或1选择差比积，2选择差比和
float V_max[Num_AD]={100};
uint32_t nor_flag=0; //是否进行归一化

//获取电感数值
uint32_t adc_allget(int num)
{
    uint32_t adc=0;
    switch(num)
    {
        case 1:
            adc=SCADC_Sample(ADC0,0,channels[0]);
            break;
        case 2:
            adc=SCADC_Sample(ADC0,0,channels[1]);
            break;
        case 3:
            adc=SCADC_Sample(ADC0,0,channels[2]);
            break;
        case 4:
            adc=SCADC_Sample(ADC0,0,channels[3]);
            break;
        case 5:
            adc=SCADC_Sample(ADC0,0,channels[4]);
            break;
        case 6:
            adc=SCADC_Sample(ADC0,0,channels[5]);
            break;
        case 7:
            adc=SCADC_Sample(ADC0,0,channels[6]);
            break;
        case 8:
            adc=SCADC_Sample(ADC0,0,channels[7]);
            break;
    }
    return adc;
}

//获取flag赋值为1的电感值
void LV_get(void)
{
    for(uint8_t i=1;i<=Num_AD;i++)
    {
        if(adc_flag[i-1]==1)
        {
            for(uint8_t j=0;j<=SampleTimes-1;j++)
            {
                LV_Temp[i-1][j]=adc_allget(i);
            }
        }
    }
}

//采集数据处理
void LV_cal(void)
{
    //剔除毛刺信号
    for(uint8_t i=0;i<=(Num_AD-1);i++)
    {
        if(adc_flag[i]==1)
        {
            for(uint8_t j=0;j<=SampleTimes-1;j++)
            {
                if(LV_Temp[i][j]>500)
                {
                    LV_Temp[i][j]=500;
                }
            }
            //排序
            bubble_sort_quicker(LV_Temp[i], SampleTimes);
            //计算均值
            LV[i]=0;
            for(uint8_t j=Num_remove;j<=SampleTimes-1-Num_remove;j++)
            {
                LV[i]=(float)LV_Temp[i][j]+LV[i];
            }
            LV[i]=LV[i]/(float)(SampleTimes-(2*Num_remove));
            if(nor_flag==1||V_max[i]>100)
            {
                LV[i]=k_nor*(LV[i]/V_max[i]);
            }
            AD[i]=LV[i];
            //差比积防无穷大
            if(LV[i] < MinLVGot)
            {
                AD[i] = MinLVGot;
            }
        }
    }
}

//冒泡排序
void bubble_sort_quicker(uint32_t arr[SampleTimes], int n)
{
    int i, j, flag;
    uint32_t temp;
    for (i = 0; i < n - 1; i++)
    {
        flag = 0;
        for (j = 0; j < n - i - 1; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                temp=arr[j+1];
                arr[j+1]=arr[j];
                arr[j]=temp;
                flag = 1;
            }
        }
        if (!flag) return;
    }
}

//丢线保护
void Protect(void)
{
    if((LV[4]+LV[6])<12)
    {
        protect_flag=1;
    }
}

//电磁主函数
float ADC_main(void)
{
    LV_get();
    LV_cal();
    Protect();
    float dev=0;
    switch(pro_flag)
    {
        case 0:
            dev = k_er*((AD[0] - AD[4]) / (AD[4] * AD[0]));
            break;
        case 1:
            dev = k_er*((AD[0] - AD[4]) / (AD[4] * AD[0] + 2));
            break;
        case 2:
            dev = k_er*((AD[0] - AD[4]) / (AD[4] + AD[0]));
            break;
        case 3:
            dev = k_er*((AD[0] - AD[4]) / ((AD[4] + AD[0])*sqrt(AD[4] + AD[0])));
            break;
        case 4:
            dev = k_er*((sqrt(AD[0]) - sqrt(AD[4])) / (AD[4] + AD[0]));
            break;
    }
    return dev;
}

//归一化操作
void Normal(void)
{
    if(nor_flag==0)
    {
        return;
    }
    float temp=0;
    uint32_t count=0;
    while(count<100)
    {
        if(nor_flag==0)
        {
            break;
        }
        for(uint8_t i=0;i<Num_AD;i++)
        {
            if(adc_flag[i]==1)
            {
                temp=adc_allget(i+1);
                if(temp>V_max[i])
                {
                    V_max[i]=temp;
                }
            }
        }
        count++;
    }
}
