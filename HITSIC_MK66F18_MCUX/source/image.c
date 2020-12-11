#include "image.h"

int f[10 * CAMERA_H];//考察连通域联通性
//每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    int   connect_num;//连通标记
}range;

//每行的所有白条子
typedef struct {
    uint8_t   num;//每行白条数量
    range   area[white_num_MAX];//该行各白条区域
}all_range;

//属于赛道的每个白条子属性
typedef struct {
    uint8_t   left;//左边界
    uint8_t   right;//右边界
    uint8_t   width;//宽度
}road_range;

//每行属于赛道的每个白条子
typedef struct {
    uint8_t   white_num;
    road_range   connected[white_num_MAX];
}road;

all_range white_range[CAMERA_H];//所有白条子
road my_road[CAMERA_H];//赛道
uint8_t IMG[CAMERA_H][CAMERA_W];//二值化后图像数组
uint8_t left_line[CAMERA_H], right_line[CAMERA_H];//赛道的左右边界
uint8_t mid_line[CAMERA_H];
int all_connect_num = 0;//所有白条子数
uint8_t top_road;//赛道最高处所在行数
uint8_t zcross_sign = 0;
uint8_t protect_sign = 0;
int32_t zcross=1;
int32_t zcmid=1;
uint8_t bend=0;
int32_t zcbegin_num=90;
int32_t zc=1;


////////////////////////////////////////////
//功能：粗犷的清车头
//输入：
//输出：
//备注：要根据自己车头的大小进行修改
///////////////////////////////////////////
void head_clear(void)
{
    uint8_t* my_map;
    for (int i = 119; i >= 102; i--)
    {
        my_map = &IMG[i][0];
        for (int j = 47; j <= 137; j++)
        {
            *(my_map + j) = white;
        }
    }
}

////////////////////////////////////////////
//功能：查找父节点
//输入：节点编号
//输出：最老祖先
//备注：含路径压缩
///////////////////////////////////////////
int find_f(int node)
{
    if (f[node] == node)return node;//找到最古老祖先，return
    f[node] = find_f(f[node]);//向上寻找自己的父节点
    return f[node];
}

////////////////////////////////////////////
//功能：提取跳变沿 并对全部白条子标号
//输入：IMG[120][188]
//输出：white_range[120]
//备注：指针提速
///////////////////////////////////////////
void search_white_range()
{
    uint8_t i, j;
    int istart = NEAR_LINE;//处理起始行
    int iend = FAR_LINE;//处理终止行
    int tnum = 0;//当前行白条数
    all_connect_num = 0;//白条编号初始化
    uint8_t* map = NULL;
    for (i = istart; i >= iend; i--)
    {
        map = &IMG[i][LEFT_SIDE];//指针行走加快访问速度
        tnum = 0;
        for (j = LEFT_SIDE; j <= RIGHT_SIDE; j++, map++)
        {
            if ((*map))//遇白条左边界
            {
                tnum++;
                if (tnum >= white_num_MAX)break;
                range* now_white = &white_range[i].area[tnum];
                now_white->left = j;

                //开始向后一个一个像素点找这个白条右边界
                map++;
                j++;

                while ((*map) && j <= RIGHT_SIDE)
                {
                    map++;
                    j++;
                }
                now_white->right = j - 1;
                now_white->connect_num = ++all_connect_num;//白条数加一，给这个白条编号
            }
        }
        white_range[i].num = tnum;
    }
}

////////////////////////////////////////////
//功能：寻找白条子连通性，将全部联通白条子的节点编号刷成最古老祖先的节点编号
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_all_connect()
{
    //f数组初始化
    for (int i = 1; i <= all_connect_num; i++)
        f[i] = i;

    //u为up d为down 即为当前处理的这两行中的上面那行和下面那行
    //u_num：上面行白条数
    //u_left：上面行当前白条左边界
    //u_right：上面行当前白条右边界
    //i_u：当前处理的这个白条是当前这行（上面行）白条中的第i_u个
    int u_num, i_u, u_left, u_right;
    int d_num, i_d, d_left, d_right;
    all_range* u_white = NULL;
    all_range* d_white = NULL;
    for (int i = NEAR_LINE; i > FAR_LINE; i--)//因为每两行每两行比较 所以循环到FAR_LINE+1
    {
        u_num = white_range[i - 1].num;
        d_num = white_range[i].num;
        u_white = &white_range[i - 1];
        d_white = &white_range[i];
        i_u = 1; i_d = 1;

        //循环到当前行或上面行白条子数耗尽为止
        while (i_u <= u_num && i_d <= d_num)
        {
            //变量先保存，避免下面访问写的冗杂且访问效率低
            u_left = u_white->area[i_u].left;
            u_right = u_white->area[i_u].right;
            d_left = d_white->area[i_d].left;
            d_right = d_white->area[i_d].right;

            if (u_left <= d_right && u_right >= d_left)//如果两个白条联通
                f[find_f(u_white->area[i_u].connect_num)] = find_f(d_white->area[i_d].connect_num);//父节点连起来（标号改为相同（可能））

            //当前算法规则，手推一下你就知道为啥这样了
            if (d_right > u_right)i_u++;
            if (d_right < u_right)i_d++;
            if (d_right == u_right) { i_u++; i_d++; }
        }
    }
}

////////////////////////////////////////////
//功能：寻找赛道
//输入：
//输出：
//备注：
///////////////////////////////////////////
void find_road()
{
    uint8_t istart = NEAR_LINE;
    uint8_t iend = FAR_LINE;
    top_road = NEAR_LINE;//赛道最高处所在行数，先初始化话为最低处
    int road_f = -1;//赛道所在连通域父节点编号，先初始化为-1，以判断是否找到赛道
    int while_range_num = 0, roud_while_range_num = 0;
    all_range* twhite_range = NULL;
    road* tmy_road = NULL;
    //寻找赛道所在连通域
    // 寻找最中心的白条子
    for (int i = 1; i <= white_range[istart].num; i++)
        if (white_range[istart].area[i].left <= CAMERA_W / 2
            && white_range[istart].area[i].right >= CAMERA_W / 2 && (white_range[istart].area[i].right - white_range[istart].area[i].left) >= 90)
            road_f = find_f(white_range[istart].area[i].connect_num);

    if (road_f == -1)//若赛道没在中间，在113行选一行最长的认为这就是赛道
    {
        int widthmax = 0, jselect = 1;
        for (int i = 1; i <= white_range[istart].num; i++)
            if (white_range[istart].area[i].right - white_range[istart].area[i].left > widthmax)
            {
                widthmax = white_range[istart].area[i].right - white_range[istart].area[i].left;
                jselect = i;
            }
        road_f = find_f(white_range[istart].area[jselect].connect_num);
    }

    //现在我们已经得到了赛道所在连通域父节点编号，接下来把所有父节点编号是road_f的所有白条子扔进赛道数组就行了
    for (int i = istart; i >= iend; i--)
    {
        //变量保存，避免之后写的冗杂且低效
        twhite_range = &white_range[i];
        tmy_road = &my_road[i];
        while_range_num = twhite_range->num;
        tmy_road->white_num = 0;
        roud_while_range_num = 0;
        for (int j = 1; j <= while_range_num; j++)
        {
            if (find_f(twhite_range->area[j].connect_num) == road_f)
            {
                top_road = i;
                tmy_road->white_num++; roud_while_range_num++;
                tmy_road->connected[roud_while_range_num].left = twhite_range->area[j].left;
                tmy_road->connected[roud_while_range_num].right = twhite_range->area[j].right;
                tmy_road->connected[roud_while_range_num].width = twhite_range->area[j].right - twhite_range->area[j].left;
            }
        }
    }

}

////////////////////////////////////////////
//功能：返回相连下一行白条子编号
//输入：i_start起始行  j_start白条标号
//输出：白条标号
//备注：认为下一行与本行赛道重叠部分对多的白条为选定赛道
///////////////////////////////////////////
uint8_t find_continue(uint8_t i_start, uint8_t j_start)
{
    uint8_t j_return;
    uint8_t j;
    uint8_t width_max = 0;
    uint8_t width_new = 0;
    uint8_t left = 0;
    uint8_t right = 0;
    uint8_t dright, dleft, uright, uleft;
    j_return = MISS;//如果没找到，输出255
    if (j_start > my_road[i_start].white_num)
        return MISS;
    //选一个重叠最大的
    for (j = 1; j <= my_road[i_start - 1].white_num; j++)
    {
        dleft = my_road[i_start].connected[j_start].left;
        dright = my_road[i_start].connected[j_start].right;
        uleft = my_road[i_start - 1].connected[j].left;
        uright = my_road[i_start - 1].connected[j].right;
        if (//相连
            dleft < uright
            &&
            dright > uleft
            )
        {
            //计算重叠大小
            if (dleft < uleft) left = uleft;
            else left = dleft;

            if (dright > uright) right = uright;
            else right = dright;

            width_new = right - left + 1;

            if (width_new > width_max)
            {
                width_max = width_new;
                j_return = j;
            }
        }

    }
    return j_return;
}

////////////////////////////////////////////
//功能：通用决定双边
//输入：
//输出：
//备注：
///////////////////////////////////////////
void ordinary_two_line(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t j_continue[CAMERA_H];//第一条连通路径
    uint8_t i_start;
    uint8_t i_end;
    uint8_t j_start = MISS;
    int width_max;

    //寻找起始行最宽的白条子
    i_start = NEAR_LINE;
    i_end = FAR_LINE;
    width_max = 0;
    for (j = 1; j <= my_road[i_start].white_num; j++)
    {
        if (my_road[i_start].connected[j].width > width_max)
        {
            width_max = my_road[i_start].connected[j].width;
            j_start = j;
        }
    }
    j_continue[i_start] = j_start;

    //记录连贯区域编号
    for (i = i_start; i > i_end; i--)
    {
        //如果相连编号大于该行白条数，非正常，从此之后都MISS
        if (j_continue[i] > my_road[i].white_num)
        {
            j_continue[i - 1] = MISS;
        }
        else
        {
            j_continue[i - 1] = find_continue(i, j_continue[i]);
        }

    }

    //全部初始化为MISS
    my_memset(left_line, MISS, CAMERA_H);
    my_memset(right_line, MISS, CAMERA_H);


    for (i = i_start; i > i_end; i--)
    {
        if (j_continue[i] <= my_road[i].white_num)
        {
            left_line[i] = my_road[i].connected[j_continue[i]].left;
            right_line[i] = my_road[i].connected[j_continue[i]].right;
            IMG[i][left_line[i]] = blue;
            IMG[i][right_line[i]] = red;
        }
        else
        {
            left_line[i] = MISS;
            right_line[i] = MISS;
        }
    }
}

void fix_un()
{
    bend = 0;
    //分别储存左边和右边界十字起始点
    int begin_left = NEAR_LINE;
    int begin_right = NEAR_LINE;
    float k_left, k_right; //储存十字之前边界的斜率
    if (Cross_begin() == 0)
    {
        for (int i = 100; i >= 0; i--) //119-102为清车头范围
        {
            if (IsCross(i, left_line[i] - 1, 3, 1) == 1) //判断是否是十字起始点
            {
                begin_left = i + 4;
                break; //退出避免之后赛道的影响
            }
        }
        for (int i = 100; i >= 0; i--)  //同上
        {
            if (IsCross(i, right_line[i] + 1, 3, 1) == 1)
            {
                begin_right = i + 4;
                break;
            }
        }
        //判断是否两边都找出了十字起始点，如果只有一边则为弯道
        if (begin_left != NEAR_LINE && begin_right != NEAR_LINE)
        {
            //最小二乘法计算两边斜率
            k_left = K_line(NEAR_LINE - ((NEAR_LINE - begin_left) / 2), begin_left, 0);
            k_right = K_line(NEAR_LINE - ((NEAR_LINE - begin_right) / 2), begin_right, 1);
            for (int i = begin_left; i >= 0; i--)
            {
                //从十字起始点向前伸，遇见黑色的点则停止
                left_line[i] = (uint8_t)((float)left_line[begin_left] + (k_left * (float)(i - begin_left)) + 0.5);  //注意数值类型转换防止数值溢出
                if ((i < begin_left - 10) && (IMG[i][left_line[i]] != white))
                {
                    for (int j = 10; j >= 0; j--)
                    {
                        left_line[i - j] = (uint8_t)((float)left_line[begin_left] + (k_left * (float)(i - j - begin_left)) + 0.5);
                    }
                    break;
                }
                IMG[i][left_line[i]] = green;
            }
            for (int i = begin_right; i >= 0; i--)
            {
                //同上
                right_line[i] = (uint8_t)((float)right_line[begin_right] + (k_right * (float)(i - begin_right)) + 0.5);
                if ((i < begin_right - 10) && (IMG[i][right_line[i]] != white))
                {
                    for (int j = 10; j >= 0; j--)
                    {
                        right_line[i - j] = (uint8_t)((float)right_line[begin_right] + (k_right * (float)(i - j - begin_right)) + 0.5);
                    }
                    break;
                }
                IMG[i][right_line[i]] = purple;
            }
        }
        if ((begin_left >= 103 || begin_left <= 15) || (begin_right >= 103 || begin_right <= 15))
        {
            bend = 1;
        }
    }
    else
    {
        find_K(0);
        find_K(1);
    }
}


float K_line(int Begin, int Final, int sign)
{
    float k = 0;
    int I = 0, i = 0, Line = 0, IL = 0;
    for (int j = Begin; j >= Final; j--)
    {
        i = i + j;
        I = I + j * j;
        if (sign == 0)
        {
            Line = Line + left_line[j];
            IL = IL + left_line[j] * j;
        }
        else
        {
            Line = Line + right_line[j];
            IL = IL + right_line[j] * j;
        }
    }
    float i_f = (float)i / (float)(Begin - Final + 1);
    float I_f = (float)I / (float)(Begin - Final + 1);
    float Line_f = (float)Line / (float)(Begin - Final + 1);
    float IL_f = (float)IL / (float)(Begin - Final + 1);
    k = (IL_f - (i_f * Line_f)) / (I_f - (i_f * i_f));
    return k;
}

////////////////////////////////////////////
//功能：数组初始化
//输入：uint8_t* ptr 数组首地址, uint8_t num初始化的值, uint8_t size数组大小
//输出：
//备注：因为k66库中认为memset函数不安全，所以无法使用；因此需要自己写一个my_memset
///////////////////////////////////////////
void my_memset(uint8_t* ptr, uint8_t num, uint8_t size)
{
    uint8_t* p = ptr;
    uint8_t my_num = num;
    uint8_t Size = size;
    for (int i = 0; i < Size; i++, p++)
    {
        *p = my_num;
    }
}
////////////////////////////////////////////
//功能：中线合成
//输入：左右边界
//输出：中线
//备注：
///////////////////////////////////////////
void get_mid_line(void)
{
    my_memset(mid_line, MISS, CAMERA_H);
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
        if (left_line[i] != MISS)
        {
            mid_line[i] = (left_line[i] + right_line[i]) / 2;
        }
        else
        {
            mid_line[i] = MISS;
        }
}
////////////////////////////////////////////
//功能：图像处理主程序
//输入：
//输出：
//备注：
///////////////////////////////////////////
uint8_t image_main(uint8_t midline)
{
    int midline_return = 0;
    head_clear();
    search_white_range();
    find_all_connect();
    find_road();
    /*到此处为止，我们已经得到了属于赛道的结构体数组my_road[CAMERA_H]*/
    ordinary_two_line();
    fix_un();
    get_mid_line();
    Protect();
    Zc_double();
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
        if (mid_line[i] != MISS)
            IMG[i][mid_line[i]] = red;
    for (int i = NEAR_LINE; i >= FAR_LINE; i--)
    {
        fullBuffer[i*188+mid_line[i]]=black;
        fullBuffer[i*188+left_line[i]]=black;
        fullBuffer[i*188+right_line[i]]=black;
        fullBuffer[i*188+(mid_line[i]+1)]=black;
        fullBuffer[i*188+(mid_line[i]-1)]=black;
        fullBuffer[i*188+94]=black;
    }
    while(midline_wrong(midline))
    {
            midline = midline + 2;
    }
    midline=midline+1;
    for(int i=187;i>=0;i--)
    {
        fullBuffer[(midline-1)*188+i]=black;
        fullBuffer[midline*188+i]=black;
        fullBuffer[(midline+1)*188+i]=black;
    }
    for(int i = midline - 1; i <= midline + 1; i ++)
    {
        midline_return=midline_return+mid_line[i];
    }
    midline_return=midline_return/3;
    if(zcmid==1)
    {
        if((bend==0)&&(zc_mid(midline)))
        {
            midline_return=94;
        }
    }
    return midline_return;
}

int IsCross(int x, int y, int Num, int Sign)
{
    //Sign标示用于判断那种十字点，1为普通，二为起始段都为十字的情况
    int count = 0;
    int Count_1 = 0;
    int Count_2 = 0;
    if (Sign == 1)
    {
        for (int i = 0; i < Num; i++)
        {
            if (IMG[x - i][y] == white)  //边界点的前面为白是普通十字点
            {
                count++;
            }
        }
    }
    else if (Sign == 0)
    {
        for (int i = 0; i < Num; i++)
        {
            if (IMG[x + i][y] == white)  //边界的前面是黑后面是白则为另一种十字点
            {
                Count_1++;
            }
            if (IMG[x - i][y] == black)
            {
                Count_2++;
            }
        }
    }
    if (Num - count <= 1)
    {
        return 1;
    }
    if ((Num - Count_1 <= 1) && (Num - Count_2 <= 1))
    {
        return 2;
    }
    return 0;
}

//用于判断二次过斑马线
void Zc_double()
{
    if ((zcross_sign == 0) && (Black_White()))
    {
        zcross_sign = 1;
        return;
    }
    if ((zcross==1)&&(((zcross_sign == 1) && (Black_White())) || (zcross_sign == 0)||(zcross_sign ==3)))
    {
        for (int i = 119; i >= 35;i--)
        {
            mid_line[i] = 94;
        }
    }
    if ((zcross_sign == 1) && (!Black_White()))
    {
        zcross_sign = 2;
        return;
    }
    if ((zcross_sign == 2) && (Black_White()))
    {
        zcross_sign = 3;
        return;
    }
    if ((zcross_sign == 3) && (!Black_White()))
    {
        zcross_sign = 4;
        return;
    }
}


//判断近处斑马线
int Black_White()
{
    int count=0;
    int black_sign = 0;
    for (int i = 1; i <= 5; i++)
    {
        for (int j = 0; j <= 188; j++)
        {
            if ((IMG[101 - i][j] == white && IMG[101 - i][j + 1] == black) || (IMG[101 - i][j] == black && IMG[101 - i][j + 1] == white))
            {
                count++;
            }
        }
        for (int j = 80; j <= 100; j++)
        {
            if (IMG[101 - i][j] == black)
            {
                black_sign++;
            }
        }
    }
    if (count >= 50 && protect_sign == 0)
    {
        return 1;
    }
    if (black_sign <= 4)
    {
        return 0;
    }
}

void Protect()
{
    int white_sign = 0;
    for (int i = 2; i <= 3; i++)
    {
        for (int j = 80; j <= 100; j++)
        {
            if (IMG[101 - i][j] == white)
            {
                white_sign++;
            }
        }
    }
    if (white_sign <= 4)
    {
        protect_sign = 1;
    }
}

int midline_wrong(int midline)
{
    int black_mid = 0;
    for (int i = mid_line[midline - 1] - 3; i <= mid_line[midline - 1] + 3; i++)
    {
        if (IMG[midline - 1][i] == black)
        {
            black_mid++;
        }
    }
    if (black_mid >= 3)
    {
        return 1;
    }
    return 0;
}

void find_K(int sign)
{
    int left = 0;
    int right = 0;
    int begin = 119;
    if (sign == 0)
    {
        for (int i = 111; i >= 20; i--)
        {
            if (left_line[i] <= 5 && left_line[i + 1] <= 5 && left_line[i - 1] > 5 && left_line[i - 2] > 5)
            {
                begin = i;
                break;
            }
        }
        while (IsWide(begin, 0))
        {
            begin = begin - 2;
        }
    }
    if (sign == 1)
    {
        for (int i = 111; i >= 20; i--)
        {
            if (right_line[i] >= 182 && right_line[i + 1] >= 182 && right_line[i - 1] < 182 && right_line[i - 2] < 182)
            {
                begin = i;
                break;
            }
        }
        while (IsWide(begin, 1))
        {
            begin = begin - 3;
        }
    }
    begin = begin - 2;
    if (sign == 0)
    {
        float k1 = 0;
        k1 = K_line(begin, begin-4, 0);
        //printf("%f\n", k1);
        for (int i = 119; i >= begin; i--)
        {
            left_line[i] = (uint8_t)((float)left_line[begin] + (k1 * (float)(i - begin)) + 0.5);
            IMG[i][left_line[i]] = green;
        }
    }
    if (sign == 1)
    {
        float k2 = 0;
        //printf("%f\n", k2);
        k2 = K_line(begin, begin-4, 1);
        for (int i = 119; i >= begin; i--)
        {
            right_line[i] = (uint8_t)((float)right_line[begin] + (k2 * (float)(i - begin)) + 0.5);
            IMG[i][right_line[i]] = purple;
        }
    }
}

int Cross_begin()
{
    int num= (112 - 90) / 10;
    for (int i = 0; i <= num; i++)
    {
        if ((IMG[90 + i * 10][3] == white && IMG[90 + i * 10][4] == white) || (IMG[90 + i * 10][184] == white && IMG[90 + i * 10][183] == white))
        {
            //printf("%d\n", 1);
            return 1;
        }
    }
    return 0;
}

int IsWide(int begin,int sign)
{
    int count = 0;
    if (sign == 0)
    {
        for (int i = 0; i <= 6; i++)
        {
            if (IMG[begin][10 * i + 4] == white)
            {
                count++;
            }
        }
    }
    if (sign == 1)
    {
        for (int i = 12; i <= 18; i++)
        {
            if (IMG[begin][10 * i + 4] == white)
            {
                count++;
            }
        }
    }
    if (count>=4)
    {
        return 1;
    }
    return 0;
}

int zc_mid(uint8_t midline)
{
    int num = (112 - midline) / 10;
    for (int i = 0; i <= num; i++)
    {
        if (fabs(right_line[midline + 10 * i] - left_line[midline + 10 * i]) <= 12)
        {
            return 1;
        }
    }
    return 0;
}
