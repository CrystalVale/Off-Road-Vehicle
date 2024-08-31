#ifndef PERIPHERAL_H_
#define PERIPHERAL_H_

//--------------------------------------------------------------------------------------------------
// 引脚配置
//--------------------------------------------------------------------------------------------------
#define servoMid 750
#define motorGpo P10_3
#define motor TCPWM_CH30_P10_2
#define servo TCPWM_CH13_P00_3
#define battery ADC0_CH21_P07_5
#define switch_1 P21_5
#define switch_2 P21_6
#define buzzzer P19_4
#define motorMaxDuty 6000
#define initpwm 17000
//--------------------------------------------------------------------------------------------------
// 函数
//--------------------------------------------------------------------------------------------------
void initOk();
/********************************************GPS****************************************************/
void LonLat2UTM(double longitude, double latitude);
double getDistance_Gps(double lon1, double lat1, double lon2, double lat2);
double getDistance_Gps_Utm(double x1, double y1, double x2, double y2);
double getAngle_Gps(double lon1, double lat1, double lon2, double lat2);
double getAngle_Gps_Utm(double x1, double y1, double x2, double y2);
double getAngle_Utm(double x_1, double y_1, double x_2, double y_2);
void LonLat2UTM(double longitude, double latitude);
void getGpsData();
void getBeaconPosition();
void beaconPositionToUtm();
void BubbleSort(double arr[], int n);
void moveFilter();
/******************************************MESSAGE**************************************************/
void getLightIndex();
/********************************************KEY****************************************************/
void keyScan();
/********************************************IPS****************************************************/
void show();
void Battery_Health();
/********************************************FLASH****************************************************/
void flash();
/********************************************IMU****************************************************/
void IMU_YAW_integral();
void IMU_Gyro_Offset_Init();
void IMU_Handle_180();
void IMU_Handle_360();
void IMU_Handle_0();
/********************************************一阶互补滤波姿态结算****************************************************/
void IMU_get_data();
void IMU_Attitude_Calculate();

//--------------------------------------------------------------------------------------------------
// 结构体
//--------------------------------------------------------------------------------------------------
typedef struct
{
    double lat;
    double lon;
    double utmlat;
    double utmlon;
    double nlat;
    double nlon;
    double direction; // 地面航向（000.0~359.9 度，以真北方为参考基准）+
    int state;
    int satellite_used; // 已连接数量
    int num;            // 踩点个数
} GPS;
typedef struct
{
    double lat;
    double lon;
    float utm_lat;
    float utm_lon;
} POINT;
typedef struct
{
    float gpro_init_x;
    float gpro_init_y;
    float gpro_init_z;
    float acc_x; // x轴加速度
    float acc_y; // y轴加速度
    float acc_z; // z轴加速度

    float gyro_x; // x轴角速度
    float gyro_y; // y轴角速度
    float gyro_z; // z轴角速度

    float Roll; // 解算所得角度
    float Pitch;
    float Yaw;

    float Roll_a; // 加速度计算得到的角度
    float Pitch_a;
    float Roll_g; // 陀螺仪计算得到的角速度
    float Pitch_g;

    float lastRoll; // 上次的解算角度
    float lastPitch;
} IMU;

extern IMU imu;
extern POINT point[30];
extern GPS gps;
extern int fliter_flag; // 滤波器长开标志位

#endif