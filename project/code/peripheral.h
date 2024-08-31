#ifndef PERIPHERAL_H_
#define PERIPHERAL_H_

//--------------------------------------------------------------------------------------------------
// ��������
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
// ����
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
/********************************************һ�׻����˲���̬����****************************************************/
void IMU_get_data();
void IMU_Attitude_Calculate();

//--------------------------------------------------------------------------------------------------
// �ṹ��
//--------------------------------------------------------------------------------------------------
typedef struct
{
    double lat;
    double lon;
    double utmlat;
    double utmlon;
    double nlat;
    double nlon;
    double direction; // ���溽��000.0~359.9 �ȣ����汱��Ϊ�ο���׼��+
    int state;
    int satellite_used; // ����������
    int num;            // �ȵ����
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
    float acc_x; // x����ٶ�
    float acc_y; // y����ٶ�
    float acc_z; // z����ٶ�

    float gyro_x; // x����ٶ�
    float gyro_y; // y����ٶ�
    float gyro_z; // z����ٶ�

    float Roll; // �������ýǶ�
    float Pitch;
    float Yaw;

    float Roll_a; // ���ٶȼ���õ��ĽǶ�
    float Pitch_a;
    float Roll_g; // �����Ǽ���õ��Ľ��ٶ�
    float Pitch_g;

    float lastRoll; // �ϴεĽ���Ƕ�
    float lastPitch;
} IMU;

extern IMU imu;
extern POINT point[30];
extern GPS gps;
extern int fliter_flag; // �˲���������־λ

#endif