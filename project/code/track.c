#include "zf_common_headfile.h"

int GoFlag = 0; // 发车标志
int light_last_index;
int voice = 0;
double distance;
double angle;
int ok;

extern float utmlat, utmlon;
extern int utm_mode;
extern int light_now_index;
extern int Daty_Z, T_M, T_N;

// 逐飞
void zf_Gps()
{
    angle = getAngle_Gps(gps.nlon, gps.nlat, point[light_now_index - 1].lon, point[light_now_index - 1].lat);
    distance = getDistance_Gps(gps.nlon, gps.nlat, point[light_now_index - 1].lon, point[light_now_index - 1].lat);
    if (distance > 11)
    {
        SetMotorDuty(5000, 1);
    }
    else if (distance > 8)
    {
        SetMotorDuty(4000, 1);
    }
    else if (distance > 6)
    {
        SetMotorDuty(3000, 1);
    }
    else if (distance < 5)
    {
        sendDataToSore(1);
        voice = 1;
        light_last_index = light_now_index;
        SetMotorDuty(0, 0);
        SetServoAngle(servoMid);
    }
}

int going;
void track()
{
    if (GoFlag)
    {
        if (light_now_index != 0)
        {
            going = 1;  // 已经发车
            if (!voice) // gps
            {
                zf_Gps();

                SetServoAngle(servoMid + angle);
            }
            else // voice
            {
                if (light_now_index != light_last_index)
                {
                    voice = 0;
                    sendDataToSore(0);
                }
            }
        }
        else // 到达终点停车
        {
            sendDataToSore(0);
            SetMotorDuty(0, 0);
            SetServoAngle(servoMid);
            voice = 0;
            if (going)
            {
                going = 0;
                GoFlag = 0;
            }
        }
    }
}
