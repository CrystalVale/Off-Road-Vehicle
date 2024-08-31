#include "zf_common_headfile.h"

int Scheme;// 1 Öð·É 0 ÁúÇñ
extern int utm_mode;

void SetServoAngle(float angle)
{
    angle = (fminf(fmaxf(angle, servoMid - 100), servoMid + 100));
    pwm_set_duty(servo, (uint32)angle);
}

void SetMotorDuty(uint32 duty, uint8 gpo)
{
    duty = (fmin(fmax(duty, 0), motorMaxDuty));
    pwm_set_duty(motor, duty);
    gpio_set_level(motorGpo, gpo);
}

void utmModeChange()
{
    if(utm_mode)
    {
        beaconPositionToUtm();
    }
}

void sendDataToSore(uint32 data)
{
    ipc_send_data(data);
    system_delay_ms(200);
}


pid_param_t PID_GPS;
pid_param_t PID_IMU;
pid_param_t PID_Init;
//pid
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid²ÎÊý³õÊ¼»¯º¯Êý
void PidInit(pid_param_t * pid)
{
    pid->kp        = 0;
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}
float PidLocCtrl(pid_param_t * pid, float error)
{
//    PID_GPS.kp=1;
//    PID_GPS.kd=1.05;


    PID_GPS.kp=1.1;
    PID_GPS.kd=5;

    PID_IMU.kp=1;//1.15
    PID_IMU.kd=7;

    /* ÀÛ»ýÎó²î */
    pid->integrator += error;

    /* Îó²îÏÞ·ù */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

//    if(pid->out>SERVO_MOTOR_LMAX)
//    {pid->out=SERVO_MOTOR_LMAX;}
//    if(pid->out<SERVO_MOTOR_RMIN)
//    {pid->out=SERVO_MOTOR_RMIN;}
    return pid->out;
//    printf("OUT:%d\n",pid->out);
}



