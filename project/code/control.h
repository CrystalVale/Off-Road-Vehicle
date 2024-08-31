void utmModeChange();
void SetMotorDuty(uint32 duty, uint8 gpo);
void SetServoAngle(float angle);
void sendDataToSore(uint32 data);
int SchemeSelection();
extern int Scheme;
typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //积分限幅

    float                out_p;  //KP输出
    float                out_i;  //KI输出
    float                out_d;  //KD输出
    float                out;    //pid输出

    float                integrator; //< 积分值
    float                last_error; //< 上次误差
    float                last_derivative;//< 上次误差与上上次误差之差
    unsigned long        last_t;     //< 上次时间
}pid_param_t;

extern pid_param_t PID_GPS;
extern pid_param_t PID_IMU;
extern pid_param_t PID_Init;
float PidLocCtrl(pid_param_t * pid, float error);
void PidInit(pid_param_t * pid);