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
    float                imax;       //�����޷�

    float                out_p;  //KP���
    float                out_i;  //KI���
    float                out_d;  //KD���
    float                out;    //pid���

    float                integrator; //< ����ֵ
    float                last_error; //< �ϴ����
    float                last_derivative;//< �ϴ���������ϴ����֮��
    unsigned long        last_t;     //< �ϴ�ʱ��
}pid_param_t;

extern pid_param_t PID_GPS;
extern pid_param_t PID_IMU;
extern pid_param_t PID_Init;
float PidLocCtrl(pid_param_t * pid, float error);
void PidInit(pid_param_t * pid);