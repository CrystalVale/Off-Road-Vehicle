#include "zf_common_headfile.h"

POINT point[30] = {0};
GPS gps = {0};
IMU imu = {0};
double angle_rad_umt = 3.1415926535897932 / 180.0; // �Ƕ�ת����
float rad_angle = 180.0 / 3.1415926;               // �Ƕ�ת����
double utmx[5], utmy[5], x_[5], y_[5];             // �����˲�����
float utmlat, utmlon;
int num = 0;   // ����ɼ�����
int index = 0; // ����ɼ�����
int utm_mode;  // GPSģʽѡ��
double lastx, lasty;
int light_now_index;
uint8 buffer_read_[20];
double _utm_x[3], _utm_y[3]; // �˶�ʱ�˲�����
int WriteFlag, ReadFlag;
int Daty_Z, T_M, T_N;
int fliter_flag;        // �˲���������־λ
int fliter_begin_mount; // �˲�����������λ

extern uint32 voiceAngle;
extern double distance;
extern double angle;
extern int GoFlag;
extern int voice;

int getPointflag = 0;

//--------------------------------------------------------------------------------------------------
// ��ʼ��
//--------------------------------------------------------------------------------------------------
int allInit()
{
    uart_init(UART_1, 115200, UART1_TX_P04_1, UART1_RX_P04_0);
    gnss_init(TAU1201);
    // imu963ra_init();
    // IMU_Gyro_Offset_Init();
    // adc_init(battery, ADC_8BIT);
    pwm_init(motor, initpwm, 0);
    gpio_init(motorGpo, GPO, 0, GPO_PUSH_PULL); // ��ʼ����������������
    pwm_init(servo, 50, servoMid);
    key_init(10);
    ips200_init(IPS200_TYPE_PARALLEL8);
    timer_init(TC_TIME2_CH0, TIMER_MS);
    flash_init();
    system_delay_init();
    pit_ms_init(PIT_CH0, 100);
    pit_ms_init(PIT_CH1, 100);
    gpio_init(switch_1, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� SWITCH1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(switch_2, GPI, GPIO_HIGH, GPI_PULL_UP); // ��ʼ�� SWITCH2 ���� Ĭ�ϸߵ�ƽ ��������
}
// ��ʼ�����
void initOk()
{
    if (allInit())
    {
        gpio_set_level(buzzzer, 1);
        system_delay_ms(200);
        gpio_set_level(buzzzer, 0);
    }
}
/********************************************GPS****************************************************/
void BubbleSort(double arr[], int n)
{
    int i;
    for (i = 1; i <= n - 1; i++)
    {
        for (int j = 1; j <= n - i; j++)
        {
            if (arr[j - 1] > arr[j])
            {
                double tmp = arr[j - 1];
                arr[j - 1] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}
// תutm����
void LonLat2UTM(double longitude, double latitude)
{
    double lon = longitude; // ����
    double lat = latitude;  // ά��
    double a = 6378.137;    // �������뾶
    double e = 0.0818192;   // �̶�����
    double k0 = 0.9996;     // ���뾭��ͶӰ��ĳ���Ϊ��ͶӰǰ��0.9996��
    double E0 = 500;
    double N0 = 0; // 0 km (Northern hemispher) or 10000 km (Southern hemisphere)

    // ���ݾ�γ�ȼ������ڵ�UTM���򣬼��㹫ʽ:zoneNumber = floor((longTemp + 180)/6) + 1//����ȡ������
    double zoneNumber = floor(lon / 6) + 31;
    // ����lambda0��ʽ��lambda0 = (zoneNumber �C 1) * 6 �C 180 + 3 (degree)
    double lambda0 = (zoneNumber - 1) * 6 - 180 + 3;
    lambda0 = lambda0 * angle_rad_umt; // ת����
    // ����ά��ת����
    double phi = lat * angle_rad_umt;    // ά�Ȼ�����
    double lambda = lon * angle_rad_umt; // ���Ȼ�����

    // ʹ�ñ�������
    double v = 1 / sqrt(1 - pow(e * sin(phi), 2));
    double A = (lambda - lambda0) * cos(phi);
    double T = pow(tan(phi), 2);
    double C = pow(e, 2) / (1 - pow(e, 2)) * pow(cos(phi), 2);
    double s = (1 - pow(e, 2) / 4 - 3 * pow(e, 4) / 64 - 5 * pow(e, 6) / 256) * phi - (3 * pow(e, 2) / 8 + 3 * pow(e, 4) / 32 + 45 * pow(e, 6) / 1024) * sin(2 * phi) + (15 * pow(e, 4) / 256 + 45 * pow(e, 6) / 1024) * sin(4 * phi) - 35 * pow(e, 6) / 3072 * sin(6 * phi);
    // ����ϵ��γ��ת��
    utmlon = E0 + k0 * a * v * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T) * pow(A, 5) / 120);
    utmlat = N0 + k0 * a * (s + v * tan(phi) * (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + (61 - 58 * T + T * T) * pow(A, 6) / 720));
    // ����λת��ΪmΪ��λ
    utmlon *= 1000;
    utmlat *= 1000;
}
// ��ȡGPS���ݴ���ṹ��
void getGpsData()
{
    gnss_data_parse();
    gps.direction = gnss.direction;
    gps.lat = gnss.latitude;
    gps.lon = gnss.longitude;
    gps.satellite_used = gnss.satellite_used;
}
// ��ȡ�ű��λ��
void getBeaconPosition()
{
    if (Scheme)
    {
        if (getPointflag)
        {
            for (int i = 4; i > 0; i--)
            {
                x_[i] = x_[i - 1];
                y_[i] = y_[i - 1];
            }
            x_[0] = gps.lon;
            y_[0] = gps.lat;

            if (num >= 5)
            {
                BubbleSort(x_, 5);
                BubbleSort(y_, 5);

                point[gps.num].lon = (x_[1] + x_[2] + x_[3]) / 3;
                point[gps.num].lat = (y_[1] + y_[2] + y_[3]) / 3;

                gps.num++;
                num = 0;
                getPointflag = 0;
            }
            else
            {
                num++;
            }
        }
    }
    else
    {
        if (getPointflag)
        {
            LonLat2UTM(gps.lon, gps.lat);
            for (int i = 4; i > 0; i--)
            {
                x_[i] = x_[i - 1];
                y_[i] = y_[i - 1];
            }
            x_[0] = utmlon;
            y_[0] = utmlat;

            if (num >= 5)
            {
                BubbleSort(x_, 5);
                BubbleSort(y_, 5);

                point[gps.num].utm_lon = (x_[1] + x_[2] + x_[3]) / 3;
                point[gps.num].utm_lat = (y_[1] + y_[2] + y_[3]) / 3;

                gps.num++;
                num = 0;
                getPointflag = 0;
            }
            else
            {
                num++;
            }
        }
    }
}
// ת���ű������ϵ
void beaconPositionToUtm()
{
    for (int i = 0; i < gps.num; i++)
    {
        LonLat2UTM(point[i].lon, point[i].lat);
        point[i].utm_lon = utmlon;
        point[i].utm_lat = utmlat;
    }
    ips200_show_string(0, 16 * 16, "OK");
}
// ��ȡ�Ƕ�UTM
double getAngle_Utm(double x_1, double y_1, double x_2, double y_2)
{
    double angle_got = atan((x_2 - x_1) / (y_2 - y_1)); // ������
    angle_got *= rad_angle;

    if ((x_2 - x_1) > 0 && (y_2 - y_1) > 0) // һ����
        return angle_got;
    if ((x_2 - x_1) < 0 && (y_2 - y_1) < 0) // ������
        return (180.0 + angle_got);
    if ((x_2 - x_1) < 0 && (y_2 - y_1) > 0) // ������
        return (360.0 + angle_got);
    if ((x_2 - x_1) > 0 && (y_2 - y_1) < 0) // ������
        return 180.0 + angle_got;

    if ((x_2 - x_1) == 0 && (y_2 - y_1) > 0) // һ���޶����޽���
        return 0.0;
    if ((x_2 - x_1) == 0 && (y_2 - y_1) < 0) // ������++�����޽���
        return 180.0;

    if ((x_2 - x_1) > 0 && (y_2 - y_1) == 0) // һ���޺������޽���
        return 90.0;
    if ((x_2 - x_1) < 0 && (y_2 - y_1) == 0) // �����޺������߽���
        return 270.0;
    return angle_got;
}
// ��ȡ�Ƕ�GPS_UTM
double getAngle_Gps_Utm(double x1, double y1, double x2, double y2)
{
    /*���㷽λ��*/
    double angle_tar = getAngle_Utm(x1, y1, x2, y2);
    /*���㺽���*/
    double angle_now = getAngle_Utm(lastx, lasty, x1, y1);
    /*���㷽λ���*/
    double tar_angle = (angle_now - angle_tar);

    /*��¼��һ������ֵ*/
    lastx = x1;
    lasty = y1;
    /*�Ƕ�ת��������ʹת��Ƕȳ�����С�Ƕȱ���ת�����*/
    /*�Ӽ�����Ҫ����ת��ҹ�*/
    if (tar_angle < -180) // ��ʱ����ת���
    {
        tar_angle = tar_angle + 360;
    }

    else if (tar_angle >= 180) // ����ת���
    {
        tar_angle = tar_angle - 360;
    }
    return tar_angle;
}
// ��ȡ�Ƕ�GPS
double getAngle_Gps(double lon1, double lat1, double lon2, double lat2)
{
    double azimuth = get_two_points_azimuth(lat1, lon1, lat2, lon2);
    double azimuthError = gnss.direction - azimuth;

    azimuthError = azimuthError > 180 ? azimuthError - 360 : azimuthError;
    azimuthError = azimuthError < -180 ? azimuthError + 360 : azimuthError;

    return azimuthError;
}
// ��ȡ����UTM
double getDistance_Gps_Utm(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}
// ��ȡ����
double getDistance_Gps(double lon1, double lat1, double lon2, double lat2)
{
    return get_two_points_distance(lat1, lon1, lat2, lon2);
}
// �˶��˲�
void moveFilter()
{
    /*��¼*/
    _utm_x[2] = _utm_x[1];
    _utm_x[1] = _utm_x[0];
    _utm_x[0] = gps.lon;

    _utm_y[2] = _utm_y[1];
    _utm_y[1] = _utm_y[0];
    _utm_y[0] = gps.lat;
    /*�˶��˲�*/
    if (!fliter_flag) // δ�����˲�
    {
        fliter_begin_mount++;
        if (fliter_begin_mount >= 3)
        {
            BubbleSort(_utm_x, 3);
            BubbleSort(_utm_y, 3);
            gps.nlon = _utm_x[2];
            gps.nlat = _utm_y[2];
            fliter_flag = 1;
            fliter_begin_mount = 0;
        }
    }
    else
    {
        BubbleSort(_utm_x, 3);
        BubbleSort(_utm_y, 3);
        gps.nlon = _utm_x[2];
        gps.nlat = _utm_y[2];
    }
}
/******************************************MESSAGE**************************************************/
// ��ȡ�ű��
void getLightIndex()
{
    int i = 0;
    uint8 data;
    while (i < 10)
    {
        if (uart_query_byte(UART_1, &data))
        {
            buffer_read_[i] = data;
            i++;
        }
    }
    int j = 0;
    while (j < 10)
    {
        if (buffer_read_[j] == 102 && buffer_read_[j + 3] == 136)
        {
            light_now_index = buffer_read_[j + 1];
            break;
        }
        j++;
    }
}
/********************************************KEY****************************************************/
// ����ɨ��
void keyScan()
{
    key_scanner();
    if (key_get_state(KEY_1))
    {
        getPointflag = 1;
        key_clear_state(KEY_1);
    }

    if (key_get_state(KEY_2))
    {
        WriteFlag = 1;
        key_clear_state(KEY_2);
    }

    if (key_get_state(KEY_3))
    {
        ReadFlag = 1;
        key_clear_state(KEY_3);
    }

    if (key_get_state(KEY_4))
    {
        GoFlag = 1;
        key_clear_state(KEY_4);
    }
}
/********************************************IPS****************************************************/
// ˢ����Ļ
void show()
{
    ips200_show_string(0, 0, "PointNum:"); // ������
    ips200_show_int(120, 16 * 0, gps.num, 2);
    ips200_show_string(0, 16 * 1, "point:");
    if (gps.num > 0)
    {
        ips200_show_float(80, 16 * 1, point[gps.num - 1].lat, 3, 6);
        ips200_show_float(80, 16 * 2, point[gps.num - 1].lon, 3, 6);
    }

    ips200_show_string(0, 16 * 3, "Now:");

    ips200_show_float(80, 16 * 3, gps.lat, 3, 6);
    ips200_show_float(80, 16 * 4, gps.lon, 3, 6);

    ips200_show_string(0, 16 * 7, "Light:");
    ips200_show_int(60, 16 * 7, light_now_index, 2);

    ips200_show_string(0, 16 * 10, "strong:");
    ips200_show_int(60, 16 * 10, gps.satellite_used, 3);
    ips200_show_string(0, 16 * 11, "voiceAngle:");
    ips200_show_int(120, 16 * 11, voiceAngle, 4);
    ips200_show_string(0, 16 * 12, "Go:");
    ips200_show_int(120, 16 * 12, GoFlag, 2);
    ips200_show_string(0, 16 * 13, "voice:");
    ips200_show_int(120, 16 * 13, voice, 2);
    ips200_show_string(0, 16 * 14, "dis:");
    ips200_show_int(120, 16 * 14, distance, 2);
    ips200_show_string(0, 16 * 15, "direction:");
    ips200_show_float(120, 16 * 15, gnss.direction, 3, 3);
}
// ��ص�ѹ
void Battery_Health()
{
    float num = 0;
    for (int i = 0; i < 3; i++)
    {
        num += (adc_convert(battery) * 3.3 * 11) / 255;
    }
    ips200_show_string(0, 16 * 18, "Battery:");
    ips200_show_float(80, 16 * 18, num / 3.0 + 0.2, 2, 2);
}
/********************************************FLASH****************************************************/
// ��ȡflash
void readFlash()
{
    if (ReadFlag)
    {
        flash_buffer_clear(); // ��ջ�����

        if (flash_check(0, 11))
        {
            flash_read_page_to_buffer(0, 11, 60); // �����ݴ�FLASHָ������ҳ����뵽������
            int j = 0;
            gps.num = (int)flash_union_buffer[0].uint32_type;
            for (int i = 1; i <= gps.num * 2; i++)
            {
                point[j].lat = (double)(flash_union_buffer[i].float_type / 10000.0);
                i++;
                point[j].lon = (double)(flash_union_buffer[i].float_type / 10000.0);
                j++;
            }
        }
    }

    ReadFlag = 0;
}

// д��flash
void writeFlash()
{
    if (WriteFlag)
    {
        flash_buffer_clear(); // ��ջ�����

        flash_union_buffer[0].uint32_type = (uint32)gps.num;
        int j = 0;
        for (int i = 1; i <= gps.num * 2; i++)
        {

            float lat = point[j].lat * 10000;
            float lon = point[j].lon * 10000;

            flash_union_buffer[i].float_type = lat;
            i++;
            flash_union_buffer[i].float_type = lon;
            j++;
        }
        flash_write_page_from_buffer(0, 11, 60); // ������������д�뵽FLASHָ������ҳ��
    }

    WriteFlag = 0;
}

// flash����
void flash()
{
    readFlash();
    writeFlash();
}
/********************************************IMU****************************************************/
// ��Ʈ��ʼ��
void IMU_Gyro_Offset_Init()
{
    for (uint16 i = 0; i < 1000; i++)
    {

        imu963ra_get_gyro();
        imu.gpro_init_x += imu963ra_gyro_x;
        imu.gpro_init_y += imu963ra_gyro_y;
        imu.gpro_init_z += imu963ra_gyro_z;
        system_delay_ms(5); // ��� 1Khz
    }
    imu.gpro_init_x /= 1000;
    imu.gpro_init_y /= 1000;
    imu.gpro_init_z /= 1000;
}
// �Խ��ٶȽ��л���
void IMU_YAW_integral()
{
    imu.gyro_x = ((float)imu963ra_gyro_x - imu.gpro_init_x) * PI / 180 / 14.3f;
    imu.gyro_y = ((float)imu963ra_gyro_y - imu.gpro_init_y) * PI / 180 / 14.3f;
    imu.gyro_z = ((float)imu963ra_gyro_z - imu.gpro_init_z) * PI / 180 / 14.3f;
    if (imu.gyro_z < 0.015 && imu.gyro_z > -0.015) // �˲�
    {
        Daty_Z -= 0;
    }
    else
    {
        IMU_Handle_180();
    }
}
// �滮
void IMU_Handle_180()
{
    Daty_Z -= RAD_TO_ANGLE(imu.gyro_z * 0.005);

    if ((Daty_Z > 0 && Daty_Z <= 180) || (Daty_Z < 0 && Daty_Z >= (-180))) // ˳ʱ��
    {
        Daty_Z = +Daty_Z;
    }
    else if (Daty_Z > 180 && Daty_Z <= 360)
    {
        Daty_Z -= 360;
    }
    else if (Daty_Z < (-180) && Daty_Z >= (-360))
    {
        Daty_Z += 360;
    }
}

/********************************************һ�׻����˲���̬����****************************************************/
#define Ka 0.90  // ���ٶȽ���Ȩ��
#define dt 0.005 // �����������λ���룩

#define ANGLE_APPROX_COEFF 0 // ����Ǳƽ�ϵ��

#define OFFSET_COUNT 200 // ��Ư�ⶨ������
// �ںϼ��ٶȼƣ�acc_m���������ǣ�gyro_m�������������㵱ǰ�ĽǶ�
float FOCF(float acc_m, float gyro_m, float *last_angle)
{
    float temp_angle;
    temp_angle = Ka * acc_m + (1 - Ka) * (*last_angle + gyro_m * dt); // ���ٶȶԲ���������ּ����ϴν���Ƕȼ�Ϊ�����������Ƴ��ĽǶ�
    *last_angle = temp_angle;
    return temp_angle;
}
// ��̬����
void IMU_Attitude_Calculate()
{
    // ���ݴ���
    imu.Roll_a = atan2(imu963ra_acc_x, imu963ra_acc_z) / (PI / 180); // ax����az�������к�����Ϊ�Ӽ��ٶȼ����Ƴ��ĽǶ�
    imu.Pitch_a = atan2(imu963ra_acc_y, imu963ra_acc_z) / (PI / 180);
    imu.Roll_g = -(imu963ra_gyro_y) / 14.3; // �����������Ƴ��Ľ��ٶȣ�14.3������������������
    imu.Pitch_g = -(imu963ra_gyro_x) / 14.3;

    // һ�׻����˲�
    imu.Roll = FOCF(imu.Roll_a, imu.Roll_g, &imu.lastRoll);
    imu.Pitch = FOCF(imu.Pitch_a, imu.Pitch_g, &imu.lastPitch);

    imu.Yaw += -(imu963ra_gyro_z) / 14.3 * dt;
    // Yaw������
    if (imu.Yaw < gnss.direction)
    {
        imu.Yaw += ANGLE_APPROX_COEFF;
    }
    else if (imu.Yaw > gnss.direction)
    {
        imu.Yaw -= ANGLE_APPROX_COEFF;
    }
}
// ��ȡ IMU963RA �ļ��ٶȺͽ��ٶȲ���ֵ
void IMU_get_data()
{
    imu963ra_get_acc();  // ��ȡ IMU963RA �ļ��ٶȲ���ֵ
    imu963ra_get_gyro(); // ��ȡ IMU963RA �Ľ��ٶȲ���ֵ
    imu963ra_gyro_x -= imu.gpro_init_x;
    imu963ra_gyro_y -= imu.gpro_init_y;
    imu963ra_gyro_z -= imu.gpro_init_z;
}
