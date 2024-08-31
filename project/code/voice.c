#include "arm_math.h" //��ѧ������
#include "math.h"
#include "zf_common_headfile.h"

#define ADC1 ADC1_CH24_P14_4
#define ADC2 ADC1_CH09_P12_5
#define ADC3 ADC1_CH25_P14_5
#define ADC4 ADC1_CH08_P12_4
int error = 1800;//��ȥֱ��ƫִ
void voice_init()
{
  /*adc��ʼ��*/
  adc_init(ADC1, ADC_12BIT);
  adc_init(ADC2, ADC_12BIT);
  adc_init(ADC3, ADC_12BIT);
  adc_init(ADC4, ADC_12BIT);
  pit_us_init(PIT_CH2, 100); // �ж�5ms
}

#define ADC_DATA_LEN 2048          // �ɼ����ݳ���
uint8 AdcFinishFlag = 0;           // adc�ɼ���ɱ�־λ
uint8 FftFinishFlag = 0;           // fft��ɱ�־λ
uint8 AdcBuffIndex = 0;            // adc˫buff������  ������ɵ��������
uint32 adcCount = 0;               // adc�����±�
int16 g_adc0Data[2][ADC_DATA_LEN]; // ADC����   adc0 1 2 3 �ɼ���˷��ź�
int16 g_adc1Data[2][ADC_DATA_LEN];
int16 g_adc2Data[2][ADC_DATA_LEN];
int16 g_adc3Data[2][ADC_DATA_LEN];

void VoiceGetSample_(int16 *adc_0, int16 *adc_1, int16 *adc_2, int16 *adc_3)
{
  int16 *data0 = adc_0;
  int16 *data1 = adc_1;
  int16 *data2 = adc_2;
  int16 *data3 = adc_3;

  uint8 adcIndex = 0;
  if (adcCount >= ADC_DATA_LEN)
  {
    adcCount = 0;
    AdcFinishFlag = 1;

    /* �л�buff������ */
    if (adcIndex == 0)
    {
      adcIndex = 1;
      AdcBuffIndex = 0;
    }
    else
    {
      adcIndex = 0;
      AdcBuffIndex = 1;
    }
  }
  if (!adcIndex) // ������0
  {
    data0[adcCount] = adc_convert(ADC1) - error;
    data1[adcCount] = adc_convert(ADC2) - error;
    data2[adcCount] = adc_convert(ADC3) - error;
    data3[adcCount] = adc_convert(ADC4) - error;
  }
  else // ������1
  {
    data0[adcCount + ADC_DATA_LEN] = adc_convert(ADC1) - error;
    data1[adcCount + ADC_DATA_LEN] = adc_convert(ADC2) - error;
    data2[adcCount + ADC_DATA_LEN] = adc_convert(ADC3) - error;
    data3[adcCount + ADC_DATA_LEN] = adc_convert(ADC4) - error;
  }
  adcCount++;
}

int findMax(float32_t arr[], int n)
{
  float32_t max = arr[0]; // �ݶ����ֵΪ�����һλ��
  int i;
  for (i = 0; i < n; i++)
  {
    if (arr[i] > max) // �������������ֵ��
      max = arr[i];
  }
  for (i = 0; i < n; i++) // �������ֵ����±ꡣ
  {
    if (max == arr[i])
      return i; // ͨ�����������±ꡣ
  }

  return 2048;
}

int nowtime;   // ����һ�νǶȵ�ʱ��
float x0, y0;  // ��Դ����
float g_angle; // ��Դ��λ��
int angle_error;
int angle__()
{
  fft_fly();
  /*����ת��*/
  if (g_angle > 180)
  {
    g_angle -= 360;
  }
  /*����ȡ*/
  angle_error = -g_angle;
  return angle_error;
}


float in_put_1[2048 * 4];  // Դ����0
float in_put_2[2048 * 4];  // Դ����1
float out_put_1[2048 * 4]; // �������洢������

float ou_put_all[2048 * 4];
float out_put_all[2048 * 2];

float Time_lag_0; // ʱ��
float Time_lag_1; // ʱ��

float MaxValue_subscript = 0; // ��ֵ��Ӧ�±�

int time___;
void fft_fly()
{
  if (AdcFinishFlag)
  {
    timer_start(TC_TIME2_CH2);
    /*����*/
    for (int i = 0; i < 2048; i++)
    {
      in_put_1[2 * i] = ((float)g_adc3Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // ��ȥֱ��ƫ��
      in_put_1[2 * i + 1] = 0.00;
      in_put_2[2 * i] = ((float)g_adc1Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // ��ȥֱ��ƫ��
      in_put_2[2 * i + 1] = 0.00;
    }
    /*����*/
    for (int i = 2048; i < 2048 * 2; i++) // ����,����֮������ݳ���Ϊ4096
    {
      in_put_1[2 * i] = 0.0;
      in_put_1[2 * i + 1] = 0.0;
      in_put_2[2 * i] = 0.0;
      in_put_2[2 * i + 1] = 0.0;
    }
    /*fft*/
    arm_cfft_instance_f32 fft_2048;          // ����FFT����
    arm_cfft_init_f32(&fft_2048, 2048 * 2);  // ��ʼ��FFT���� ������㳤��
    arm_cfft_f32(&fft_2048, in_put_1, 0, 1); // 32λ����FFT����
    arm_cfft_f32(&fft_2048, in_put_2, 0, 1); // 32λ����FFT����
    /*��������*/
    // ȡ������г˻�����
    arm_cmplx_conj_f32(in_put_1, out_put_1, 2048 * 2);
    arm_cmplx_mult_cmplx_f32(out_put_1, in_put_2, ou_put_all, 2048 * 2);
    /*��fft*/
    arm_cfft_init_f32(&fft_2048, 2048 * 2);    // ��ʼ��FFT���� ������㳤��
    arm_cfft_f32(&fft_2048, ou_put_all, 1, 1); // 32λ����FFT����
    for (int i = 0; i < 2048 * 2; i++)
    {
      out_put_all[i] = (float)sqrt(ou_put_all[2 * i] * ou_put_all[2 * i] + ou_put_all[2 * i + 1] * ou_put_all[2 * i + 1]); // ����ȡģ
    }
    float temp = 0;                // �м����
    for (int i = 0; i < 2048; i++) // ������ǰ��һ�������һ�뽻��λ��
    {
      temp = out_put_all[i];
      out_put_all[i] = out_put_all[i + 2048];
      out_put_all[i + 2048] = temp;
    }
    MaxValue_subscript = (float)findMax(out_put_all, 4096); // �ҵ����ֵ�±�
    Time_lag_0 = 2048 - MaxValue_subscript;                 // ʱ��Ϊ���ֵ�±�
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /*����*/
    for (int i = 0; i < 2048; i++)
    {
      in_put_1[2 * i] = ((float)g_adc0Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // ��ȥֱ��ƫ��
      in_put_1[2 * i + 1] = 0.00;
      in_put_2[2 * i] = ((float)g_adc2Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // ��ȥֱ��ƫ��
      in_put_2[2 * i + 1] = 0.00;
    }
    /*����*/
    for (int i = 2048; i < 2048 * 2; i++) // ����,����֮������ݳ���Ϊ4096
    {
      in_put_1[2 * i] = 0.0;
      in_put_1[2 * i + 1] = 0.0;
      in_put_2[2 * i] = 0.0;
      in_put_2[2 * i + 1] = 0.0;
    }
    /*fft*/
    arm_cfft_init_f32(&fft_2048, 2048 * 2);  // ��ʼ��FFT���� ������㳤��
    arm_cfft_f32(&fft_2048, in_put_1, 0, 1); // 32λ����FFT����
    arm_cfft_f32(&fft_2048, in_put_2, 0, 1); // 32λ����FFT����
    /*��������*/
    // ȡ������г˻�����
    arm_cmplx_conj_f32(in_put_1, out_put_1, 2048 * 2);
    arm_cmplx_mult_cmplx_f32(out_put_1, in_put_2, ou_put_all, 2048 * 2);
    /*��fft*/
    arm_cfft_init_f32(&fft_2048, 2048 * 2);    // ��ʼ��FFT���� ������㳤��
    arm_cfft_f32(&fft_2048, ou_put_all, 1, 1); // 32λ����FFT����
    for (int i = 0; i < 2048 * 2; i++)
    {
      out_put_all[i] = (float)sqrt(ou_put_all[2 * i] * ou_put_all[2 * i] + ou_put_all[2 * i + 1] * ou_put_all[2 * i + 1]); // ����ȡģ
    }
    temp = 0;                      // �м����
    for (int i = 0; i < 2048; i++) // ������ǰ��һ�������һ�뽻��λ��
    {
      temp = out_put_all[i];
      out_put_all[i] = out_put_all[i + 2048];
      out_put_all[i + 2048] = temp;
    }
    MaxValue_subscript = (float)findMax(out_put_all, 4096); // �ҵ����ֵ�±�
    Time_lag_1 = 2048 - MaxValue_subscript;                 // ʱ��Ϊ���ֵ�±�
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    x0 = Time_lag_0;                    // ����1�͹���3�������
    y0 = Time_lag_1;                    // ����2�͹���0�������
    g_angle = atanf(x0 / y0) * 57.2974; // ��Դ��λ��
    if (x0 > 0 && y0 > 0)
      g_angle = g_angle;
    if (x0 > 0 && y0 < 0)
      g_angle = 180 + g_angle;
    if (x0 < 0 && y0 < 0)
      g_angle = 180 + g_angle;
    if (x0 < 0 && y0 > 0)
      g_angle = 360 + g_angle;
    if (x0 == 0 && y0 > 0)
      g_angle = 0;
    if (x0 > 0 && y0 == 0)
      g_angle = 90;
    if (x0 == 0 && y0 < 0)
      g_angle = 180;
    if (x0 < 0 && y0 == 0)
      g_angle = 270;
    time___ = (int)timer_get(TC_TIME2_CH2) - time___;
    // timer_clear(TC_TIME2_CH2);
    //        timer_stop(TC_TIME2_CH2);
    AdcFinishFlag = 0;
  }
}
