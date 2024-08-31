#include "arm_math.h" //数学函数库
#include "math.h"
#include "zf_common_headfile.h"

#define ADC1 ADC1_CH24_P14_4
#define ADC2 ADC1_CH09_P12_5
#define ADC3 ADC1_CH25_P14_5
#define ADC4 ADC1_CH08_P12_4
int error = 1800;//减去直流偏执
void voice_init()
{
  /*adc初始化*/
  adc_init(ADC1, ADC_12BIT);
  adc_init(ADC2, ADC_12BIT);
  adc_init(ADC3, ADC_12BIT);
  adc_init(ADC4, ADC_12BIT);
  pit_us_init(PIT_CH2, 100); // 中断5ms
}

#define ADC_DATA_LEN 2048          // 采集数据长度
uint8 AdcFinishFlag = 0;           // adc采集完成标志位
uint8 FftFinishFlag = 0;           // fft完成标志位
uint8 AdcBuffIndex = 0;            // adc双buff缓冲区  缓冲完成的区域序号
uint32 adcCount = 0;               // adc数组下标
int16 g_adc0Data[2][ADC_DATA_LEN]; // ADC数据   adc0 1 2 3 采集麦克风信号
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

    /* 切换buff缓冲区 */
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
  if (!adcIndex) // 缓冲区0
  {
    data0[adcCount] = adc_convert(ADC1) - error;
    data1[adcCount] = adc_convert(ADC2) - error;
    data2[adcCount] = adc_convert(ADC3) - error;
    data3[adcCount] = adc_convert(ADC4) - error;
  }
  else // 缓冲区1
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
  float32_t max = arr[0]; // 暂定最大值为数组第一位。
  int i;
  for (i = 0; i < n; i++)
  {
    if (arr[i] > max) // 遍历数组获得最大值。
      max = arr[i];
  }
  for (i = 0; i < n; i++) // 利用最大值获得下标。
  {
    if (max == arr[i])
      return i; // 通过函数返回下标。
  }

  return 2048;
}

int nowtime;   // 结算一次角度的时间
float x0, y0;  // 声源坐标
float g_angle; // 声源方位角
int angle_error;
int angle__()
{
  fft_fly();
  /*象限转换*/
  if (g_angle > 180)
  {
    g_angle -= 360;
  }
  /*误差获取*/
  angle_error = -g_angle;
  return angle_error;
}


float in_put_1[2048 * 4];  // 源波形0
float in_put_2[2048 * 4];  // 源波形1
float out_put_1[2048 * 4]; // 计算结果存储处数组

float ou_put_all[2048 * 4];
float out_put_all[2048 * 2];

float Time_lag_0; // 时延
float Time_lag_1; // 时延

float MaxValue_subscript = 0; // 峰值对应下标

int time___;
void fft_fly()
{
  if (AdcFinishFlag)
  {
    timer_start(TC_TIME2_CH2);
    /*输入*/
    for (int i = 0; i < 2048; i++)
    {
      in_put_1[2 * i] = ((float)g_adc3Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // 减去直流偏置
      in_put_1[2 * i + 1] = 0.00;
      in_put_2[2 * i] = ((float)g_adc1Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // 减去直流偏置
      in_put_2[2 * i + 1] = 0.00;
    }
    /*补零*/
    for (int i = 2048; i < 2048 * 2; i++) // 补零,补零之后的数据长度为4096
    {
      in_put_1[2 * i] = 0.0;
      in_put_1[2 * i + 1] = 0.0;
      in_put_2[2 * i] = 0.0;
      in_put_2[2 * i + 1] = 0.0;
    }
    /*fft*/
    arm_cfft_instance_f32 fft_2048;          // 定义FFT对象
    arm_cfft_init_f32(&fft_2048, 2048 * 2);  // 初始化FFT对象 赋予计算长度
    arm_cfft_f32(&fft_2048, in_put_1, 0, 1); // 32位浮点FFT运算
    arm_cfft_f32(&fft_2048, in_put_2, 0, 1); // 32位浮点FFT运算
    /*共轭数乘*/
    // 取共轭并进行乘积运算
    arm_cmplx_conj_f32(in_put_1, out_put_1, 2048 * 2);
    arm_cmplx_mult_cmplx_f32(out_put_1, in_put_2, ou_put_all, 2048 * 2);
    /*逆fft*/
    arm_cfft_init_f32(&fft_2048, 2048 * 2);    // 初始化FFT对象 赋予计算长度
    arm_cfft_f32(&fft_2048, ou_put_all, 1, 1); // 32位浮点FFT运算
    for (int i = 0; i < 2048 * 2; i++)
    {
      out_put_all[i] = (float)sqrt(ou_put_all[2 * i] * ou_put_all[2 * i] + ou_put_all[2 * i + 1] * ou_put_all[2 * i + 1]); // 复数取模
    }
    float temp = 0;                // 中间变量
    for (int i = 0; i < 2048; i++) // 将数据前面一半与后面一半交换位置
    {
      temp = out_put_all[i];
      out_put_all[i] = out_put_all[i + 2048];
      out_put_all[i + 2048] = temp;
    }
    MaxValue_subscript = (float)findMax(out_put_all, 4096); // 找到最大值下标
    Time_lag_0 = 2048 - MaxValue_subscript;                 // 时延为最大值下标
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /*输入*/
    for (int i = 0; i < 2048; i++)
    {
      in_put_1[2 * i] = ((float)g_adc0Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // 减去直流偏置
      in_put_1[2 * i + 1] = 0.00;
      in_put_2[2 * i] = ((float)g_adc2Data[AdcBuffIndex][i] * 3.3 / 4096) - 1.3; // 减去直流偏置
      in_put_2[2 * i + 1] = 0.00;
    }
    /*补零*/
    for (int i = 2048; i < 2048 * 2; i++) // 补零,补零之后的数据长度为4096
    {
      in_put_1[2 * i] = 0.0;
      in_put_1[2 * i + 1] = 0.0;
      in_put_2[2 * i] = 0.0;
      in_put_2[2 * i + 1] = 0.0;
    }
    /*fft*/
    arm_cfft_init_f32(&fft_2048, 2048 * 2);  // 初始化FFT对象 赋予计算长度
    arm_cfft_f32(&fft_2048, in_put_1, 0, 1); // 32位浮点FFT运算
    arm_cfft_f32(&fft_2048, in_put_2, 0, 1); // 32位浮点FFT运算
    /*共轭数乘*/
    // 取共轭并进行乘积运算
    arm_cmplx_conj_f32(in_put_1, out_put_1, 2048 * 2);
    arm_cmplx_mult_cmplx_f32(out_put_1, in_put_2, ou_put_all, 2048 * 2);
    /*逆fft*/
    arm_cfft_init_f32(&fft_2048, 2048 * 2);    // 初始化FFT对象 赋予计算长度
    arm_cfft_f32(&fft_2048, ou_put_all, 1, 1); // 32位浮点FFT运算
    for (int i = 0; i < 2048 * 2; i++)
    {
      out_put_all[i] = (float)sqrt(ou_put_all[2 * i] * ou_put_all[2 * i] + ou_put_all[2 * i + 1] * ou_put_all[2 * i + 1]); // 复数取模
    }
    temp = 0;                      // 中间变量
    for (int i = 0; i < 2048; i++) // 将数据前面一半与后面一半交换位置
    {
      temp = out_put_all[i];
      out_put_all[i] = out_put_all[i + 2048];
      out_put_all[i + 2048] = temp;
    }
    MaxValue_subscript = (float)findMax(out_put_all, 4096); // 找到最大值下标
    Time_lag_1 = 2048 - MaxValue_subscript;                 // 时延为最大值下标
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    x0 = Time_lag_0;                    // 硅麦1和硅麦3做互相关
    y0 = Time_lag_1;                    // 硅麦2和硅麦0做互相关
    g_angle = atanf(x0 / y0) * 57.2974; // 声源方位角
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
