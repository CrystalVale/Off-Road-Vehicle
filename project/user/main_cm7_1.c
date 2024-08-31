/*********************************************************************************************************************
 * CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� CYT4BB ��Դ���һ����
 *
 * CYT4BB ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          main_cm7_1
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          IAR 9.40.1
 * ����ƽ̨          CYT4BB
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"

int angle_error2;
uint32 voiceflag = 0;

// �������ݽ��ջص����� �������һ�����ķ�����Ϣ �˺��Ļᴥ���жϲ��ҿ����ڻص�������ȡ����
void my_ipc_callback(uint32 receive_data)
{
    voiceflag = receive_data;
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();             // ���Դ�����Ϣ��ʼ��

    SCB_DisableDCache(); // �ر�DCashe

    ipc_communicate_init(IPC_PORT_2, my_ipc_callback); // ��ʼ��IPCģ�� ѡ��˿�2 ��д�жϻص�����

    voice_init();

    timer_init(TC_TIME2_CH2, TIMER_MS);

    /*���pwm��ʼ��*/
    pwm_init(servo, 50, servoMid);

    while (true)
    {
        /*�����Ǽ���*/
        angle_error2 = angle__();
        ipc_send_data(angle_error2);

        // printf("angle_error2:%d\n", angle_error2);
        if (voiceflag)
        {
            if ((angle_error2 < 70 && angle_error2 > 20) || (angle_error2 > -70 && angle_error2 < -20))
            {
                SetServoAngle(angle_error2);//������
                SetMotorDuty(2500, 1);
                
            }
            else if (angle_error2 >= 70 || angle_error2 <= -70)
            {
                SetServoAngle(-1.5 * angle_error2);//��������
                SetMotorDuty(2500, 0);
            }
            else
            {
                SetServoAngle(servoMid);
                SetMotorDuty(2500, 1);
            }
        }
    }
}

// **************************** �������� ****************************
