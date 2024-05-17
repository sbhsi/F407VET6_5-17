/**			                                                    
		   ____                    _____ _______ _____       XTARK@���˴���
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   �����˿�����
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ��˾��վ�� www.xtark.cn   www.tarkbot.com
  * �Ա����̣� https://xtark.taobao.com  
  * ����΢�ţ� ���˴��£���ע���ںţ���ȡ���¸�����Ѷ��
  *      
  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  ����������PID�ٶȿ���
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_speed.h"
#include "ax_robot.h"


/**
  * @��  ��  ���PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ ,��Χ����250��
  *          spd_current: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//PID���������PWMֵ
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//����PWM����ֵ
	return motor_pwm_out;
}	

//��̬��,pd
int Gesture(float Angle, float Gyro)
{
	float AngleBias;  //��̬��ƫ��
	int GPWM;  //��̬��PWM���
	
	AngleBias=Angle-BalanAngle;  //������̬��ƫ��
	GPWM=Kp_Gesture*AngleBias+Kd_Gesture*(Gyro-0);  //������̬��PWM���
	if(GPWM > 4200) GPWM = 4200;
	else if (GPWM<-4200) GPWM = -4200;
	
	return GPWM;
}
//�ٶȻ���PI������
int Speed(int EcA, int EcB)
{
	static float EncoderBias=0,	EB_Integral=0, EncoderBiaslast = 0;  //�ٶ�ƫ����ƫ�����
	int SPWM;  //�ٶȻ�PWM���
	EncoderBias=EncoderBiaslast*0.7+(EcA+EcB-0)*0.3;  //�ٶ�ƫ���ͨ�˲�
	EncoderBiaslast = EncoderBias;
	EB_Integral+=EncoderBias;  //ƫ�����
	if (EB_Integral>10000) EB_Integral=10000;  //�����޷�
	if (EB_Integral<-10000) EB_Integral=-10000;  //�����޷�
	SPWM=Kp_Speed*EncoderBias+Ki_Speed*EB_Integral;  //�����ٶȻ�PWM���
	return SPWM;
}
/**
  * @��  ��  ���PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//PID���������PWMֵ
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//����PWM����ֵ
	return motor_pwm_out;
}

/**
  * @��  ��  ���PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_SPEED_PidCtlC(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//PID���������PWMֵ
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//����PWM����ֵ
	return motor_pwm_out;
}

/**
  * @��  ��  ���PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_SPEED_PidCtlD(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias,bias_last;

	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//PID���������PWMֵ
	motor_pwm_out += ax_motor_kp*bias + ax_motor_kd*(bias-bias_last);
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if(motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//����PWM����ֵ
	return motor_pwm_out;
}


/******************* (C) ��Ȩ 2023 XTARK **************************************/

