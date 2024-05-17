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
  * @��  ��  MPU6050 ����
  *
  ******************************************************************************
  * @˵  ��
  *
  * 1.MPU6050������ٶȴ�����������������
  * 2.IICͨ�Ų���IO��ģ�ⷽʽ
  *
  ******************************************************************************
  */
#include "ax_mpu6050.h" 
#include "ax_sys.h"
#include "ax_delay.h"
//#include "inv_mpu.c"
#include "inv_mpu_dmp_motion_driver.h"
//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //���ģʽ

//IO��������	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //����SDA 

//��������
static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data);
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len);

#define q30  1073741824.0f
short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x03) {                   //����0x03ΪMPU6050
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);			//��ȡ��ǰ�����ǵ�״̬
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);			//���ݶ�ȡ��״̬����У׼
		
        mpu_get_accel_sens(&accel_sens);	//��ȡ��ǰ���ٶȼƵ�״̬
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);			//���ݶ�ȡ��״̬����У׼
		//printf("setting bias succesfully ......\r\n");
    }
}

/**
  * @��  ��  MPU6050��������ʼ��
  * @��  ��  ��	  
  * @����ֵ  ��
  */
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
void IIC_Init(void)
{			
	//��ʼ��I2C2
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

	//SCL����Ϊ�������
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	//SDA����Ϊ��©���
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
    IIC_SCL=1;
    IIC_SDA=1;
}
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;            // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

void AX_MPU6050_Init(void)
{	
	//��ʼ��I2C2
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

	//SCL����Ϊ�������
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	//SDA����Ϊ��©���
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
    IIC_SCL=1;
    IIC_SDA=1;
	
	//��λ����MPU6050
    MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x80);      //��λ
	AX_Delayus(100000);  	//�ȴ��������ȶ�
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x00);      //�������״̬
	
	//����MPU6050�Ĵ���  
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,0x18);      //���������� Ĭ��2000deg/s
   MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,0x00);     //���ټ����� Ĭ��2g	
	MPU6050_WriteRegister(MPU6050_INT_EN_REG,0x00);      //�ر��ж�
	MPU6050_WriteRegister(MPU6050_FIFO_EN_REG,0x00);       //�ر�FIFO
	
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x01);      //����CLKSEL,PLL X��Ϊ�ο�
	MPU6050_WriteRegister(MPU6050_PWR_MGMT2_REG,0x00); 	    //���ٶ��������Ƕ�����	

}


/**
  * @��  ��  MPU6050���ü��ٶ����� 
  * @��  ��  range�� ���ٶ�����2g��4g��8g��16g
  *          �����õļ��ٶ�����ACC_RANGE_2G��ACC_RANGE_4G��ACC_RANGE_8G��ACC_RANGE_16G
  * @����ֵ  ��
  */
void AX_MPU6050_SetAccRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,range<<3);
}


/**
  * @��  ��  MPU6050��������������
  * @��  ��  range ����������250��/S��500��/S��1000��/S��2000��/S
  *          �����õ�����������GYRO_RANGE_250��GYRO_RANGE_500��GYRO_RANGE_1000��GYRO_RANGE_2000	
  * @����ֵ  ��
  */
void AX_MPU6050_SetGyroRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,range<<3);
}

/**
  * @��  ��  MPU6050���������ǲ�����
  * @��  ��  smplrate �����ǲ����ʣ���Χ10~1000Hz	  
  * @����ֵ  ��
  */
void AX_MPU6050_SetGyroSmplRate(uint16_t smplrate)
{	
	if(smplrate>1000)
		smplrate = 1000;
	if(smplrate<10)
		smplrate = 10;
	
	MPU6050_WriteRegister(MPU6050_SAMPLE_RATE_REG,(uint8_t)(1000/smplrate -1));	
}

/**
  * @��  ��  MPU6050���õ�ͨ�˲�������
  * @��  ��  bandwidth ��ͨ�˲�������
  *          �����õĴ��� DLPF_ACC184_GYRO188��DLPF_ACC94_GYRO98��DLPF_ACC44_GYRO42��
  *                        DLPF_ACC21_GYRO20��DLPF_ACC10_GYRO10��DLPF_ACC5_GYRO5
  * @����ֵ  ��
  */
void AX_MPU6050_SetDLPF(uint8_t bandwidth)
{
	MPU6050_WriteRegister(MPU6050_CFG_REG,bandwidth);
}

/**
  * @��  ��  MPU6050��ȡ�������¶�ֵ
  * @��  ��  ��	  
  * @����ֵ  �������¶�ֵ��
  */
float AX_MPU6050_GetTempValue(void)
{	
	uint8_t buf[2];
	int16_t tmp;

	MPU6050_ReadRegister(MPU6050_TEMP_OUTH_REG,buf,2);

	tmp = (buf[0]<<8)| buf[1];
	
	return ( 36.53f + ((double)tmp/340.0f) );	
}


/**
  * @��  ��  MPU6050��ȡ������ٶȼĴ������ֵ
  * @��  ��  pbuf����ȡ�����ݻ�����ָ�� 
  * @����ֵ  ��
  */
void AX_MPU6050_GetAccData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	//��ȡ���ٶ�����
	MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG,buf,6);
	
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}

/**
  * @��  ��  MPU6050��ȡ���������ǼĴ������ֵ
  * @��  ��  pbuf����ȡ�����ݻ�����ָ�� 
  * @����ֵ  ��
  */
void AX_MPU6050_GetGyroData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	//��ȡ����������
	MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG,buf,6);
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}



//--------------------------I2C ��ʼ����������-----------------------------------------


/**
  * @��  ��  ����IIC��ʼ�ź�
  * @��  ��  ��
  * @����ֵ  ����״̬
  */
uint8_t IIC_Start(void)
{
	//SDA�����
	SDA_OUT();     
	
	//START�ź�: ��SCLΪ��ʱ, SDA�Ӹ߱�ɵ�, ��ʾ��ʼ�ź�
	IIC_SDA=1;
	
	if(!READ_SDA)
	{
		return 0;	
	}
		
	IIC_SCL=1;
	AX_Delayus(1);
	
 	IIC_SDA=0;
	if(READ_SDA)
		return 0;
	
	AX_Delayus(1);
	
	//ǯסI2C���ߣ�׼�����ͻ�������� 
	IIC_SCL=0;
	
	return 1;
}

/**
  * @��  ��  ����IICֹͣ�ź�
  * @��  ��  ��
  * @����ֵ  ��
  */	  
void IIC_Stop(void)
{
	//SDA�����
	SDA_OUT(); 
	
	//STOP�ź�: ��SCLΪ��ʱ, SDA�ӵͱ�ɸ�, ��ʾֹͣ�ź�
	IIC_SCL=0;
	IIC_SDA=0;
 	AX_Delayus(1);
	
	//����I2C���߽����ź�
	IIC_SCL=1; 
	IIC_SDA=1;
	AX_Delayus(1);							   	
}

/**
  * @��  ��  �ȴ�Ӧ���źŵ���
  * @��  ��  ��
  * @����ֵ  0������Ӧ��ɹ�  1������Ӧ��ʧ��
  */
uint8_t  IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	
	//SDA����Ϊ����  
	SDA_IN();     
	
	//�����ͷ�SDA��(��ʱ�ⲿ������������SDA��)
	IIC_SDA=1;
	AX_Delayus(1);	
	
	//SCL=1, ��ʱ�ӻ����Է���ACK
	IIC_SCL=1;
	AX_Delayus(1);	
	
	//�ȴ�Ӧ��
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 0;
		}
		
	    //AX_Delayus(1);
	}
	
	//SCL=0, ����ACK���	
	IIC_SCL=0;
	AX_Delayus(1);
	
	return 1;  
} 

/**
  * @��  ��  ����ACKӦ�����IIC��ʼ�ź�
  * @��  ��  ��
  * @����ֵ  ��
  */
void IIC_Ack(void)
{
	IIC_SCL=0;
	
	//SDA�����
	SDA_OUT();
	
	IIC_SDA=0;
	AX_Delayus(1);
	
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}

/**
  * @��  ��  ����NACKӦ��
  * @��  ��  ��
  * @����ֵ  ��
  */	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	
	//SDA�����
	SDA_OUT();
	
	IIC_SDA=1;
	AX_Delayus(1);
	
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}

/**
  * @��  ��  IIC����һ���ֽ�,���شӻ�����Ӧ��
  * @��  ��  data: Ҫ���͵�����
  * @����ֵ  ��
  */		  
void IIC_Send_Byte(uint8_t data)
{                        
    uint8_t t;   
	
	//SDA�����
	SDA_OUT();

    //����ʱ�ӿ�ʼ���ݴ���
    IIC_SCL=0;
	
    for(t=0; t<8; t++)
    {   
		//��λ�ȷ���		
        IIC_SDA=(data&0x80)>>7;
  
		AX_Delayus(1);   
		IIC_SCL=1;
		AX_Delayus(1); 
		IIC_SCL=0;	
		AX_Delayus(1);
		
	    //����1λ,������һ�η���		
		data<<=1; 
    }	 
} 	 
  
/**
  * @��  ��  IIC��ȡһ���ֽ�
  * @��  ��  ack:  ack=1ʱ������ack; ack=0ʱ������nack
  * @����ֵ  ���յ�������
  */
uint8_t IIC_Read_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
	
	//SDA����Ϊ����
	SDA_IN();
	
	//����1���ֽ�����
    for(i=0;i<8;i++ )
	{
		//��λ�����,�������յ�������λҪ����
		receive<<=1;
		
        IIC_SCL=0; 
        AX_Delayus(2);
		IIC_SCL=1;
        
        if(READ_SDA)
		{
			receive++; 
		}
		
		AX_Delayus(2); 
    }	
	
    if (ack)
    {
        IIC_Ack();  //����ACK
    }
    else
    {
        IIC_NAck();   //����NACK
    }

    return receive;
}


/**
  * @��  ��  д��ָ����ַָ����������
  * @��  ��  dev_addr���豸��ַ
  *          reg_addr���Ĵ�����ַ
  *          len�����ݳ���
  *          data����������
  * @����ֵ  ���ؽ��
  */
uint8_t IIC_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, const uint8_t *data)
{
	int i;
	
    if (!IIC_Start())
        return 1;
	
    IIC_Send_Byte(dev_addr << 1 );
	
    if (!IIC_Wait_Ack()) 
	{
        IIC_Stop();
        return 1;
    }
	
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
	
	for (i = 0; i < len; i++) 
	{
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) 
		{
            IIC_Stop();
            return 0;
        }
    }
	
    IIC_Stop();
    return 0;
}

/**
  * @��  ��  ��ȡָ����ַָ����������
  * @��  ��  dev_addr���豸��ַ
  *          reg_addr���Ĵ�����ַ
  *          len�����ݳ���
  *          data����������
  * @����ֵ  ���ؽ��
  */
uint8_t IIC_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    if (!IIC_Start())
        return 1;
	
    IIC_Send_Byte(dev_addr << 1);
    if (!IIC_Wait_Ack()) 
	{
        IIC_Stop();
        return 1;
    }
	
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
	
    IIC_Start();
    IIC_Send_Byte((dev_addr << 1)+1);
    IIC_Wait_Ack();
    while (len) 
	{
        if (len == 1)
            *data = IIC_Read_Byte(0);
        else
            *data = IIC_Read_Byte(1);
		
        data++;
        len--;
    }
	
    IIC_Stop();
    return 0;
}

/**
  * @��  ��  MPU6050д�Ĵ�����
  * @��  ��  reg_addr���Ĵ�����ַ
  *          pdata����������
  *          len�����ݳ���
  * @����ֵ  ��
  */
static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data)
{
	IIC_Write(MPU6050_ADDR,reg_address,1,&data);
}

/**
  * @��  ��  MPU6050���Ĵ�����
  * @��  ��  ��	  
  * @����ֵ  ��
  */
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len)
{
	IIC_Read(MPU6050_ADDR,reg_address,len,pdata);
}
/******************* DMP�� **************************************/
/**************************************************************************
�������ܣ�MPU6050����DMP�ĳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void DMP_Init(void)
{ 
	u8 temp[1]={0};
	IIC_Read(0x68,0x75,1,temp);
	
	//printf("mpu_set_sensor complete ......\r\n");
//	if(temp[0]!=0x68)NVIC_SystemReset();
	if(!mpu_init())
	{
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			//printf("mpu_set_sensor complete ......\r\n");
		}
		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
		{
			//printf("mpu_configure_fifo complete ......\r\n");
		}
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
		{
			//printf("mpu_set_sample_rate complete ......\r\n");
		}
		if(!dmp_load_motion_driver_firmware())
		{
			//printf("dmp_load_motion_driver_firmware complete ......\r\n");
		}
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
		{
			//printf("dmp_set_orientation complete ......\r\n");
		}
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_GYRO_CAL))
		{
			//printf("dmp_enable_feature complete ......\r\n");
		}
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
		{
			//printf("dmp_set_fifo_rate complete ......\r\n");
		}
		run_self_test();
		if(!mpu_set_dmp_state(1))
		{
			//printf("mpu_set_dmp_state complete ......\r\n");
		}
	}
}
/**************************************************************************
�������ܣ���ȡMPU6050����DMP����̬��Ϣ
��ڲ�������
����  ֵ����
**************************************************************************/
void Read_DMP(void)
{	
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		
	if (sensors & INV_WXYZ_QUAT )
	{    
		 q0=quat[0] / q30;
		 q1=quat[1] / q30;
		 q2=quat[2] / q30;
		 q3=quat[3] / q30;
		 Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	
		 Roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
		 Yaw = atan2(2 * (q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3;//yaw
	}
}
/**************************************************************************
�������ܣ���ȡ�Ƕ� 0-359
��ڲ�������
����  ֵ����
**************************************************************************/
void getAngle(float *yaw,float *yaw_acc_error)
{
	Read_DMP();                   //===��ȡYaw(-180 - 179)
	
	if(Yaw < 0)
		Yaw = Yaw + 360;
	*yaw = Yaw;                   //===ת��yaw(   0 - 359)
	
	*yaw = *yaw - *yaw_acc_error; //===��ȥyaw��ʱ�������Ư��
	
	if(*yaw < 0)
		*yaw = *yaw+360;
}

//�õ�dmp����������(ע��,��������Ҫ�Ƚ϶��ջ,�ֲ������е��)
//pitch:������ ����:0.1��   ��Χ:-90.0�� <---> +90.0��
//roll:�����  ����:0.1��   ��Χ:-180.0��<---> +180.0��
//yaw:�����   ����:0.1��   ��Χ:-180.0��<---> +180.0��
//����ֵ:0,����
//    ����,ʧ��
uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw)
{
    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
    unsigned long sensor_timestamp;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;
    /* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
     * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
    **/
    /*if (sensors & INV_XYZ_GYRO )
    send_packet(PACKET_TYPE_GYRO, gyro);
    if (sensors & INV_XYZ_ACCEL)
    send_packet(PACKET_TYPE_ACCEL, accel); */
    /* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
     * The orientation is set by the scalar passed to dmp_set_orientation during initialization.
    **/
    if(sensors&INV_WXYZ_QUAT)
    {
        q0 = quat[0] / q30;	//q30��ʽת��Ϊ������
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        //����õ�������/�����/�����
        *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
        *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
    } else return 2;
    return 0;
}
/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ������� PD7 �������� �ж������� ������ʽ �ж�ͨ��
����  ֵ���� 
**************************************************************************/
void MBOT_EXTI_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);          //�ⲿ�жϣ���Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);        //ʹ��GPIO�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	                 //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                //��������
	GPIO_Init(GPIOD, &GPIO_InitStructure);					     //�����趨������ʼ��GPIO
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource7);
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;      //�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 	                         //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		 //ʹ���ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //��ռ���ȼ�2�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		 //�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			     //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); 
}