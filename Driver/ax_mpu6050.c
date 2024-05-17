/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @内  容  MPU6050 操作
  *
  ******************************************************************************
  * @说  明
  *
  * 1.MPU6050三轴加速度传感器、三轴陀螺仪
  * 2.IIC通信采用IO口模拟方式
  *
  ******************************************************************************
  */
#include "ax_mpu6050.h" 
#include "ax_sys.h"
#include "ax_delay.h"
//#include "inv_mpu.c"
#include "inv_mpu_dmp_motion_driver.h"
//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //输出模式

//IO操作函数	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //输入SDA 

//函数定义
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
    if (result == 0x03) {                   //返回0x03为MPU6050
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);			//读取当前陀螺仪的状态
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);			//根据读取的状态进行校准
		
        mpu_get_accel_sens(&accel_sens);	//读取当前加速度计的状态
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);			//根据读取的状态进行校准
		//printf("setting bias succesfully ......\r\n");
    }
}

/**
  * @简  述  MPU6050传感器初始化
  * @参  数  无	  
  * @返回值  无
  */
/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void IIC_Init(void)
{			
	//初始化I2C2
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	//SCL设置为推挽输出
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	//SDA设置为开漏输出
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
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
	//初始化I2C2
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	//SCL设置为推挽输出
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	//SDA设置为开漏输出
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
    IIC_SCL=1;
    IIC_SDA=1;
	
	//复位唤醒MPU6050
    MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x80);      //复位
	AX_Delayus(100000);  	//等待传感器稳定
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x00);      //解除休眠状态
	
	//配置MPU6050寄存器  
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,0x18);      //陀螺仪量程 默认2000deg/s
   MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,0x00);     //加速计量程 默认2g	
	MPU6050_WriteRegister(MPU6050_INT_EN_REG,0x00);      //关闭中断
	MPU6050_WriteRegister(MPU6050_FIFO_EN_REG,0x00);       //关闭FIFO
	
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x01);      //设置CLKSEL,PLL X轴为参考
	MPU6050_WriteRegister(MPU6050_PWR_MGMT2_REG,0x00); 	    //加速度与陀螺仪都工作	

}


/**
  * @简  述  MPU6050设置加速度量程 
  * @参  数  range： 加速度量程2g、4g、8g、16g
  *          可设置的加速度量程ACC_RANGE_2G、ACC_RANGE_4G、ACC_RANGE_8G、ACC_RANGE_16G
  * @返回值  无
  */
void AX_MPU6050_SetAccRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,range<<3);
}


/**
  * @简  述  MPU6050设置陀螺仪量程
  * @参  数  range 陀螺仪量程250°/S、500°/S、1000°/S、2000°/S
  *          可设置的陀螺仪量程GYRO_RANGE_250、GYRO_RANGE_500、GYRO_RANGE_1000、GYRO_RANGE_2000	
  * @返回值  无
  */
void AX_MPU6050_SetGyroRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,range<<3);
}

/**
  * @简  述  MPU6050设置陀螺仪采样率
  * @参  数  smplrate 陀螺仪采样率，范围10~1000Hz	  
  * @返回值  无
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
  * @简  述  MPU6050设置低通滤波器带宽
  * @参  数  bandwidth 低通滤波器带宽
  *          可设置的带宽： DLPF_ACC184_GYRO188、DLPF_ACC94_GYRO98、DLPF_ACC44_GYRO42、
  *                        DLPF_ACC21_GYRO20、DLPF_ACC10_GYRO10、DLPF_ACC5_GYRO5
  * @返回值  无
  */
void AX_MPU6050_SetDLPF(uint8_t bandwidth)
{
	MPU6050_WriteRegister(MPU6050_CFG_REG,bandwidth);
}

/**
  * @简  述  MPU6050获取传感器温度值
  * @参  数  无	  
  * @返回值  传感器温度值。
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
  * @简  述  MPU6050获取三轴加速度寄存器输出值
  * @参  数  pbuf：读取的数据缓冲区指针 
  * @返回值  无
  */
void AX_MPU6050_GetAccData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	//读取加速度数据
	MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG,buf,6);
	
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}

/**
  * @简  述  MPU6050获取三轴陀螺仪寄存器输出值
  * @参  数  pbuf：读取的数据缓冲区指针 
  * @返回值  无
  */
void AX_MPU6050_GetGyroData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	//读取陀螺仪数据
	MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG,buf,6);
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}



//--------------------------I2C 初始化操作函数-----------------------------------------


/**
  * @简  述  产生IIC起始信号
  * @参  数  无
  * @返回值  返回状态
  */
uint8_t IIC_Start(void)
{
	//SDA线输出
	SDA_OUT();     
	
	//START信号: 当SCL为高时, SDA从高变成低, 表示起始信号
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
	
	//钳住I2C总线，准备发送或接收数据 
	IIC_SCL=0;
	
	return 1;
}

/**
  * @简  述  产生IIC停止信号
  * @参  数  无
  * @返回值  无
  */	  
void IIC_Stop(void)
{
	//SDA线输出
	SDA_OUT(); 
	
	//STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号
	IIC_SCL=0;
	IIC_SDA=0;
 	AX_Delayus(1);
	
	//发送I2C总线结束信号
	IIC_SCL=1; 
	IIC_SDA=1;
	AX_Delayus(1);							   	
}

/**
  * @简  述  等待应答信号到来
  * @参  数  无
  * @返回值  0，接收应答成功  1，接收应答失败
  */
uint8_t  IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	
	//SDA设置为输入  
	SDA_IN();     
	
	//主机释放SDA线(此时外部器件可以拉低SDA线)
	IIC_SDA=1;
	AX_Delayus(1);	
	
	//SCL=1, 此时从机可以返回ACK
	IIC_SCL=1;
	AX_Delayus(1);	
	
	//等待应答
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
	
	//SCL=0, 结束ACK检查	
	IIC_SCL=0;
	AX_Delayus(1);
	
	return 1;  
} 

/**
  * @简  述  产生ACK应答产生IIC起始信号
  * @参  数  无
  * @返回值  无
  */
void IIC_Ack(void)
{
	IIC_SCL=0;
	
	//SDA线输出
	SDA_OUT();
	
	IIC_SDA=0;
	AX_Delayus(1);
	
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}

/**
  * @简  述  产生NACK应答
  * @参  数  无
  * @返回值  无
  */	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	
	//SDA线输出
	SDA_OUT();
	
	IIC_SDA=1;
	AX_Delayus(1);
	
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}

/**
  * @简  述  IIC发送一个字节,返回从机有无应答
  * @参  数  data: 要发送的数据
  * @返回值  无
  */		  
void IIC_Send_Byte(uint8_t data)
{                        
    uint8_t t;   
	
	//SDA线输出
	SDA_OUT();

    //拉低时钟开始数据传输
    IIC_SCL=0;
	
    for(t=0; t<8; t++)
    {   
		//高位先发送		
        IIC_SDA=(data&0x80)>>7;
  
		AX_Delayus(1);   
		IIC_SCL=1;
		AX_Delayus(1); 
		IIC_SCL=0;	
		AX_Delayus(1);
		
	    //左移1位,用于下一次发送		
		data<<=1; 
    }	 
} 	 
  
/**
  * @简  述  IIC读取一个字节
  * @参  数  ack:  ack=1时，发送ack; ack=0时，发送nack
  * @返回值  接收到的数据
  */
uint8_t IIC_Read_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
	
	//SDA设置为输入
	SDA_IN();
	
	//接收1个字节数据
    for(i=0;i<8;i++ )
	{
		//高位先输出,所以先收到的数据位要左移
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
        IIC_Ack();  //发送ACK
    }
    else
    {
        IIC_NAck();   //发送NACK
    }

    return receive;
}


/**
  * @简  述  写入指定地址指定长度数据
  * @参  数  dev_addr：设备地址
  *          reg_addr：寄存器地址
  *          len：数据长度
  *          data：数据内容
  * @返回值  返回结果
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
  * @简  述  读取指定地址指定长度数据
  * @参  数  dev_addr：设备地址
  *          reg_addr：寄存器地址
  *          len：数据长度
  *          data：数据内容
  * @返回值  返回结果
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
  * @简  述  MPU6050写寄存器。
  * @参  数  reg_addr：寄存器地址
  *          pdata：数据内容
  *          len：数据长度
  * @返回值  无
  */
static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data)
{
	IIC_Write(MPU6050_ADDR,reg_address,1,&data);
}

/**
  * @简  述  MPU6050读寄存器。
  * @参  数  无	  
  * @返回值  无
  */
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len)
{
	IIC_Read(MPU6050_ADDR,reg_address,len,pdata);
}
/******************* DMP库 **************************************/
/**************************************************************************
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
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
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
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
函数功能：获取角度 0-359
入口参数：无
返回  值：无
**************************************************************************/
void getAngle(float *yaw,float *yaw_acc_error)
{
	Read_DMP();                   //===读取Yaw(-180 - 179)
	
	if(Yaw < 0)
		Yaw = Yaw + 360;
	*yaw = Yaw;                   //===转换yaw(   0 - 359)
	
	*yaw = *yaw - *yaw_acc_error; //===减去yaw随时间的向上漂移
	
	if(*yaw < 0)
		*yaw = *yaw+360;
}

//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
//pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
//roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
//yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
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
        q0 = quat[0] / q30;	//q30格式转换为浮点数
        q1 = quat[1] / q30;
        q2 = quat[2] / q30;
        q3 = quat[3] / q30;
        //计算得到俯仰角/横滚角/航向角
        *pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
        *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
    } else return 2;
    return 0;
}
/**************************************************************************
函数功能：外部中断初始化
入口参数：无 PD7 上拉输入 中断线配置 触发方式 中断通道
返回  值：无 
**************************************************************************/
void MBOT_EXTI_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);          //外部中断，需要使能AFIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);        //使能GPIO端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	                 //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                //上拉输入
	GPIO_Init(GPIOD, &GPIO_InitStructure);					     //根据设定参数初始化GPIO
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource7);
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;      //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	 	                         //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;		 //使能外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //抢占优先级2， 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		 //子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			     //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 
}