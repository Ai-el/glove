/************************************************************
  * Copyright @shanghai.siyi
  * FileName: mpu6050_io.c
  * CreatDate: 2020.3.5
  * Description:
  * Version:		1
  * Subversion:	1
  * Revision:		1
  * change log:
	* 2020.3.5:first edit
  ***********************************************************/
#include "mpu6050_io.h"

static void Soft_I2C_Configuration(void);
static void Soft_I2C_Delay(uint32_t dly);
static uint8_t Soft_I2C_START(void);
static void Soft_I2C_STOP(void);
static void Soft_I2C_SendACK(void);
static void Soft_I2C_SendNACK(void);
static uint8_t Soft_I2C_SendByte(uint8_t anbt_i2c_data);
static uint8_t Soft_I2C_ReceiveByte_WithACK(void);
static uint8_t Soft_I2C_ReceiveByte(void);
static uint8_t Soft_DMP_I2C_Read(uint8_t soft_dev_addr, uint8_t soft_reg_addr, 
																	uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf);

static uint8_t Soft_DMP_I2C_Write(uint8_t soft_dev_addr, uint8_t soft_reg_addr,
																	uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf);


#define ST_Sensors_I2C_WriteRegister  Soft_DMP_I2C_Write
#define ST_Sensors_I2C_ReadRegister   Soft_DMP_I2C_Read
#define I2C_Direction_Transmitter      ((uint8_t)0x00)
#define I2C_Direction_Receiver         ((uint8_t)0x01)

#define Soft_I2C_SDA_STATE   	HAL_GPIO_ReadPin(I2C_SDA_PORT, Soft_I2C_SDA)
#define Soft_I2C_DELAY 				Soft_I2C_Delay(100000)
#define Soft_I2C_NOP					Soft_I2C_Delay(10) 
//
#define Soft_I2C_READY		0x00
#define Soft_I2C_BUS_BUSY	0x01	
#define Soft_I2C_BUS_ERROR	0x02
//
#define Soft_I2C_NACK	  0x00 
#define Soft_I2C_ACK		0x01

static void Soft_I2C_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SCL_PORT_GPIO_CLK_ENABLE();
	SDA_PORT_GPIO_CLK_ENABLE();
	
  GPIO_InitStruct.Pin = Soft_I2C_SCL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = Soft_I2C_SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
	
	Soft_I2C_SDA_0;
	Soft_I2C_SCL_1;
	Soft_I2C_DELAY;
	//
	//
	Soft_I2C_SCL_1; 
	Soft_I2C_SDA_1;
	Soft_I2C_DELAY; 
}

void MPU_INT_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	
  MPU_INT_RCC_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = MPU_INT_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MPU_INT_GPIO, &GPIO_InitStruct);  
  
  HAL_NVIC_SetPriority(MPU_INT_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(MPU_INT_EXTI_IRQn);
}

static void Soft_I2C_Delay(uint32_t dly) 
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

static uint8_t Soft_I2C_START(void)
{ 
	Soft_I2C_SDA_1; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	if(!Soft_I2C_SDA_STATE) return Soft_I2C_BUS_BUSY;
	//
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
  //
 	Soft_I2C_SCL_0;  
 	Soft_I2C_NOP; 
	//
 	if(Soft_I2C_SDA_STATE) return Soft_I2C_BUS_ERROR;
	//
 	return Soft_I2C_READY;
}

static void Soft_I2C_STOP(void)
{
 	Soft_I2C_SDA_0; 
 	Soft_I2C_NOP;
  // 
 	Soft_I2C_SCL_1; 
 	Soft_I2C_NOP;    
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_NOP;
}

static void Soft_I2C_SendACK(void)
{
 	Soft_I2C_SDA_0;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_1;
 	Soft_I2C_NOP;
 	Soft_I2C_SCL_0; 
 	Soft_I2C_NOP;  
}

static void Soft_I2C_SendNACK(void)
{
	Soft_I2C_SDA_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;
	Soft_I2C_SCL_0; 
	Soft_I2C_NOP;  
}

uint8_t Soft_I2C_Wait_Ack(void)
{
	uint8_t ucErrTime=0;

	Soft_I2C_SDA_1;
	Soft_I2C_NOP;	   
	Soft_I2C_SCL_1;
	Soft_I2C_NOP;	 
	
	while(Soft_I2C_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Soft_I2C_STOP();
			return Soft_I2C_BUS_ERROR;
		}
	}
	Soft_I2C_SCL_0;// ±÷” ‰≥ˆ0 	   
	return 0;  
} 

static uint8_t Soft_I2C_SendByte(uint8_t soft_i2c_data)
{
 	uint8_t i;
 	
	Soft_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{
    if(soft_i2c_data&0x80)
		{
			Soft_I2C_SDA_1;
		}
    else
		{
			Soft_I2C_SDA_0;
		}
    //
    soft_i2c_data<<=1;
    Soft_I2C_NOP;
    //
    Soft_I2C_SCL_1;
    Soft_I2C_NOP;
    Soft_I2C_SCL_0;
    Soft_I2C_NOP; 
 	}
	return Soft_I2C_Wait_Ack();  
}

static uint8_t Soft_I2C_ReceiveByte(void)
{
	uint8_t i,soft_i2c_data;
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_SCL_0; 
 	soft_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
    Soft_I2C_SCL_1;
    Soft_I2C_NOP; 
    soft_i2c_data<<=1;
    //
    if(Soft_I2C_SDA_STATE)	soft_i2c_data|=0x01; 

    Soft_I2C_SCL_0;  
    Soft_I2C_NOP;         
 	}
	Soft_I2C_SendNACK();
 	return soft_i2c_data;
}

static uint8_t Soft_I2C_ReceiveByte_WithACK(void)
{
	uint8_t i,soft_i2c_data;
	//
 	Soft_I2C_SDA_1;
 	Soft_I2C_SCL_0; 
 	soft_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
    Soft_I2C_SCL_1;
    Soft_I2C_NOP; 
    soft_i2c_data<<=1;
    //
    if(Soft_I2C_SDA_STATE)	soft_i2c_data|=0x01; 

    Soft_I2C_SCL_0;  
    Soft_I2C_NOP;         
 	}
	Soft_I2C_SendACK();
 	return soft_i2c_data;
}

static uint8_t Soft_DMP_I2C_Write(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf)
{		
		uint8_t i, result=0;
		Soft_I2C_START();
		result  = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Transmitter);	
		if(result != 0) return result;
	
		result = Soft_I2C_SendByte(soft_reg_addr);  
		if(result != 0) return result;
	
		for (i=0;i<soft_i2c_len;i++) 
		{
			result = Soft_I2C_SendByte(soft_i2c_data_buf[i]); 
			if (result != 0) return result;
		}
		Soft_I2C_STOP();
		return 0x00;
}

static uint8_t Soft_DMP_I2C_Read(uint8_t soft_dev_addr, uint8_t soft_reg_addr, uint8_t soft_i2c_len,unsigned char *soft_i2c_data_buf)
{
		uint8_t result;
	
		Soft_I2C_START();
		result  = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Transmitter);			
		if(result != 0) return result;

		result = Soft_I2C_SendByte(soft_reg_addr); 
		if(result != 0) return result;

		Soft_I2C_START();
		result = Soft_I2C_SendByte(soft_dev_addr << 1 | I2C_Direction_Receiver);
		if(result != 0) return result;
	
		//
    while (soft_i2c_len)
		{
			if (soft_i2c_len==1) *soft_i2c_data_buf =Soft_I2C_ReceiveByte();  
      else *soft_i2c_data_buf =Soft_I2C_ReceiveByte_WithACK();
      soft_i2c_data_buf++;
      soft_i2c_len--;
    }
		Soft_I2C_STOP();
    return 0x00;
}


void mdelay(uint32_t tick)
{
	HAL_Delay(tick);
}

int get_tick_count(unsigned long *count)
{
  *count = HAL_GetTick();
	return 0;
}

static unsigned short RETRY_IN_MLSEC  = 55;

void Set_I2C_Retry(unsigned short ml_sec)
{
  RETRY_IN_MLSEC = ml_sec;
}

unsigned short Get_I2C_Retry(void)
{
  return RETRY_IN_MLSEC;
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len,
                                       unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
  ret = 0;
  ret = ST_Sensors_I2C_ReadRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    HAL_Delay(retry_in_mlsec);
    goto tryReadAgain;
  }
  return ret;
}

int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len,
                                        const unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
  ret = 0;
  ret = ST_Sensors_I2C_WriteRegister( slave_addr, reg_addr, len, ( unsigned char *)data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;

    HAL_Delay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  return ret;
}

void I2C_Bus_Init(void)
{	
	Set_I2C_Retry(5);
	
	Soft_I2C_Configuration();
	MPU_INT_GPIO_Init();
}

void show_debug_msg(void)
{
	log_i("SiYi Data Glove with IMU Project.\n");
	log_i("Target Build at :%s\n", __DATE__);
	log_i("Software Version: %d.%d\n", __VERSION, __SUB_VERSION);
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(MPU_INT_GPIO_PIN);
}




/******************END OF FILE****/
