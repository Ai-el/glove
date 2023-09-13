/************************************************************
  * Copyright @shanghai.siyi
  * FileName: mpu6050_io.h
  * CreatDate: 2020.3.5
  * Description:
  * Version:		1
  * Subversion:	1
  * Revision:		1
  * change log:
	* 2020.3.5:first edit
  ***********************************************************/


#ifndef __MPU6050_IO_H__
#define	__MPU6050_IO_H__

#include "stm32f1xx_hal.h"
#include <inttypes.h>

/*Modify*****************************************************************/
#define MPU_INT_RCC_CLK_ENABLE			__HAL_RCC_GPIOB_CLK_ENABLE
#define MPU_INT_GPIO_PIN						GPIO_PIN_12
#define MPU_INT_GPIO								GPIOB
#define MPU_INT_EXTI_IRQHandler			EXTI15_10_IRQHandler
#define MPU_INT_EXTI_IRQn						EXTI15_10_IRQn

void EXTI15_10_IRQHandler(void);

#define SDA_PORT_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define Soft_I2C_SDA								GPIO_PIN_7
#define I2C_SDA_PORT								GPIOB
#define SCL_PORT_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define Soft_I2C_SCL								GPIO_PIN_6
#define I2C_SCL_PORT								GPIOB
/******************************************************************/
//
#define Soft_I2C_SCL_0		do{\
													HAL_GPIO_WritePin(\
															I2C_SCL_PORT,Soft_I2C_SCL,GPIO_PIN_RESET);\
													}while(0);
#define Soft_I2C_SCL_1		do{\
													HAL_GPIO_WritePin(\
															I2C_SCL_PORT,Soft_I2C_SCL,GPIO_PIN_SET);\
													}while(0);
#define Soft_I2C_SDA_0		do{\
													HAL_GPIO_WritePin(\
															I2C_SDA_PORT,Soft_I2C_SDA,GPIO_PIN_RESET);\
													}while(0);
#define Soft_I2C_SDA_1		do{\
													HAL_GPIO_WritePin(\
															I2C_SDA_PORT,Soft_I2C_SDA,GPIO_PIN_SET);\
													}while(0);

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
//	<h>MPU FUNCTION CONFIGURATION

//	<c1.5>TARGET PLATFORM on STM32
//	<i>Select Platform as STM32
#define TARGET_ON_STM32
//	</c>
													
//	<c1.5>MPU Debug ON/OFF
//  <i>Enable or Disable Debug message
#define MPU_DEBUG_ON	1
//	</c>

//	<o>Default_MPU Data Rate In HZ
//	<i>Default Data Sample Rate in HZ
//		<i>(0-100), default:100
#define DEFAULT_MPU_HZ         (100)

#ifndef M_PI
	#define M_PI (3.14159265358979)
#endif
													
#ifdef	MPU_DEBUG_ON
#define MPL_LOGI(fmt,arg...)\
do{\
	printf("<<-MPU-LOGI->> [%s] [%d]"fmt"\n",\
					__FILE__, __LINE__, ##arg);\
}while(0);

#define MPL_LOGE(fmt,arg...)\
do{\
	printf("<<-MPU-LOGE->> [%s] [%d]"fmt"\n",\
					__FILE__, __LINE__, ##arg);\
}while(0);
#endif

#if (defined __CC_ARM) && !(defined __GNUC__)
#define __no_operation()	__ASM("nop")
#endif

#if defined TARGET_ON_STM32
#define i2c_write   Sensors_I2C_WriteRegister
#define i2c_read    Sensors_I2C_ReadRegister 
#define delay_ms    mdelay
#define get_ms      get_tick_count
#define log_i       MPL_LOGI
#define log_e       MPL_LOGE
	#ifndef	min
		#define min(a,b) ((a<b)?a:b)
	#endif
#endif

// </h>

/************Configuration Wizard Finish**********************************/

void mdelay(uint32_t tick);		
int get_tick_count(unsigned long *count);





void I2C_Bus_Init(void);
void Set_I2C_Retry(unsigned short ml_sec);
unsigned short Get_I2C_Retry(void);
int Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, 
                                          unsigned short RegisterLen, unsigned char *RegisterValue);

int Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, 
                                           unsigned short RegisterLen, const unsigned char *RegisterValue);



	

#endif

/***************END OF FILE****/
