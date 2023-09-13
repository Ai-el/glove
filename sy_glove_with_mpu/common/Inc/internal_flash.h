/************************************************************
  * Copyright @shanghai.siyi
  * FileName: internal_flash.h
  * CreatDate: 2020.3.5
  * Description:
  * Version:		1
  * Subversion:	1
  * Revision:		1
  * change log:
	* 2020.3.5:first edit
  ***********************************************************/
	
	
#ifndef __INTERNAL_FLASH_H__
#define __INTERNAL_FLASH_H__

#include "stm32f1xx_hal.h"



//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h>Internal Flash Parameters
#ifdef STM32F103xB
//		<o>INTERNAL_FLASH_SIZE
//		<i>Inrernal Flash Size in Bytes
	#define INTERNAL_FLASH_SIZE	128
#endif


#if INTERNAL_FLASH_SIZE <= 128
//		<o>INTERNAL_FLASH_SECTOR_SIZE
//		<i>Internal Flash Sector Size in Bytes
	#define INTERNAL_FLASH_SECTOR_SIZE	1024
#else
	#define INTERNAL_FLASH_SECTOR_SIZE	2048
#endif

// </h>

//		<o>FLASH_USER_START_ADDR
//		<i>Start Address of Inrernal Flash for User 
#define FLASH_USER_START_ADDR	((uint32_t) FLASH_BASE + 0x0001F800)

/************Configuration Wizard Finish**********************************/


uint16_t Internal_Flash_Read_HalfWord(uint32_t Addr);

void Internal_Flash_Write_Bytes( uint32_t WriteAddr, uint16_t * pBuffer, uint16_t NumToWrite);
void Internal_Flash_Read_Bytes( uint32_t ReadAddr, uint16_t * pBuffer, uint16_t NumToRead );



#endif

