/************************************************************
  * Copyright @shanghai.siyi
  * FileName: internal_flash.c
  * CreatDate: 2020.3.5
  * Description:
  * Version:		1
  * Subversion:	1
  * Revision:		1
  * change log:
	* 2020.3.5:first edit
  ***********************************************************/

#include "internal_flash.h"


static uint16_t FLASH_BUF[ INTERNAL_FLASH_SECTOR_SIZE / 2];

static void Flash_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
	for(uint16_t j = 0; j < NumToWrite; j ++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[j]);
		WriteAddr += 2;
	}
}


uint16_t Internal_Flash_Read_HalfWord(uint32_t Addr)
{
	return *(__IO uint16_t *)Addr;
}



void Internal_Flash_Write_Bytes( uint32_t WriteAddr, uint16_t * pBuffer, uint16_t NumToWrite)
{
	uint32_t SECTOR_Error = 0;
	uint16_t addr_in_sector;//addr in sector
	uint16_t sector_remain_size;//sector remain size  
 	uint16_t i;
	uint32_t sector_index;//sector index in flash
	uint32_t offset_addr;//offset to flash base address
	
	if(WriteAddr < FLASH_BASE || (WriteAddr >= (FLASH_BANK1_END)))
	{
		return;//error addr
	}
	
	HAL_FLASH_Unlock();
	
	
	offset_addr = WriteAddr - FLASH_BASE;
	
	sector_index = offset_addr / INTERNAL_FLASH_SECTOR_SIZE;
	
	addr_in_sector = (offset_addr % INTERNAL_FLASH_SECTOR_SIZE) / 2;
	
	sector_remain_size = INTERNAL_FLASH_SECTOR_SIZE / 2 - addr_in_sector; 
	
	if(NumToWrite <= sector_remain_size)
	{
		sector_remain_size = NumToWrite;
	}
	
	while(1) 
	{	
		Internal_Flash_Read_Bytes(sector_index * INTERNAL_FLASH_SECTOR_SIZE + FLASH_BASE,
															FLASH_BUF, INTERNAL_FLASH_SECTOR_SIZE / 2);//read out all
		for(i = 0; i < sector_remain_size; i ++)//Check
		{
			if(FLASH_BUF[addr_in_sector + i] != 0XFFFF)
			{
				break;
			}
				
		}
		if(i < sector_remain_size)
		{
      FLASH_EraseInitTypeDef EraseInitStruct;
      /* Fill EraseInit structure*/
      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress   = sector_index * INTERNAL_FLASH_SECTOR_SIZE + FLASH_BASE;
      EraseInitStruct.NbPages       = 1;
      HAL_FLASHEx_Erase(&EraseInitStruct, &SECTOR_Error);

			for(i = 0; i < sector_remain_size; i++)//copy
			{
				FLASH_BUF[i + addr_in_sector] = pBuffer[i];
			}
			Flash_Write_NoCheck(sector_index * INTERNAL_FLASH_SECTOR_SIZE+FLASH_BASE,
													FLASH_BUF, INTERNAL_FLASH_SECTOR_SIZE / 2);
		}
    else
    {
      Flash_Write_NoCheck(WriteAddr, pBuffer, sector_remain_size);
		}
		
    if(NumToWrite == sector_remain_size)
		{
			break;
		}
		else
		{ 
			//write to next sector
			sector_index ++;
			addr_in_sector = 0;
		  pBuffer += sector_remain_size;
			WriteAddr += sector_remain_size; 
		  NumToWrite -= sector_remain_size;
			if(NumToWrite > (INTERNAL_FLASH_SECTOR_SIZE / 2))
			{
				sector_remain_size = INTERNAL_FLASH_SECTOR_SIZE / 2;//write to next
			}
			else
			{
				sector_remain_size = NumToWrite;
			}
				
		}
	}
	
	HAL_FLASH_Lock();
}






void Internal_Flash_Read_Bytes( uint32_t ReadAddr, uint16_t * pBuffer, uint16_t NumToRead )
{
	uint16_t i;
	
	for(i = 0; i < NumToRead/2; i ++)//字节对齐
	{
		pBuffer[i] = Internal_Flash_Read_HalfWord(ReadAddr);
		ReadAddr += 2;
	}
}



