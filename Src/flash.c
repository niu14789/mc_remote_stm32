/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	* BEEP TIM3 CHANNEL1 PWM Gerente
	* LED is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dev.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
static int flash_write(unsigned int addr,void * data , unsigned int len)
{
	/* extern */
	extern void FLASH_PageErase(uint32_t PageAddress);
	int ret = 0;
	unsigned short * dat = (unsigned short *)data;
	/* erase frase */
	FLASH_PageErase(addr);
	/* write into flash */
	for( int i = 0 ; i < len / 2 ; i ++ )
	{
			ret += HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,addr + i * 2 , dat[i]);
	}
	/*------*/
	return ret;
}
/* flash read into specify buffer */
static int flash_read(unsigned int addr,void * data,unsigned int len)
{
	/* judging the length */
	if( (addr + len) >= 128*1024 )//overflow
	{
		return (-1);
	}
	/* get base */
	unsigned char * base = (unsigned char *)addr;
	unsigned char * dat  = (unsigned char *)data;
	/* copy data */
	for( int i = 0 ; i < len ; i ++ )
	{
		dat[i] = base[i];
	}
	/* return ok */
	return 0;
}
/* Define various functional interfaces */
static int flash_ioctrl(unsigned int cmd,unsigned int param,void * data , unsigned int len)
{
	int ret = 0;
	dev_HandleTypeDef   * dev;
	fls_HandleTypeDef * fls;
	/* parse the cmd */
	switch(cmd)
	{
		case CALI_BUFFER:
			/* judging the length */
			if( len != sizeof(fls_HandleTypeDef) )
			{
				return (-1);/* can not */
			}
			/* get interface */
			dev = (dev_HandleTypeDef   *)param;
			fls = (fls_HandleTypeDef *)data;
			/* read flash data into fls */
			flash_read(EEBASE_ADDR,fls,sizeof(fls_HandleTypeDef));
			/* calibate crc and check */
			ret = ( dev->read(0,fls,len - 2 ) == fls->crc_check ) ? 1 : 0 ; //remove the crc16 section
			/* break */
			break;
		case 1:
			break;
		default:
			break;
	}
	/* return ret */
	return ret;
}
/* init */
int Flash_Init(dev_HandleTypeDef * dev)
{
	/* unclock flash */
	HAL_FLASH_Unlock();
	/* copy functions */
	dev->write  = flash_write;
	dev->read   = flash_read;
	dev->ioctrl = flash_ioctrl;
	/* return Ok */
	return 0;
}












