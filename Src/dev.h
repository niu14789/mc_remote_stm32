/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : notify.c
  * Description        : This file provides code for the LED ctrl and beep ctrl 
  *                      and de-Initialization codes.
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
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NOTIFY_H__
#define __NOTIFY_H__
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#define SILIENCE_DEBUG         (0)
/* ADC settings */
#define SAMPLE_DEEP            (20)
#define SAMPLE_CHANNEL_COUNT   (5) 
/* flash eeprom base addr */
#define EEBASE_ADDR            (0x0800F800)  
/* for ioctrls */
#define LOWPOWER    (0)
#define THRERROR    (1)
#define CALIBRATE   (2)
#define HARDFAULT   (3)
#define INIT        (4)
/* led defines */
#define GO_RO       (0)  /* green and red are on  */
#define GF_RF       (1)  /* green and red are off */
#define GO_RF       (2)  /* green on . red off */
#define GF_RO       (3)  /* green off . red on */
/* flash */
#define GS_RO       (4)  /* green slow . red on   */
#define GS_RF       (5)  /* green slow . red off  */
#define GS_RS       (6)  /* green slow . red slow */
#define GT_RO       (7)  /* green fast . red on   */
#define GT_RF       (8)  /* green fast . red off  */
#define GT_RT       (9)  /* green fast . red fast */
#define GO_RS       (10) /* green on   . red slow */
#define GF_RS       (11) /* green off  . red slow */
#define GO_RT       (12) /* green on   . red fast */
#define GF_RT       (13) /* green off  . red fast */
/* for common */
#define UNIQUE_ID         (0)
#define KEY_CALIBRATION   (1)
/* for flash operation */
#define CALI_BUFFER (0)
/*------------*/
/* USER CODE END Includes */
/**
  * @brief  device Base Handle Structure definition
  */
typedef struct
{
	/* important interface */
	int (*write)(unsigned int addr,void * data , unsigned int len);
	int (*read)(unsigned int addr,void * data , unsigned int len);
	/* configration */
	int (*config)(unsigned addr,void *data,unsigned len);
	/* common ctrl interface */
	int (*enable)(void);
	int (*disable)(void);
	/* ioctrl */
	int (*ioctrl)(unsigned int cmd,unsigned int param,void * data,unsigned len);
	/* get state */
	int (*state)(void);
	/* process */
	void (*process)(unsigned int pm1,unsigned int pm2,unsigned int pm3,unsigned int pm4);
}dev_HandleTypeDef;
/**
  * @brief  device Base Handle Structure definition
  */
typedef struct
{
  unsigned short channel[4][3];//high,mid,low
  unsigned short unique_id;
	unsigned short rev[18];
  unsigned short crc_check;
}fls_HandleTypeDef;

/* some functions */
/* beep init */
int Beep_Init(dev_HandleTypeDef * dev);
int led_Init(dev_HandleTypeDef * dev);
int CRC16_Init(dev_HandleTypeDef * dev);
int common_Init(dev_HandleTypeDef * dev);
int Flash_Init(dev_HandleTypeDef * dev);

/* end of files */
#endif













