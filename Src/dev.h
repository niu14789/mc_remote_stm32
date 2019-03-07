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
#define LOWPOWER    (0)
#define THRERROR    (1)
#define CALIBRATE   (2)
#define REV_TESTD   (3)
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
	int (*ioctrl)(unsigned int cmd,void * data,unsigned len);
	/* get state */
	int (*state)(void);
}dev_HandleTypeDef;


/* some functions */
/* beep init */
int Beep_Init(dev_HandleTypeDef * dev);
/* end of files */
#endif













