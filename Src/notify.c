/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : notify.c
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

/* reset the tim3 channel 1 */
static void beep_simple_settings(unsigned int psr,unsigned int autoload,unsigned int pulse) 
{                                       
		/* disable TIM */                   
		TIM3->CR1 &=~ 0x01;                
		/* set psr and counter and pulse */ 
		TIM3->PSC  = psr;                   
		TIM3->ARR  = autoload;               
		TIM3->CCR1 = pulse; 
		/* enable TIM */                    
		TIM3->CR1 |= 0x01;	                
}                                       
/* Define various functional interfaces */
static int beep_ioctrl(unsigned int cmd,void * data , unsigned int len)
{
	/* parse the cmd */
	switch(cmd)
	{
		case LOWPOWER:
			 /* settings */
		   beep_simple_settings(9000,800,750);
			break;
		case THRERROR:
			 /* settings */
		   beep_simple_settings(36000,2000,1900);
			break;
		case CALIBRATE:
			 /* settings */
		   beep_simple_settings(36000,4000,3800);			
			break;
		case REV_TESTD:
			break;
		default:
			break;
	}
	/* return */
	return 0;
}
/* close the beep */
static int disable(void)
{
	/* disable TIM */                   
	TIM3->CR1 &=~ 0x01;  	
	/* return as normal */
	return 0;
}
/* beep init */
int Beep_Init(dev_HandleTypeDef * dev)
{
	/* disable the PWM first */
	disable();
	/* set functions */
	dev->ioctrl = beep_ioctrl;
	dev->disable = disable;
	/* return OK */
	return 0;
}














