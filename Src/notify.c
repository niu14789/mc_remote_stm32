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
static int beep_ioctrl(unsigned int cmd,unsigned int param,void * data , unsigned int len)
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
		case HARDFAULT:
			/* Some hardware may be damaged */
		  beep_simple_settings(36000,4000,1000);		
			break;
		case INIT:
			/* init */
		  beep_simple_settings(9000,800,400);	
		default:
			break;
	}
	/* return */
	return 0;
}
/* close the beep */
static int disable(void)
{
	/* set psr and counter and pulse */   
	TIM3->CCR1 = 0xffff; 
	/* disable TIM */                   
	TIM3->CR1 &=~ 0x01;             	
	/* return as normal */
	return 0;
}
/* delay for a while , just for notify */
static void delay_notify(unsigned int ms)
{
	unsigned int tick;
	/* get tick */
	tick = HAL_GetTick();
	/* wait */
	while( !((HAL_GetTick() - tick) > ms) );
}
/* beep init */
int Beep_Init(dev_HandleTypeDef * dev)
{
	/* play a short music */
#if SILIENCE_DEBUG	
	beep_ioctrl(INIT,0,0,0);
#endif	
	/* hold on a while */
	delay_notify(100);
	/* disable the PWM first */
	disable();
	/* set functions */
	dev->ioctrl = beep_ioctrl;
	dev->disable = disable;
	/* return OK */
	return 0;
}
/* This is the led init blow */
/* 
reset the tim4 channel 3 & 4 
LED settings:TIM4 ch3 and TIM4 ch4
*/
static void led_simple_settings(unsigned int psr,unsigned int autoload,unsigned int pulse1,unsigned int pulse2) 
{                                       
		/* disable TIM */                   
		TIM4->CR1 &=~ 0x01;                
		/* set psr and counter and pulse */ 
		TIM4->PSC  = psr;                   
		TIM4->ARR  = autoload;               
		TIM4->CCR3 = pulse1; 
	  TIM4->CCR4 = pulse2;
		/* enable TIM */                    
		TIM4->CR1 |= 0x01;	                
} 
/* Define various functional interfaces */
static int led_ioctrl(unsigned int cmd,unsigned int param,void * data , unsigned int len)
{
	/* parse the cmd */
	switch(cmd)
	{
		case GO_RO: 
			/* green and red are on  */
			led_simple_settings(36000,2000,0,0);
			break;
		case GF_RF:
			/* green and red are off */
			led_simple_settings(36000,2000,2001,2001);
			break;
		case GO_RF: 
			/* green on . red off   */
		  led_simple_settings(36000,2000,0,2001);
			break;
		case GF_RO: 
			/* green off . red on   */
		  led_simple_settings(36000,2000,2001,0);
			break;
		/* flash */
		case GS_RO: 
			/* green slow . red on   */
		  led_simple_settings(36000,2000,1000,0);
			break;
		case GS_RF: 
			/* green slow . red off  */
			led_simple_settings(36000,2000,1000,2001);
		  break;
		case GS_RS: /* green slow . red slow */
			led_simple_settings(36000,2000,1000,1000);
			break;
		case GT_RO: 
			/* green fast . red on   */
			led_simple_settings(6000,2000,1000,0);
			break;
		case GT_RF: 
			/* green fast . red off  */
		  led_simple_settings(6000,2000,1000,2001);
			break;
		case GT_RT: 
			/* green fast . red fast */
		  led_simple_settings(6000,2000,1000,1000);
			break;
		case GO_RS: 
			/* green on   . red slow */
		  led_simple_settings(36000,2000,0,1000);
			break;
		case GF_RS: 
			led_simple_settings(36000,2000,2001,1000);
			/* green off  . red slow */
			break;
		case GO_RT: /* green on   . red fast */
			led_simple_settings(6000,2000,0,1000);
			break;
		case GF_RT: 
			/* green off  . red fast */	
		  led_simple_settings(6000,2000,2001,1000);
			break;
		default:
			break;
	}
	/* return */
	return 0;
}
/* led init */
int led_Init(dev_HandleTypeDef * dev)
{
	/* init status */
	led_ioctrl(GO_RO,0,0,0);
	/* set functions */
	dev->ioctrl = led_ioctrl;
	/* return OK */
	return 0;
}










