/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : common.c
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
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	/*------------------*/
#define DEEP  (100)
/* it should in stack */
static unsigned short channel_raw[4][DEEP];
/* USER CODE END Includes */
/* Define various functional interfaces */
static int common_ioctrl(unsigned int cmd,unsigned int param,void * data , unsigned int len)
{
	unsigned int unique[3];
	dev_HandleTypeDef * dev;
	int ret;
	/* parse the cmd */
	switch(cmd)
	{
		/* some common functions */
		case UNIQUE_ID:
			/* the unique device identifier (UID based on 96 bits) */
		  HAL_GetUID(unique);
		  /* calc crc */
		  dev = (dev_HandleTypeDef *)data;
		  /* calc crc */
		  ret = dev->read(0,(void *)unique,sizeof(unique));
 		  /**/
			break;
		case KEY_CALIBRATION:
			/* red pc13 */
			ret = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
			break;
		default:
			break;
	}
	/* ret */
	return ret;
}
/* data transfer */
static int adc_simple_filter(const unsigned int * base_data,unsigned int * raw , unsigned int len)
{	
	/* Initializes */
	unsigned short max[SAMPLE_CHANNEL_COUNT];
	unsigned short min[SAMPLE_CHANNEL_COUNT];
	unsigned int   sum[SAMPLE_CHANNEL_COUNT];
	/* Initializes full of zero */
	memset((void *)&max,0,sizeof(max));
	memset((void *)&min,0xff,sizeof(min));
	memset((void *)&sum,0,sizeof(sum));
	/* get base addr */
	const unsigned int * base_addr = base_data;
	/* set a simple filter and Calculate the average of each channel */
	for( int i = 0 ; i < SAMPLE_DEEP * SAMPLE_CHANNEL_COUNT ; i ++ )
	{
		/* Get the maximum value in the data */
		if( max[i%SAMPLE_CHANNEL_COUNT] < base_addr[i] )
		{
			max[i%SAMPLE_CHANNEL_COUNT] = base_addr[i];
		}
		/* Get the maximum value in the data */
		if( min[i%SAMPLE_CHANNEL_COUNT] > base_addr[i] )
		{
			min[i%SAMPLE_CHANNEL_COUNT] = base_addr[i];
		}		
		/* Calculate the sum of data */
		sum[i%SAMPLE_CHANNEL_COUNT] += base_addr[i];
	}
	/* filter enable */
	for( int i = 0 ; i < SAMPLE_CHANNEL_COUNT ; i ++ )
	{
		sum[i] -= max[i] + min[i];
	}
	/* full buffer */
	if( len < SAMPLE_CHANNEL_COUNT )
	{
		return (-1);
	}
	/* full buffer */
	for( int i = 0 ; i < SAMPLE_CHANNEL_COUNT ; i ++ )
	{	
	   raw[i] = (unsigned int)( (float)sum[i] / (float)(SAMPLE_DEEP - 2)/* Remove the maximum and minimum */ );
	}
	/* return OK */
	return 0;
}
/* int common read */
int common_read(unsigned int addr,void * data , unsigned int len)
{
	return adc_simple_filter((const unsigned int *)addr,(unsigned int *)data,len);
}
/* check mean and varianter */
static unsigned short check_mean_var(unsigned short * mean_get)
{
  /* ss */
  float mean[4];
  float vart[4];	
  /*-------------*/
  memset(mean,0,sizeof(mean));
  memset(vart,0,sizeof(vart));
  /* check app */
  for( int i = 0 ; i < DEEP ; i ++ )
  {
    mean[0] += (float)channel_raw[0][i] / (float)DEEP;
    mean[1] += (float)channel_raw[1][i] / (float)DEEP;
    mean[2] += (float)channel_raw[2][i] / (float)DEEP;
    mean[3] += (float)channel_raw[3][i] / (float)DEEP;
  }
  /* calibrate vartai */
  for( int i = 0 ; i < DEEP ; i ++ )
  {
    vart[0] += ((float)channel_raw[0][i] - mean[0])*((float)channel_raw[0][i] - mean[0]) / (float)DEEP;
    vart[1] += ((float)channel_raw[1][i] - mean[1])*((float)channel_raw[1][i] - mean[1]) / (float)DEEP;
    vart[2] += ((float)channel_raw[2][i] - mean[2])*((float)channel_raw[2][i] - mean[2]) / (float)DEEP;
    vart[3] += ((float)channel_raw[3][i] - mean[3])*((float)channel_raw[3][i] - mean[3]) / (float)DEEP;
  }
  /* check */
  if( (vart[0] < 1) && (vart[1] < 1) && (vart[2] < 1) && (vart[3] < 1) )
  {
    /* get mean */
    mean_get[0] = (unsigned short)(mean[0]+0.5f);
    mean_get[1] = (unsigned short)(mean[1]+0.5f);
    mean_get[2] = (unsigned short)(mean[2]+0.5f);
    mean_get[3] = (unsigned short)(mean[3]+0.5f);
    /* return ok */
    return 0;//ok
  }
  else
  {
    return 1;//error
  }
}
/* process */
void common_process(unsigned int pm1,unsigned int pm2,unsigned int pm3,unsigned int pm4)
{
	unsigned int adc_raw[5];
	static unsigned int freq_ctrl = 0;
	static unsigned char calibration_step = 0;
	static unsigned int  channel_count = 0;
	static unsigned char calibration_flag = 0;
	unsigned short calibration_tmp[4];	
	/* fls_HandleTypeDef */
	fls_HandleTypeDef * fls = ( fls_HandleTypeDef * )pm2;
	/* infinite loop */
	while(1)
	{
		/* every 60000 cycles that get once */
	  if( ( freq_ctrl ++ ) > 60000 )
	  {
	 	  freq_ctrl = 0;
			/* get data */
		  common_read((unsigned int)pm1,adc_raw,sizeof(adc_raw)/sizeof(adc_raw[0]));
			/* channel_raw */
			channel_raw[0][channel_count] = adc_raw[0];
			channel_raw[1][channel_count] = adc_raw[1];
			channel_raw[2][channel_count] = adc_raw[2];
			channel_raw[3][channel_count] = adc_raw[3];
			/* get OK */
			if( ++channel_count == DEEP )
			{
				channel_count = 0;//reset
				/* check var */
				if( check_mean_var(calibration_tmp) == 0 )
				{
					/* switch the step */
					calibration_flag = 1;
				}				
			}
	  }
		/* get data */
		switch(calibration_step)
		{
			case 0://get high		
				break;
			case 1://get low
				break;
			case 2://get mid
				break;
			case 3://check 
				break;
		}
	}
}
/* beep init */
int common_Init(dev_HandleTypeDef * dev)
{
	/* settings */
	dev->ioctrl  = common_ioctrl;
	dev->read    = common_read;
	dev->process = common_process;
	/* return */
	return 0;
}














