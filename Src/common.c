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
	tep_HandleTypeDef * tep;
	int ret;
  unsigned int adc_cur = 0;
  unsigned int high,mid,low;	
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
		case KEY_RC:
			/* read rc key */
		  ret = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
			break;
		case TRANSFER_RC:
			/* get data */
		  if( len != sizeof(tep_HandleTypeDef))
			{
				return (-1);
			}
			/* transfer data */
			tep = (tep_HandleTypeDef *)data;
			/*---------------*/
			for( int i = 0 ; i < 4 ; i ++ )
			{
				  /* get current data */
          adc_cur = tep->raw[i];
				  /* get range */
          high = tep->fls->channel[i][0];
				  mid  = tep->fls->channel[i][1];
				  low  = tep->fls->channel[i][2];
          /* filtering maxium and minium */
          if( FABS(	high , adc_cur ) < 5 || adc_cur > high )
					{
						adc_cur = high;
					}	
					/* mid */
          if( FABS(	mid , adc_cur ) < 5 )
					{
						adc_cur = mid;
					}						
          /* low */
          if( FABS(	low , adc_cur ) < 5 || adc_cur < low )
					{
						adc_cur = low;
					}	      
          /* transfer start */
          if( tep->dir[i] )
					{
						if( adc_cur > mid )
						{
								tep->rc->channel[i] = (unsigned short)(((float)500/(float)(high-mid)) * (float)(adc_cur - mid ) + 500 );
						}
						else
						{
								tep->rc->channel[i] = (unsigned short)(((float)500/(float)(mid-low)) * (float)(adc_cur - low ));
						}							
					}else
          {
						if( adc_cur > mid )
						{
							tep->rc->channel[i] = (unsigned short)(((float)500/(float)((float)mid-(float)high)) * (float)((float)adc_cur - (float)mid ) + 500 );
						}
						else
						{
							tep->rc->channel[i] = (unsigned short)(((float)500/(float)((float)low-(float)mid)) * (float)((float)adc_cur - (float)low ) + 1000);
						}
          }	
			}
			/* get other channel */
			tep->rc->channel567 = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)<<15 | HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)<<14 |
			                      HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)<<13 | HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)<<12 ;
			/* unique id */
			tep->rc->unique_id = tep->unique_id;
			/* calcu crc */
			tep->rc->crc = tep->crc->read(0,tep->rc,sizeof(rcs_HandleTypeDef) - 2 );
      /* break */			
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
  if( (vart[0] < 10) && (vart[1] < 10) && (vart[2] < 10) && (vart[3] < 10) )
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
void common_process(unsigned int pm1,unsigned int pm2,unsigned int pm3,unsigned int pm4,unsigned int pm5)
{
	unsigned int adc_raw[5];
	static unsigned int freq_ctrl = 0;
	static unsigned char calibration_step = 0;
	static unsigned int  channel_count = 0;
	static unsigned char calibration_flag = 0;
	unsigned short calibration_tmp[4];	
	/* fls_HandleTypeDef */
	fls_HandleTypeDef * fls = ( fls_HandleTypeDef * )pm2;
	dev_HandleTypeDef * led = ( dev_HandleTypeDef * )pm3;
	dev_HandleTypeDef * fla = ( dev_HandleTypeDef * )pm4;
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
			for( int i = 0 ; i < 4 ; i ++ )
			  channel_raw[i][channel_count] = adc_raw[i];
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
				/* stabled */
			  if( calibration_flag == 1 )
				{
					calibration_flag = 0;//clear
					/* Determine whether data is available */
					if( FOUR_THAN(calibration_tmp,2000) )
					{
							/* save data */
						  for( int i = 0 ; i < 4 ; i ++ )
							   fls->channel[i][0] = calibration_tmp[i];
							/* Switching state */
							calibration_step = 1;
							/* led */		
              led->ioctrl(GF_RO,0,0,0);						
					}
				}
			  /* break */
				break;
			case 1://get low
        /* stabled */
			  if( calibration_flag == 1 )
				{
					calibration_flag = 0;//clear
					/* Determine whether data is available */
					if( FOUR_LESS(calibration_tmp,1500))
					{
							/* save data */
						  for( int i = 0 ; i < 4 ; i ++ )
							  fls->channel[i][2] = calibration_tmp[i];
							/* Switching state */
							calibration_step = 2;
							/* led */		
              led->ioctrl(GO_RF,0,0,0);						
					}
				}				
				break;
			case 2://get mid
        /* stabled */
			  if( calibration_flag == 1 )
				{
						calibration_flag = 0;//clear
						/* Determine whether data is available */
						/* save data */
					  for( int i = 0 ; i < 4 ; i ++ )
						  fls->channel[i][1] = calibration_tmp[i];
						/* Switching state */
					  for( int i = 0 ; i < 4 ; i ++ )
							calibration_tmp[i] = FABS(( fls->channel[i][0] - fls->channel[i][2] ) / 2 + fls->channel[i][2] , 
					                                fls->channel[i][1] );
					  /* Determine whether data is available */
					  if( FOUR_LESS(calibration_tmp,150))
						{
								/* write into flash memory */
								if( fla->ioctrl(WRITE_CALI,pm5,fls,sizeof(fls_HandleTypeDef)) == 0 )
								{
									/* ok */
									/* calibrate OK */
									led->ioctrl(GO_RO,0,0,0);		
									/* wait for key */
									while( common_ioctrl(KEY_CALIBRATION,0,0,0) == 0 );
									/* out */
									return;								
							}
							else
							{
								 /* error reset */
							   led->ioctrl(GT_RT,0,0,0);
							}
						}
						else
						{
						  /* error reset */
							led->ioctrl(GS_RS,0,0,0);
							/* reset */
							calibration_step = 0;
						}
				}					
				break;
			default :
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














