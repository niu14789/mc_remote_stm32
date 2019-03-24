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

#include "nrf24l01.h"

/* USER CODE END Includes */

SPI_HandleTypeDef * rf_spi_handle;
static unsigned char  tr_addr_g[5] = {INIT_ADDR};
static unsigned char  rf_ch_g = 60;
/* delay */
static void rf_delay_ms(unsigned int t)
{
  //unsigned int tmp = t;
  /* delay */
  //while(tmp--);
}
/* spi write */
unsigned char hal_spi_rw_byte(unsigned char data)
{
	unsigned char d[2];
	/* structet */
	d[0] = data;
	/* send and receive */
	HAL_SPI_TransmitReceive(rf_spi_handle,&d[0],&d[1],1,0xffff);
	/* return data */
	return d[1];
}	
/**
  * @brief :NRF24L01���Ĵ���
  * @param :
           @Addr:�Ĵ�����ַ
  * @note  :��ַ���豸����Ч
  * @retval:��ȡ������
  */
unsigned char NRF24L01_Read_Reg( unsigned char RegAddr )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );			//Ƭѡ
	
    hal_spi_rw_byte( NRF_READ_REG | RegAddr );	//������ ��ַ
    btmp = hal_spi_rw_byte( 0xFF );				//������
	
    RF24L01_SET_CS_HIGH( );			//ȡ��Ƭѡ
	
    return btmp;
}

/**
  * @brief :NRF24L01��ָ�����ȵ�����
  * @param :
  *			@reg:��ַ
  *			@pBuf:���ݴ�ŵ�ַ
  *			@len:���ݳ���
  * @note  :���ݳ��Ȳ�����255����ַ���豸����Ч
  * @retval:��ȡ״̬
  */
void NRF24L01_Read_Buf( unsigned char RegAddr, unsigned char *pBuf, unsigned char len )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );			//Ƭѡ
	
    hal_spi_rw_byte( NRF_READ_REG | RegAddr );	//������ ��ַ
    for( btmp = 0; btmp < len; btmp ++ )
    {
        *( pBuf + btmp ) = hal_spi_rw_byte( 0xFF );	//������
    }
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
}

/**
  * @brief :NRF24L01д�Ĵ���
  * @param :��
  * @note  :��ַ���豸����Ч
  * @retval:��д״̬
  */
void NRF24L01_Write_Reg( unsigned char RegAddr, unsigned char Value )
{
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    hal_spi_rw_byte( NRF_WRITE_REG | RegAddr );	//д���� ��ַ
    hal_spi_rw_byte( Value );			//д����
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
}

/**
  * @brief :NRF24L01дָ�����ȵ�����
  * @param :
  *			@reg:��ַ
  *			@pBuf:д������ݵ�ַ
  *			@len:���ݳ���
  * @note  :���ݳ��Ȳ�����255����ַ���豸����Ч
  * @retval:д״̬
  */
void NRF24L01_Write_Buf( unsigned char RegAddr, unsigned char *pBuf, unsigned char len )
{
    unsigned char i;
	
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    hal_spi_rw_byte( NRF_WRITE_REG | RegAddr );	//д���� ��ַ
    for( i = 0; i < len; i ++ )
    {
        hal_spi_rw_byte( *( pBuf + i ) );		//д����
    }
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ

}

/**
  * @brief :���TX������
  * @param :��
  * @note  :��
  * @retval:��
  */
void NRF24L01_Flush_Tx_Fifo ( void )
{
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    hal_spi_rw_byte( FLUSH_TX );	//��TX FIFO����
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
}

/**
  * @brief :���RX������
  * @param :��
  * @note  :��
  * @retval:��
  */
void NRF24L01_Flush_Rx_Fifo( void )
{
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    hal_spi_rw_byte( FLUSH_RX );	//��RX FIFO����
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
}

/**
  * @brief :����ʹ����һ������
  * @param :��
  * @note  :��
  * @retval:��
  */
void NRF24L01_Reuse_Tx_Payload( void )
{
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    hal_spi_rw_byte( REUSE_TX_PL );		//����ʹ����һ������
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
}

/**
  * @brief :NRF24L01�ղ���
  * @param :��
  * @note  :��
  * @retval:��
  */
void NRF24L01_Nop( void )
{
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    hal_spi_rw_byte( NOP );		//�ղ�������
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
}

/**
  * @brief :NRF24L01��״̬�Ĵ���
  * @param :��
  * @note  :��
  * @retval:RF24L01״̬
  */
unsigned char NRF24L01_Read_Status_Register( void )
{
    unsigned char Status;
	
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    Status = hal_spi_rw_byte( NRF_READ_REG + STATUS );	//��״̬�Ĵ���
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
	
    return Status;
}

/**
  * @brief :NRF24L01���ж�
  * @param :
           @IRQ_Source:�ж�Դ
  * @note  :��
  * @retval:�����״̬�Ĵ�����ֵ
  */
unsigned char NRF24L01_Clear_IRQ_Flag( unsigned char IRQ_Source )
{
    unsigned char btmp = 0;

    IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//�жϱ�־����
    btmp = NRF24L01_Read_Status_Register( );			//��״̬�Ĵ���
			
    RF24L01_SET_CS_LOW( );			//Ƭѡ
    hal_spi_rw_byte( NRF_WRITE_REG + STATUS );	//д״̬�Ĵ�������
    hal_spi_rw_byte( IRQ_Source | btmp );		//����Ӧ�жϱ�־
    RF24L01_SET_CS_HIGH( );			//ȡ��Ƭѡ
	
    return ( NRF24L01_Read_Status_Register( ));			//����״̬�Ĵ���״̬
}

/**
  * @brief :��RF24L01�ж�״̬
  * @param :��
  * @note  :��
  * @retval:�ж�״̬
  */
unsigned char RF24L01_Read_IRQ_Status( void )
{
    return ( NRF24L01_Read_Status_Register( ) & (( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT )));	//�����ж�״̬
}
 
 /**
  * @brief :��FIFO�����ݿ���
  * @param :��
  * @note  :��
  * @retval:���ݿ���
  */
unsigned char NRF24L01_Read_Top_Fifo_Width( void )
{
    unsigned char btmp;
	
    RF24L01_SET_CS_LOW( );		//Ƭѡ
	
    hal_spi_rw_byte( R_RX_PL_WID );	//��FIFO�����ݿ�������
    btmp = hal_spi_rw_byte( 0xFF );	//������
	
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
	
    return btmp;
}

 /**
  * @brief :�����յ�������
  * @param :��
  * @note  :��
  * @retval:
           @pRxBuf:���ݴ�ŵ�ַ�׵�ַ
  */
unsigned char NRF24L01_Read_Rx_Payload( unsigned char *pRxBuf )
{
    unsigned char Width, PipeNum;
	
    PipeNum = ( NRF24L01_Read_Reg( STATUS ) >> 1 ) & 0x07;	//������״̬
    Width = NRF24L01_Read_Top_Fifo_Width( );		//���������ݸ���

    RF24L01_SET_CS_LOW( );		//Ƭѡ
    hal_spi_rw_byte( RD_RX_PLOAD );			//����Ч��������
	
    for( PipeNum = 0; PipeNum < Width; PipeNum ++ )
    {
        *( pRxBuf + PipeNum ) = hal_spi_rw_byte( 0xFF );		//������
    }
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
    NRF24L01_Flush_Rx_Fifo( );	//���RX FIFO
	
    return Width;
}

 /**
  * @brief :�������ݣ���Ӧ��
  * @param :
  *			@pTxBuf:�������ݵ�ַ
  *			@len:����
  * @note  :һ�β�����32���ֽ�
  * @retval:��
  */
void NRF24L01_Write_Tx_Payload_Ack( unsigned char *pTxBuf, unsigned char len )
{
    unsigned char btmp;
    unsigned char length = ( len > 32 ) ? 32 : len;		//���ݳ����Լ32 ��ֻ����32��

    NRF24L01_Flush_Tx_Fifo( );		//��TX FIFO
	
    RF24L01_SET_CS_LOW( );			//Ƭѡ
    hal_spi_rw_byte( WR_TX_PLOAD );	//��������
	
    for( btmp = 0; btmp < length; btmp ++ )
    {
        hal_spi_rw_byte( *( pTxBuf + btmp ) );	//��������
    }
    RF24L01_SET_CS_HIGH( );			//ȡ��Ƭѡ
}

 /**
  * @brief :�������ݣ�����Ӧ��
  * @param :
  *			@pTxBuf:�������ݵ�ַ
  *			@len:����
  * @note  :һ�β�����32���ֽ�
  * @retval:��
  */
void NRF24L01_Write_Tx_Payload_NoAck( unsigned char *pTxBuf, unsigned char len )
{
    if( len > 32 || len == 0 )
    {
        return ;		//���ݳ��ȴ���32 ���ߵ���0 ��ִ��
    }
	
    RF24L01_SET_CS_LOW( );	//Ƭѡ
    hal_spi_rw_byte( WR_TX_PLOAD_NACK );	//��������
    while( len-- )
    {
        hal_spi_rw_byte( *pTxBuf );			//��������
		pTxBuf++;
    }
    RF24L01_SET_CS_HIGH( );		//ȡ��Ƭѡ
}

 /**
  * @brief :�ڽ���ģʽ����TX FIFOд����(��ACK)
  * @param :
  *			@pData:���ݵ�ַ
  *			@len:����
  * @note  :һ�β�����32���ֽ�
  * @retval:��
  */
void NRF24L01_Write_Tx_Payload_InAck( unsigned char *pData, unsigned char len )
{
    unsigned char btmp;
	
	len = ( len > 32 ) ? 32 : len;		//���ݳ��ȴ���32����ֻд32���ֽ�

    RF24L01_SET_CS_LOW( );			//Ƭѡ
    hal_spi_rw_byte( W_ACK_PLOAD );		//����
    for( btmp = 0; btmp < len; btmp ++ )
    {
        hal_spi_rw_byte( *( pData + btmp ) );	//д����
    }
    RF24L01_SET_CS_HIGH( );			//ȡ��Ƭѡ
}

 /**
  * @brief :���÷��͵�ַ
  * @param :
  *			@pAddr:��ַ��ŵ�ַ
  *			@len:����
  * @note  :��
  * @retval:��
  */
void NRF24L01_Set_TxAddr( unsigned char *pAddr, unsigned char len )
{
	len = ( len > 5 ) ? 5 : len;					//��ַ���ܴ���5���ֽ�
    NRF24L01_Write_Buf( TX_ADDR, pAddr, len );	//д��ַ
}

 /**
  * @brief :���ý���ͨ����ַ
  * @param :
  *			@PipeNum:ͨ��
  *			@pAddr:��ַ����ŵ�ַ
  *			@Len:����
  * @note  :ͨ��������5 ��ַ���Ȳ�����5���ֽ�
  * @retval:��
  */
void NRF24L01_Set_RxAddr( unsigned char PipeNum, unsigned char *pAddr, unsigned char Len )
{
    Len = ( Len > 5 ) ? 5 : Len;
    PipeNum = ( PipeNum > 5 ) ? 5 : PipeNum;		//ͨ��������5 ��ַ���Ȳ�����5���ֽ�

    NRF24L01_Write_Buf( RX_ADDR_P0 + PipeNum, pAddr, Len );	//д���ַ
}

 /**
  * @brief :����ͨ���ٶ�
  * @param :
  *			@Speed:�ٶ�
  * @note  :��
  * @retval:��
  */
void NRF24L01_Set_Speed( nRf24l01SpeedType Speed )
{
	unsigned char btmp = 0;
	
	btmp = NRF24L01_Read_Reg( RF_SETUP );
	btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	
	if( Speed == SPEED_250K )		//250K
	{
		btmp |= ( 1<<5 );
	}
	else if( Speed == SPEED_1M )   //1M
	{
   		btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	}
	else if( Speed == SPEED_2M )   //2M
	{
		btmp |= ( 1<<3 );
	}

	NRF24L01_Write_Reg( RF_SETUP, btmp );
}

 /**
  * @brief :���ù���
  * @param :
  *			@Speed:�ٶ�
  * @note  :��
  * @retval:��
  */
void NRF24L01_Set_Power( nRf24l01PowerType Power )
{
    unsigned char btmp;
	
	btmp = NRF24L01_Read_Reg( RF_SETUP ) & ~0x07;
    switch( Power )
    {
        case POWER_F18DBM:
            btmp |= PWR_18DB;
            break;
        case POWER_F12DBM:
            btmp |= PWR_12DB;
            break;
        case POWER_F6DBM:
            btmp |= PWR_6DB;
            break;
        case POWER_0DBM:
            btmp |= PWR_0DB;
            break;
        default:
            break;
    }
    NRF24L01_Write_Reg( RF_SETUP, btmp );
}

 /**
  * @brief :����Ƶ��
  * @param :
  *			@FreqPoint:Ƶ�����ò���
  * @note  :ֵ������127
  * @retval:��
  */
void RF24LL01_Write_Hopping_Point( unsigned char FreqPoint )
{
    NRF24L01_Write_Reg(  RF_CH, FreqPoint & 0x7F );
}

/**
  * @brief :NRF24L01���
  * @param :��
  * @note  :��
  * @retval:��
  */ 
int NRF24L01_check( void )
{
	unsigned char i;
	unsigned char buf[5]={ 0XA5, 0XA5, 0XA5, 0XA5, 0XA5 };
	unsigned char read_buf[ 5 ] = { 0 };
	unsigned char try_time = 0;
	while( 1 )
	{
		NRF24L01_Write_Buf( TX_ADDR, buf, 5 );			//д��5���ֽڵĵ�ַ
		NRF24L01_Read_Buf( TX_ADDR, read_buf, 5 );		//����д��ĵ�ַ  
		for( i = 0; i < 5; i++ )
		{
			if( buf[ i ] != read_buf[ i ] )
			{
				break;
			}	
		} 
		
		if( 5 == i )
		{
			return 0;
		}
		else
		{
		  try_time ++;
		}
		/* timeout */
		if( try_time > 5 )
		{
			return (-1);
		}
		/*----------*/
		rf_delay_ms( 2000 );
	}
}

 /**
  * @brief :����ģʽ
  * @param :
  *			@Mode:ģʽ����ģʽ�����ģʽ
  * @note  :��
  * @retval:��
  */
void RF24L01_Set_Mode( nRf24l01ModeType Mode )
{
    unsigned char controlreg = 0;
	controlreg = NRF24L01_Read_Reg( CONFIG );
	
    if( Mode == MODE_TX )       
	{
		controlreg &= ~( 1<< PRIM_RX );
	}
    else 
	{
		if( Mode == MODE_RX )  
		{ 
			controlreg |= ( 1<< PRIM_RX ); 
		}
	}

    NRF24L01_Write_Reg( CONFIG, controlreg );
}

/**
  * @brief :NRF24L01����һ������
  * @param :
  *			@txbuf:�����������׵�ַ
  *			@Length:�������ݳ���
  * @note  :��
  * @retval:
  *			MAX_TX���ﵽ����ط�����
  *			TX_OK���������
  *			0xFF:����ԭ��
  */  
unsigned char NRF24L01_TxPacket( unsigned char *txbuf, unsigned char Length ,unsigned char * addr,unsigned char rf_ch)
{
	unsigned char l_Status = 0;
	uint16_t l_MsTimes = 0;
	
	RF24L01_SET_CS_LOW( );		//Ƭѡ
	hal_spi_rw_byte( FLUSH_TX );
	RF24L01_SET_CS_HIGH( );
	
	RF24L01_SET_CE_LOW( );		
	NRF24L01_Write_Buf( WR_TX_PLOAD, txbuf, Length );	//д���ݵ�TX BUF 32�ֽ�  TX_PLOAD_WIDTH
	RF24L01_SET_CE_HIGH( );			//��������
	while( 0 != RF24L01_GET_IRQ_STATUS( ))
	{
		rf_delay_ms( 1 );
		if( 500 == l_MsTimes++ )						//500ms��û�з��ͳɹ������³�ʼ���豸
		{
      RF24L01_Init(addr,rf_ch);
			RF24L01_Set_Mode( MODE_TX );
			break;
		}
	}
	l_Status = NRF24L01_Read_Reg(STATUS);						//��״̬�Ĵ���
	NRF24L01_Write_Reg( STATUS, l_Status );						//���TX_DS��MAX_RT�жϱ�־
	
	if( l_Status & MAX_TX )	//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg( FLUSH_TX,0xff );	//���TX FIFO�Ĵ���
		return MAX_TX; 
	}
	if( l_Status & TX_OK )	//�������
	{
		return TX_OK;
	}
	
	return 0xFF;	//����ԭ����ʧ��
}

/**
  * @brief :NRF24L01��������
  * @param :
  *			@rxbuf:�������ݴ�ŵ�ַ
  * @note  :��
  * @retval:���յ������ݸ���
  */ 
unsigned char NRF24L01_RxPacket( unsigned char *rxbuf ,unsigned char *addr)
{
	unsigned char l_Status = 0, l_RxLength = 0;//, l_100MsTimes = 0;
	
	RF24L01_SET_CS_LOW( );		//Ƭѡ
	hal_spi_rw_byte( FLUSH_RX );
	RF24L01_SET_CS_HIGH( );
	
	while( 0 != RF24L01_GET_IRQ_STATUS( ))
	{
//		rf_delay_ms( 100 );
//		
//		if( 30 == l_100MsTimes++ )		//3sû���չ����ݣ����³�ʼ��ģ��
//		{
//			RF24L01_Init(addr );
//			RF24L01_Set_Mode( MODE_RX );
//			break;
//		}
	}
	
	l_Status = NRF24L01_Read_Reg( STATUS );		//��״̬�Ĵ���
	NRF24L01_Write_Reg( STATUS,l_Status );		//���жϱ�־
	if( l_Status & RX_OK)	//���յ�����
	{
		l_RxLength = NRF24L01_Read_Reg( R_RX_PL_WID );		//��ȡ���յ������ݸ���
		NRF24L01_Read_Buf( RD_RX_PLOAD,rxbuf,l_RxLength );	//���յ����� 
		NRF24L01_Write_Reg( FLUSH_RX,0xff );				//���RX FIFO
		return l_RxLength; 
	}	
	
	return 0;				//û���յ�����	
}

 /**
  * @brief :RF24L01ģ���ʼ��
  * @param :��
  * @note  :��
  * @retval:��
  */
void RF24L01_Init(unsigned char * addr,unsigned char rf_ch )
{
    /* addr */
    RF24L01_SET_CE_HIGH( );
    NRF24L01_Clear_IRQ_Flag( IRQ_ALL );
#if DYNAMIC_PACKET == 1

    NRF24L01_Write_Reg( DYNPD, ( 1 << 0 ) ); 	//��1?�������̨�1?����?��y?Y3��?��
    NRF24L01_Write_Reg( FEATRUE, 0x07 );
    NRF24L01_Read_Reg( DYNPD );
    NRF24L01_Read_Reg( FEATRUE );
	
#elif DYNAMIC_PACKET == 0
    
    L01_WriteSingleReg( L01REG_RX_PW_P0, FIXED_PACKET_LEN );	//1��?����y?Y3��?��
	
#endif	//DYNAMIC_PACKET

    NRF24L01_Write_Reg( CONFIG, /*( 1<<MASK_RX_DR ) |*/		//?����??D??
                                      ( 1 << EN_CRC ) |     //��1?��CRC 1??��??��
                                      ( 1 << PWR_UP ) );    //?a??������?
    NRF24L01_Write_Reg( EN_AA, ( 1 << ENAA_P0 ) );   		//�����̨�0��??����|��e
    NRF24L01_Write_Reg( EN_RXADDR, ( 1 << ERX_P0 ) );		//�����̨�0?����?
    NRF24L01_Write_Reg( SETUP_AW, AW_5BYTES );     			//��??��?��?�� 5??��??��
    NRF24L01_Write_Reg( SETUP_RETR, ARD_4000US |
                        ( REPEAT_CNT & 0x0F ) );         	//???��̨���y����?? 250us
    NRF24L01_Write_Reg( RF_CH, rf_ch );             			//3?��??�������̨�
    NRF24L01_Write_Reg( RF_SETUP, 0x26 );

    NRF24L01_Set_TxAddr( addr, 5 );                      //����??TX��??��
    NRF24L01_Set_RxAddr( 0, addr, 5 );                   //����??RX��??��
		
    NRF24L01_Set_Speed(SPEED_1M);
}
/* set package */
int nrf_write(unsigned int addr,void * data , unsigned int len)
{
	/* set TX mode */
  RF24L01_Set_Mode( MODE_TX );
	/* send data */
  NRF24L01_TxPacket( (unsigned char *)data , len ,tr_addr_g , rf_ch_g);
  /* return */
	return 0;
}
/* nrf read */
int nrf_read(unsigned int addr,void * data , unsigned int len)
{
	RF24L01_Set_Mode( MODE_RX );
	/* read */
	unsigned char read_len = NRF24L01_RxPacket( (unsigned char *)data ,tr_addr_g);
	/* return */
	return read_len;
}
/* delay for a while , just for notify */
static void delay_ms_rf(unsigned int ms)
{
	unsigned int tick;
	/* get tick */
	tick = HAL_GetTick();
	/* wait */
	while( !((HAL_GetTick() - tick) > ms) );
}
/* dev init */
int nrf24L01_Init( dev_HandleTypeDef * dev , void * spi_handle ,unsigned int unique_id)
{
	  /* transfer interface */
	  if( spi_handle != 0 && rf_spi_handle == 0 )
		{
			rf_spi_handle = spi_handle;
		}
		/* copy data */
		dev->write = nrf_write;
		dev->state = delay_ms_rf;
		dev->read  = nrf_read;
		/*---------*/
		if( NRF24L01_check() != 0 )
		{
			return (-1);
		}
		/* set addr num */
		if( HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7) == 1 )//no press state
		{
			tr_addr_g[0] = unique_id >> 8;
			tr_addr_g[1] = unique_id & 0xff;
			/* set tf channel */
			rf_ch_g = (tr_addr_g[0] + tr_addr_g[1]) & 0x7f;//0~6bit is the rf channel
		}
		/* init */
		RF24L01_Init(tr_addr_g,rf_ch_g);
		/* ok */
		return 0;
}






