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
#include "fs.h"
#include "f_shell.h"
#include "xn297.h"
#include "state.h"
#include "gd32f30x.h"
#include "string.h"
/* some defines */
link_stru s_rf_link;

int config_default(void);

/* USER CODE BEGIN Includes */
FS_INODE_REGISTER("xn297.d",xn297,xn297_heap_init,0);
/* defined functions */
static int xn297_heap_init(void)
{
	/* init as default */
	xn297.flip.f_inode = &xn297;
	xn297.config = config_default;
	/* file interface  */
	xn297.ops.open  = xn297_fopen;
#if 0
	xn297.ops.read  = xn297_fread;
	xn297.ops.ioctl = xn297_ioctrl;
#endif
	/* init flags */
  xn297.flip.f_oflags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return xn297 result */
	return rf_init();
}

int rf_binding(void);

int config_default(void)
{
	 if( rf_binding() == FS_OK )
	 {
		/* delete init thread and create a read data thread */
		shell_create_dynamic("rf_link_timer",rf_link_timer,0);	//1ms
  	shell_create_dynamic("rf_link_function",rf_link_function,0);	//4ms 
	 }
}


/* file & driver 's interface */
static struct file * xn297_fopen (FAR struct file *filp)
{
	/* return flip data */
	return &xn297.flip;
}
/* static hardware init */
static void xn297_basic_init(void)
{
	/* spi param handle */
	spi_parameter_struct Spiparam;
	/* enable clock first */
	rcu_periph_clock_enable(RCU_SPI2);
	rcu_periph_clock_enable(RCU_AF);
	/* JTAG-DP disabled and SW-DP enabled, so SPI0 can use PB3 and PB4 */
	gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP,ENABLE);
	/* config gpio */
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);     // CE
	gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_13);        // IRQ
  /* 2.4G POWER EN */
  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
	/* spi io config */
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);     // CS
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);       // SCK
	gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4);         // MISO
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);       // MOSI
	/* SPI2 parameter config */
	Spiparam.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
	Spiparam.device_mode = SPI_MASTER;
	Spiparam.frame_size = SPI_FRAMESIZE_8BIT;
	Spiparam.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	Spiparam.nss = SPI_NSS_SOFT;
	Spiparam.prescale = SPI_PSC_16; // 60MHz /32 = 1.875MHz
	Spiparam.endian = SPI_ENDIAN_MSB;
	/* load param */
	spi_init(SPI2, &Spiparam);
	/* enable SPI2 */
	spi_enable(SPI2);
	/* enable nssp */
	spi_nssp_mode_enable(SPI2);
	/* nss output enable */
	spi_nss_output_enable(SPI2);
	/* release */
	gpio_bit_set(GPIOA,GPIO_PIN_15);	
	/* enable 2.4G power */
	gpio_bit_set(GPIOC,GPIO_PIN_2);	
	/* end of func */
}
/* spi2 write and read a byte */
static unsigned char spi2_wr_byte( unsigned char WriteByte )
{
	/* wait until spi is idle */
	while (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_TBE))
	{ 
		/* nothing need to do , just waitting */
	}
	/* send a data */
	spi_i2s_data_transmit(SPI2, WriteByte);
	/* wait until spi is idle */
	while (RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE))
	{ 
		/* nothing need to do , just waitting */
	}
	/* read a data */
	unsigned char rd = spi_i2s_data_receive(SPI2);
	/* reutnr */
	return rd;
}
/* static rf write reg */
static void rf_write_reg( unsigned char reg,  unsigned char wdata)
{
	/* enable spi dev */
	CSN_LOW;
	/* write spi reg */
	SPI_RW(reg);
	/* write spi data */
	SPI_RW(wdata);
	/* disable spi dev */
	CSN_HIGH;
	/* end of data */
}
/* rf read reg */
static unsigned char rf_read_reg( unsigned char reg )
{
	/* definf*/
	unsigned char tmp;
	/* enable spi dev */  
	CSN_LOW;
	/* write spi reg */
	SPI_RW(reg);
	/* read spi */
	tmp = SPI_RW(0);
	/* disable spi dev */
	CSN_HIGH;
  /* return */
	return tmp;
}
/* spi rf write buf */
void rf_write_multiple( unsigned char reg, const unsigned char *pBuf, unsigned char length)
{
	/* enable spi dev */ 
	CSN_LOW;    
	/* write spi reg */
	SPI_RW(reg);
	/* write */
	for( int i = 0 ; i < length ; i++)
	{
		/* write */
		SPI_RW(pBuf[i]);
	}   
	/* disable spi dev */
	CSN_HIGH;
	/* end of func */
}
/* static spi read buf */
void rf_read_multiple( unsigned char reg, unsigned char *pBuf,  unsigned char length)
{
	/* enable spi dev */ 
	CSN_LOW;    
	/* write spi reg */
	SPI_RW(reg);
	/* read */
	for( int i = 0 ; i < length ; i++)
	{
		/* read */
		pBuf[i] = SPI_RW(0xff);
	}   
	/* disable spi dev */
	CSN_HIGH;
	/* end of func */                                                                 		
}
/* set to tx mode */
void rf_set_tx(void)
{
	/* chip enable */
	CE_LOW;
	/* write reg */
	rf_write_reg(W_REGISTER + CONFIG_XN297L , 0X8E);
	/* chip disable */
	CE_HIGH;
	/* end of func */
}
/* set to rx mode */
void rf_set_rx(void)
{
	/* chip enable */
	CE_LOW;
	/* write reg */
	rf_write_reg(W_REGISTER + CONFIG_XN297L,  0X8F );						
	/* chip disable */
	CE_HIGH;
	/* end of func */
}
/* get rf status */
unsigned char rf_get_statue(void)
{
	return rf_read_reg(STATUS)&0x70;
}
/* clear tf status */
void rf_clear_status(void)
{
	rf_write_reg(W_REGISTER + STATUS,0x70);
}
/* clear rf fifo */
void rf_clear_fifo(void)
{
	rf_write_reg(FLUSH_TX, 0);
	rf_write_reg(FLUSH_RX, 0);
}
/* rf_set channel */
void rf_set_channel( unsigned char Channel)
{    
	//CE_LOW;
	rf_write_reg(W_REGISTER + RF_CH, Channel);
}
/* set address */
int rf_write_address(const unsigned char *pbuf)
{
	/* declares */
	unsigned char data[5] = {0};
	const unsigned char tx_addr_def[] = DEFAULT_TX_ADDE;
  /* chip enale */
	CE_LOW;
	/* write buf */
	rf_write_multiple( W_REGISTER + TX_ADDR , pbuf , sizeof(tx_addr_def));	
	/* read data */
	rf_write_multiple( W_REGISTER + RX_ADDR_P0 , pbuf , sizeof(tx_addr_def));	
	/* check */
	rf_read_multiple( R_REGISTER  + RX_ADDR_P0 , data , sizeof(tx_addr_def));
	/* check */
	return memcmp(tx_addr_def,data,sizeof(tx_addr_def)) ? FS_ERR : FS_OK;	
	/* end of file */
}
/* rf_set_power */
static void rf_set_power(unsigned char rf_power)
{    
	rf_write_reg(W_REGISTER + RF_SETUP,  rf_power);
}
/* send data */
static unsigned char rf_tx( unsigned char *ucPayload,  unsigned char length)
{
	/* decleares */
	unsigned char Status_Temp;
	/* write data */
	rf_write_multiple(W_TX_PAYLOAD, ucPayload, length);     
	/* wait until irq if low . this maybe case some error */
	while(IRQ_STATUS); 
	/* read tx status */
	Status_Temp = rf_read_reg(STATUS) & 0x70; 
	/* clear statue ans fifo */
	rf_write_reg(W_REGISTER + STATUS, Status_Temp);
	/* clear statue ans fifo */
	rf_write_reg(FLUSH_TX,0); 
	/* return */
	return Status_Temp;
}
/* rf_tx_transmintdata */
void rf_tx_transmintdata( unsigned char *ucTXPayload,  unsigned char length)
{    
	/* write data to txfifo */
	rf_write_multiple(W_ACK_PAYLOAD, ucTXPayload, length);
	/*	rf entery tx mode start send data */
	CE_HIGH;    
	/* end of func */
}
/* read data from fifo */
static unsigned char rf_dump_rxd( unsigned char *ucPayload,  unsigned char length)
{
	/* check irq */
	if( IRQ_STATUS )
	{
		/* didn't get any data . return */
		return FS_OK;
	}
	/* looks like that we've got some data */
	CE_LOW;
	/* read data and clear fifo status */
	rf_read_multiple(R_RX_PAYLOAD, ucPayload, length);                                	
	rf_write_reg(FLUSH_RX, 0);	
	rf_write_reg(W_REGISTER + STATUS, 0x70);                      
  /* chip enable */	
	CE_HIGH;                                                                    		
  /* return length */
	return length;
}
/* static */
static int rf_init(void)
{
	unsigned char feature = 0x00;
	const unsigned char tx_addr_def[] = DEFAULT_TX_ADDE;
#if(DATA_RATE == DR_1M)   
	unsigned char  BB_cal_data[]    = {0x0A,0x6D,0x67,0x9C,0x46};                               //1M速率配置
	//    unsigned char  BB_cal_data[]    = {0x2A,0xEC,0x6F,0x9C,0x46};                               //1M速率配置
	unsigned char  RF_cal_data[]    = {0xF6,0x37,0x5D};
	unsigned char  RF_cal2_data[]   = {0x45,0x21,0xef,0x2C,0x5A,0x50};
	unsigned char  Dem_cal_data[]   = {0x01};  
	unsigned char  Dem_cal2_data[]  = {0x0b,0xDF,0x02};  
#elif(DATA_RATE == DR_250K)
	unsigned char   BB_cal_data[]    = { 0x12,0xec,0x6f,0xa1,0x46};                          //250K速率配置
	unsigned char    RF_cal_data[]    = {0xF6,0x37,0x5C};
	unsigned char   RF_cal2_data[]   = {0xd5,0x21,0xeb,0x2c,0x5a,0x40};
	unsigned char    Dem_cal_data[]   = {0x1F};  
	unsigned char    Dem_cal2_data[]  = {0x0b,0xdf,0x02};
#endif
	/* basic hardware init */
	xn297_basic_init();
	/* enable chip */
	CE_LOW;
	/* Software Reset */
	rf_write_reg(RST_FSPI, 0x5A);											
	rf_write_reg(RST_FSPI, 0XA5);
  /* CLEAR TXFIFO */
	rf_write_reg(FLUSH_TX, 0);	    			 
	/* CLEAR  RXFIFO */
	rf_write_reg(FLUSH_RX, 0);
	/* CLEAR  STATUS */
	rf_write_reg(W_REGISTER + STATUS, 0x70);							
	/* Enable Pipe0 */
	rf_write_reg(W_REGISTER + EN_RXADDR, 0x01);							
	/* address witdth is 5 bytes */
	rf_write_reg(W_REGISTER + SETUP_AW,  0x03);							
	/* 2478M HZ */
	rf_write_reg(W_REGISTER + RF_CH,    DEFAULT_CHANNEL);           
  /* 8 bytes */	
	rf_write_reg(W_REGISTER + RX_PW_P0,  PAYLOAD_WIDTH);						
	/* Writes TX_Address to PN006 */
	rf_write_multiple(W_REGISTER + TX_ADDR,   ( unsigned char*)tx_addr_def, sizeof(tx_addr_def));	
	/* RX_Addr0 same as TX_Adr for Auto.Ack */
	rf_write_multiple(W_REGISTER + RX_ADDR_P0,( unsigned char*)tx_addr_def, sizeof(tx_addr_def));
	/* write multiple */
	rf_write_multiple(W_REGISTER + BB_CAL,    BB_cal_data,  sizeof(BB_cal_data));
	rf_write_multiple(W_REGISTER + RF_CAL2,   RF_cal2_data, sizeof(RF_cal2_data));
	rf_write_multiple(W_REGISTER + DEM_CAL,   Dem_cal_data, sizeof(Dem_cal_data));
	rf_write_multiple(W_REGISTER + RF_CAL,    RF_cal_data,  sizeof(RF_cal_data));
	rf_write_multiple(W_REGISTER + DEM_CAL2,  Dem_cal2_data,sizeof(Dem_cal2_data));
	rf_write_reg(W_REGISTER + DYNPD, 0x00);					
	/* DBM */
	rf_write_reg(W_REGISTER + RF_SETUP,  RF_POWER);
	rf_write_reg(ACTIVATE, 0x73);
  /* transmit type */
#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)      
	rf_write_reg(W_REGISTER + SETUP_RETR,0x01);							//  3 retrans... 	
	rf_write_reg(W_REGISTER + EN_AA,     0x01);							// Enable Auto.Ack:Pipe0  	
#elif(TRANSMIT_TYPE == TRANS_BURST_MODE)                                                                
	rf_write_reg(W_REGISTER + SETUP_RETR,0x00);							// Disable retrans... 	
	rf_write_reg(W_REGISTER + EN_AA,     0x00);							// Disable AutoAck 
#endif
  /* payload width */
	if( PAYLOAD_WIDTH < 33 )											
	{
		feature =  0x40;
	}
	else
	{
		feature =  0x58;
	}
  /* en dynpload */
#if(EN_DYNPLOAD == 1 )
	feature |= 0x04;
	rf_write_reg(W_REGISTER + DYNPD, 0x01);
#endif
  /* en ack payload */
#if(EN_ACK_PAYLOAD == 1 )
	feature |= 0x02;
#endif  
  /* reset */
	rf_write_reg(W_REGISTER + FEATURE, feature);
  /* set low */
	CE_LOW;
  /* set addr and check */
	return rf_write_address(tx_addr_def);
}
/* void rf binding delay */
static void rf_binding_delay(unsigned int t)
{
	while(t--);
}
/* rf check sum */
unsigned char rf_checksum(unsigned char *pbuf,unsigned char len)
{
	/* check sum */
	unsigned char checksum = 0;
	/* data */
	for( int i = 0 ; i < len ; i ++ )
	{
			checksum ^= pbuf[i];        
	}
	/* return */
	return checksum;
}
/* */
static int rf_receive_data(unsigned char *pbuf,unsigned char len)
{
	/* read data */
	rf_read_multiple(R_RX_PAYLOAD,pbuf, len); 
	/* check data */
	return (rf_checksum( pbuf ,len - 1 ) == pbuf[len - 1]) ? FS_OK : FS_ERR;
	/* end of func */
}
/* rf binding thread */
static int rf_binding(void)
{    
	/* some neccery define */
	unsigned char   binding_step = 0;
	unsigned char   binding_success_flag = 0;    
	unsigned char   u8_temp;
	unsigned int    exit_binding_counter = 0;
	/* default addr */
	const unsigned char tx_addr_def[] = DEFAULT_TX_ADDE;
	/* create a random data as a seek*/
	unsigned short addr_create = rand();
	/* set rf to default freq and channel */
	rf_write_address((unsigned char *)tx_addr_def);
	/* default channel */
	rf_set_channel(RF_BINDONG_CH);
	/* set the rf new address */
	s_rf_link.rf_new_address[2] = addr_create >> 8;
	s_rf_link.rf_new_address[3] = addr_create & 0xff; 
	/* set to rx mode */
	rf_set_rx();	
	/* initloop */
	while( binding_success_flag == 0 )
	{       
		/* default cmd case */
		switch (binding_step)
		{        
			/* first get rf data */					
			case 0:   
				/* read statue */        
				u8_temp = rf_get_statue(); 
				/* got some data or not */
				if( u8_temp & RX_DR_FLAG )		                                           
				{  
					/* read data */
					rf_read_multiple(R_RX_PAYLOAD,s_rf_link.rx_buffer, PAYLOAD_WIDTH);            
					/* copy data */
					memcpy(&s_rf_link.tx_buffer[1],&s_rf_link.rf_new_address[0],5);
          /* create buffer */
					s_rf_link.tx_buffer[0] = RF_LINK_BUFFER_HEAD;
					s_rf_link.tx_buffer[6] = binding_step | 0x10;
					/* calibate the txbuffer sum */
					s_rf_link.tx_buffer[ PAYLOAD_WIDTH -1 ] = rf_checksum(s_rf_link.tx_buffer,PAYLOAD_WIDTH-1); 
					/* send the data */
					rf_tx_transmintdata(s_rf_link.tx_buffer,PAYLOAD_WIDTH);
					/* receive ok ? */
					if( s_rf_link.rx_buffer[6] == 2 )
					{
						binding_step ++; 
					}          					
				}
				/* delay some ms to wait until send ok */	
				rf_binding_delay(10*8000);
				/* clear rf flag and fifo buffer */
				rf_clear_fifo();
				rf_clear_status();				
				/* end of func */
				break;
				/* end of func */
			case 1: 
			case 2:
				/* read statue */        
				u8_temp = rf_get_statue(); 
				/* got some data or not */
				if( u8_temp & RX_DR_FLAG )		                                           
				{ 
					/* clear exit counter */
					exit_binding_counter = 0;
					/* get data */
					if( rf_receive_data( s_rf_link.rx_buffer , PAYLOAD_WIDTH ) == FS_OK )
					{                                               
						/* save new addr */
						memcpy(&s_rf_link.rf_new_address[0],&s_rf_link.rx_buffer[1],5);
						/* save new version */
						memcpy(&s_rf_link.remoter_version[0],&s_rf_link.rx_buffer[10],3);
						/* copy data */
						memcpy(&s_rf_link.tx_buffer[1],&s_rf_link.rf_new_address[0],5);
						/* create data buffer */                               
						s_rf_link.tx_buffer[0] = RF_LINK_BUFFER_HEAD;
						s_rf_link.tx_buffer[6] = binding_step | 0x10; 
						/* calibate the txbuffer sum */
						s_rf_link.tx_buffer[PAYLOAD_WIDTH-1] = rf_checksum(s_rf_link.tx_buffer,PAYLOAD_WIDTH-1); 
						/* send the data */
						rf_tx_transmintdata(s_rf_link.tx_buffer,PAYLOAD_WIDTH);
						/* delay some ms to wait until send ok */		
						if( binding_step < 2 )
						{
							binding_step++; 
						}
					} 
					/* delay some ms to wait until send ok */
					rf_binding_delay(10*8000);
					/* clear rf flag and fifo buffer */
					rf_clear_fifo();
					rf_clear_status ();					
				}      
				else
				{
					/* can not read data */
					if( binding_step == 2 )
					{   
            /* over time */						
						if( exit_binding_counter++ > 2000 )
						{
							/* init again */
							rf_init();
							/* set the rf to new addr */
							rf_write_address(s_rf_link.rf_new_address);
							/* into rx mode */
							rf_set_rx();
							/* out */
							binding_success_flag = 1;				
							/* end of cmd */
						}                                
					}
				}
				break;
			default:
				break;            
		}            
	} 
	/* bindling ok */
	if( binding_success_flag == 1 )
	{
//			//rf_id_store();
//			rf_write_address(s_rf_link.rf_new_address);
		return FS_OK;
	}
	/* error */
  return FS_ERR;
}
















