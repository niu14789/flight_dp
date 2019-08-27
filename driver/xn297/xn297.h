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
	* common is TIM4 CH3 and CH4
  */
/* USER CODE END Header */
#ifndef __XN297_H__
#define __XN297_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"

static int xn297_heap_init(void);
static struct file * xn297_fopen (FAR struct file *filp);
static unsigned char spi2_wr_byte( unsigned char WriteByte );
static void rf_write_reg( unsigned char reg,  unsigned char wdata);
static unsigned char rf_read_reg( unsigned char reg );
static void rf_write_multiple( unsigned char reg,const unsigned char *pBuf, unsigned char length);
static void rf_read_multiple( unsigned char reg, unsigned char *pBuf,  unsigned char length);
static int rf_init(void);
/* port init */
#define  SPI2_NSS_HIG()   (gpio_bit_set(GPIOA,GPIO_PIN_15))
#define  SPI2_NSS_LOW()   (gpio_bit_reset(GPIOA,GPIO_PIN_15))

#define  SPI_RW           spi2_wr_byte
#define  CSN_LOW          SPI2_NSS_LOW() 
#define  CSN_HIGH         SPI2_NSS_HIG()
#define  delay_ms         delay_ms(...)

#define  CE_PIN           GPIO_PIN_13
#define  CE_GPIO_PORT     GPIOC
#define  CE_GPIO_CLK      RCU_GPIOC
#define  CE_LOW           (gpio_bit_reset(CE_GPIO_PORT,CE_PIN))
#define  CE_HIGH          (gpio_bit_set(CE_GPIO_PORT,CE_PIN))

#define  IRQ_PIN          GPIO_PIN_13
#define  IRQ_GPIO_PORT    GPIOB
#define  IRQ_GPIO_CLK     RCU_GPIOB
#define  IRQ_STATUS       (gpio_input_bit_get(IRQ_GPIO_PORT,IRQ_PIN))

#define  DEFAULT_TX_ADDE {0xCC,0xCC,0xCC,0xCC,0xCC}

#define  TRANS_ENHANCE_MODE    1 
#define  TRANS_BURST_MODE      2  

#define  RF13dBm    0x3f     // 13dBm 
#define  RF10dBm    0x0f     // 10dBm 
#define  RF8dBm     0x15     // 8dbm      
#define  RF7dBm     0x07     // 7dbm   
#define  RF5dBm     0x2c     // 5dbm   
#define  RF4dBm     0x06     // 4dbm   
#define  RF2dBm     0x05     // 2dbm  
#define  RF0dBm     0x0B     // 0dBm  
#define  RF_3dBm    0x04     // -3dBm     
#define  RF_6dBm    0x0A     // -6dBm 
#define  RF_10dBm   0x02     // -10dBm 
#define  RF_18dBm   0x01     // -18dBm 
#define  RF_30dBm   0x00     // -30dBm 

/********************SPI  REGISTER  ********************/
#define		R_REGISTER			     0x00    //SPI read RF data
#define		W_REGISTER			     0x20    //SPI write RF data
#define		R_RX_PAYLOAD		     0x61    //Read RX Payload
#define		W_TX_PAYLOAD		     0xA0    //Write TX Payload
#define		FLUSH_TX			       0xE1    //Flush RX FIFO
#define		FLUSH_RX			       0xE2    //Flush TX FIFO
#define		REUSE_TX_PL			     0xE3    //Reuse TX Payload
#define		ACTIVATE			       0x50    //ACTIVATE
#define		DEACTIVATE			     0x50    //DEACTIVATE
#define		R_RX_PL_WID			     0x60    //Read width of RX data 
#define		W_ACK_PAYLOAD		     0xA8    //Data with ACK
#define		W_TX_PAYLOAD_NOACK	 0xB0    //TX Payload no ACK Request
#define		CE_FSPI_ON	         0xFD    // CE HIGH
#define		CE_FSPI_OFF	         0xFC    // CE LOW
#define		RST_FSPI	           0x53    // RESET
#define		NOP_N				         0xFF
             
/******************CONTROL  REGISTER*******************/
#define		CONFIG_XN297L   0x00            
#define		EN_AA				    0x01
#define		EN_RXADDR			  0x02
#define		SETUP_AW			  0x03
#define		SETUP_RETR			0x04
#define		RF_CH			    	0x05
#define		RF_SETUP			  0x06
#define		STATUS				  0x07
#define		OBSERVE_TX			0x08
#define		RPD			        0x09
#define		RX_ADDR_P0			0x0A
#define		RX_ADDR_P1			0x0B
#define		RX_ADDR_P2			0x0C
#define		RX_ADDR_P3			0x0D
#define		RX_ADDR_P4			0x0E
#define		RX_ADDR_P5			0x0F
#define		TX_ADDR			  	0x10
#define		RX_PW_P0		  	0x11
#define		RX_PW_P1		  	0x12
#define		RX_PW_P2			  0x13
#define		RX_PW_P3			  0x14
#define		RX_PW_P4			  0x15
#define		RX_PW_P5			  0x16
#define		FIFO_STATUS			0x17
#define		DEM_CAL				  0x19
#define   RF_CAL2				  0x1A
#define   DEM_CAL2			  0x1B
#define		DYNPD				    0x1C
#define		FEATURE				  0x1D	
#define		RF_CAL				  0x1E
#define		BB_CAL				  0x1F

/*************************CONTROL CMD****************************************/
#define  DR_1M            0X00				//1Mbps
#define  DR_2M            0X40				//2Mbps
#define  DR_250K          0XC0				//258Kbps

/******************* Function declare *******************/

#define  DEFAULT_CHANNEL      78		    //2478 MHz           
#define  PAYLOAD_WIDTH        20				// 8bytes
/* TRANS_BURST_MODE  TRANS_BURST_MODE    TRANS_ENHANCE_MODE */
#define  TRANSMIT_TYPE        TRANS_ENHANCE_MODE
/* DR_1M  2Mbps            DR_2M    DR_1M */ 
#define  DATA_RATE            DR_250K                
/* TX power 13dBm */
#define  RF_POWER             (RF0dBm |DATA_RATE)		

#define  RX_DR_FLAG           0x40       // 接收中断标志位
#define  TX_DS_FLAG           0x20       // 发送完成中断标志位
#define  RX_TX_FLAG           0x60       // 发送接收完成中断标志位，ack_payload 模式下使用
#define  MAX_RT_FLAG          0x10       // 发送重传超时中断标志位

#if(TRANSMIT_TYPE == TRANS_ENHANCE_MODE)
#define         EN_DYNPLOAD                    0
#define         EN_ACK_PAYLOAD                 1
#define         ACK_PAYLOAD_WIDTH              20
#endif

#endif

/* end of file */


















