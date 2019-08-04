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
#include "fs_config.h"
#include "f_ops.h"
#include "string.h"
#include "gps.h"
#include "dstruct.h"
/* functions declare */
static int gps_heap_init(void);
static int gps_default_config(void);
static void gps_init_thread(void);
static void gps_read_thread(void);
static void gps_task_process( void );
/* fs inode system register */
FS_INODE_REGISTER("/GPS/",gps,gps_heap_init,0);
/* some test define */
struct file * usart0_p,* usart1_p,* usart2_p,* usart3_p;
/* gps define */
/* gps_date_flox*/
gps_data ublox_gps;
/* heap init */
static int gps_heap_init(void)
{
  /* full of zero */
	memset(&gps,0,sizeof(gps));
	/* shell base */
	gps.shell_i = shell_sched_getfiles();
	/* driver config */
	gps.config = gps_default_config;
	/* file interface */
	gps.flip.f_inode = &gps;
	gps.flip.f_path = "/gps.o";
	/* heap */
	
	/* add your own code here */
  gps.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}

static unsigned char test_dma_buf0[256];
static unsigned char test_dma_buf1[256];
static unsigned char test_dma_buf2[256];
static unsigned char test_dma_buf3[256];

/* gps_default_config gps init */
static int gps_default_config(void)
{
	/* open usart 0 , 1 , 2 , 3 */
	usart0_p = open("/UART/0",__FS_OPEN_ALWAYS);
	usart1_p = open("/UART/1",__FS_OPEN_ALWAYS);
	usart2_p = open("/UART/2",__FS_OPEN_ALWAYS);
	usart3_p = open("/UART/3",__FS_OPEN_ALWAYS);
	/* config */
	if( usart0_p == NULL || usart1_p == NULL || 
		  usart2_p == NULL || usart3_p == NULL )
	{
		/* can not find some uart */
		return FS_ERR;
	}
	/* config msg */
	uart_config_msg msg;
	/* init param */
	msg.index = 0;
	msg.rx_dma_buffer = (unsigned int)test_dma_buf0;
	msg.rx_dma_deepth = sizeof(test_dma_buf0);
	msg.baudrate = 9600;
	/* init */
	fs_ioctl(usart0_p,0,sizeof(msg),&msg);
	/* ------------------------------------------------------------------------*/
	/* init param */
	msg.index = 1;
	msg.rx_dma_buffer = (unsigned int)test_dma_buf1;
	msg.rx_dma_deepth = sizeof(test_dma_buf1);
	msg.baudrate = 115200;
	/* init */
	fs_ioctl(usart1_p,0,sizeof(msg),&msg);
	/* ------------------------------------------------------------------------*/	
	/* init param */
	msg.index = 2;
	msg.rx_dma_buffer = (unsigned int)test_dma_buf2;
	msg.rx_dma_deepth = sizeof(test_dma_buf2);
	msg.baudrate = 115200;
	/* init */
	fs_ioctl(usart2_p,0,sizeof(msg),&msg);
	/* ------------------------------------------------------------------------*/	
	/* init param */
	msg.index = 3;
	msg.rx_dma_buffer = (unsigned int)test_dma_buf3;
	msg.rx_dma_deepth = sizeof(test_dma_buf3);
	msg.baudrate = 115200;
	/* init */
	fs_ioctl(usart3_p,0,sizeof(msg),&msg);
	/* ------------------------------------------------------------------------*/	
	shell_create_dynamic("gps_init_thread",gps_init_thread,0);
	/* end of file */
	return FS_OK;
}

static unsigned char read_d0[256];
static unsigned char read_d1[256];
static unsigned char read_d2[256];
static unsigned char read_d3[256];
/* create a gps thread */
/* gps state */
volatile unsigned char gps_state_g = 1;
/* cread a read data thread */
static void gps_init_thread(void)
{
  static unsigned char init_cnt = 0;
	/* ----------- */
	if(gps_state_g)
	{
		gps_state_g = gps_configure();
		/* ok? */
		if( gps_state_g )
		{
			/* fail */
			init_cnt++;
			/* -------- */
			if( init_cnt > 10 )
			{
				shell_delete_dynamic("gps_init_thread",0xff);
			}
		}
	}
	else
	{
		// gps_usart1_init();
		shell_delete_dynamic("gps_init_thread",0xff);
		shell_create_dynamic("gps_task_process",gps_task_process,1);
	}	
}
/* ublox gps thread */
static void gps_task_process( void )
{
	/* define the buffer */
	unsigned char buf[ 2 * UBX_UART_MAX_DMA_SIZE ];
	// unsigned char buf[ UBX_USART1_MAX_DMA_SIZE ];

	unsigned int 	count = 0;
  /* memset the buf */
	memset(buf, 0, sizeof(buf));
	count= 0;
	/* read uart */
	count = fs_read(usart0_p, (char *)buf, sizeof(buf));
	/* get data? */
	if (count > 0)
	{
		/* pass received bytes to the packet decoder */
		for (int i = 0; i < count; i++)
		{
			gps_parse_char(buf[i]);
		}
	}
}

























