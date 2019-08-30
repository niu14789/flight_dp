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

/* Includes ------------------------------------------------------------------*/

#include "fs.h"
#include "f_shell.h"
#include "fs_config.h"
#include "string.h"
#include "wifi.h"
#include "f_ops.h"
#include "f_drv.h"
#include "state.h"
#include "wifi_link.h"
/* file interface */
struct file * wifi_usart3 = NULL;
/* wifi reciver type */
/* gps define */
static unsigned char wifi_dma_cache[1024];
/* define the inode */
FS_INODE_REGISTER("wifi.d",wifi,wifi_heap_init,0);
/* heap init */
static int wifi_heap_init(void)
{
	 /* full of zero */
	memset(&wifi,0,sizeof(wifi));
	/* shell base */
	wifi.shell_i = shell_sched_getfiles();
	/* driver config */
	wifi.config = wifi_default_config;
	/* file interface */
	wifi.flip.f_inode = &wifi;
	wifi.flip.f_path = "wifi.d";
	/* file interface */
	wifi.ops.open = wifi_fopen;
	wifi.ops.ioctl = wifi_ioctrl;
	/* heap */

	/* add your own code here */
    wifi.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT|__FS_IS_INODE_NODATA; 
	/* ------end of file------ */
	return FS_OK;
}
/* fdg */
static int wifi_default_config(void)
{
	/* open usart 3 */
	wifi_usart3 = open("/UART/3",__FS_OPEN_ALWAYS);
	/* config */
	if( wifi_usart3 == NULL )
	{
		/* can not find some uart */
		return FS_ERR;
	}
	/* config msg */
	uart_config_msg msg;
	/* init param */
	msg.mode = 0;//none 8 1
	msg.index = 3;
	msg.rx_dma_buffer = (unsigned int)wifi_dma_cache;
	msg.rx_dma_deepth = sizeof(wifi_dma_cache);
	msg.baudrate = 115200;
	msg.tx_mode = _UART_TX_DMA;
	msg.rx_mode = _UART_RX_IT;
	/* init */
	fs_ioctl(wifi_usart3,0,sizeof(msg),&msg);	
	/* create a thread here */
    
    wifi_link_init();
    
	shell_create_dynamic("wifi_send_thread",wifi_send_thread,1);//4ms
	shell_create_dynamic("wifi_receive_thread",wifi_receive_thread,1);//4ms
	/*return*/
	return FS_OK;
}
/* file interface */
/* file & driver 's interface */
static struct file * wifi_fopen (FAR struct file *filp)
{
	return &wifi.flip;
}
/* ioctrl for control sbus */
static int wifi_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* write */
		  /* end of cmd */
	 	  break;
		case 1:
			/* decode */
		  /* end of cmd */
		  break;
		case 2:
			/* copy data */
		  /* end of cmd */
		  break;	
		default :
      break;
	}
	/* return OK */
	return ret;
}
/* send and rev thread */
static void wifi_send_thread(void)
{
	wifi_link_data_send();
}
/* wifi rev thread */
static void wifi_receive_thread(void)
{
	wifi_link_data_receive();
}
/* wifi send bytes */
void wifi_send_bytes(const void * data,unsigned int len)
{
	/* send to usart3 */
	fs_write(wifi_usart3,data,len);
}
/* wifi get bytes */
int wifi_receive_bytes(void * data,unsigned int len)
{
	return fs_read(wifi_usart3,data,len);
}



















