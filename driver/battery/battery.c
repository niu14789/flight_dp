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
#include "battery.h"
#include "f_ops.h"
#include "f_drv.h"
#include "state.h"
#include "bat_dec.h"
/* file interface */
struct file * bat_usart2 = NULL;
/* bat reciver type */
/* gps define */
static unsigned char bat_dma_cache[128];
/* key */
const unsigned char Mkey[8]={0x23,0x55,0xaa,0xfa,0xd7,0x36,0xae,0xc5};
const unsigned char Skey[8]={0x58,0x0a,0x6d,0x4e,0x93,0x66,0xef,0x18};
/* unknow array */
unsigned char g_abyValidKey[8] = {0};
/* start code */
const unsigned char byDataBuff[8]={0xaa,0xbb,0x00,0x01,0x00,0x01,0xcc,0xdd};
/* bat data */
static battery_msg_def battery_msg;
/* define the inode */
FS_INODE_REGISTER("bat.d",bat,bat_heap_init,0);
/* heap init */
static int bat_heap_init(void)
{
	 /* full of zero */
	memset(&bat,0,sizeof(bat));
	/* shell base */
	bat.shell_i = shell_sched_getfiles();
	/* driver config */
	bat.config = bat_default_config;
	/* file interface */
	bat.flip.f_inode = &bat;
	bat.flip.f_path = "bat.d";
	/* file interface */
	bat.ops.open = bat_fopen;
	bat.ops.ioctl = bat_ioctrl;
	/* heap */

	/* add your own code here */
  bat.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT|__FS_IS_INODE_NODATA;
	/* ------end of file------ */
	return FS_OK;
}
/* fdg */
static int bat_default_config(void)
{
	/* open usart 3 */
	bat_usart2 = open("/UART/2",__FS_OPEN_ALWAYS);
	/* config */
	if( bat_usart2 == NULL )
	{
		/* can not find some uart */
		return FS_ERR;
	}
	/* config msg */
	uart_config_msg msg;
	/* init param */
	msg.mode = 0;//none 8 1
	msg.index = 2;
	msg.rx_dma_buffer = (unsigned int)bat_dma_cache;
	msg.rx_dma_deepth = sizeof(bat_dma_cache);
	msg.baudrate = 115200;
	msg.tx_mode = _UART_TX_NARMOL;
	msg.rx_mode = _UART_RX_IT;
	/* init */
	fs_ioctl(bat_usart2,0,sizeof(msg),&msg);	
	/* create a thread here */
	DesEncrypt(&Skey[0],&g_abyValidKey[0],&Mkey[0]);
	/* send a start code */
	fs_write(bat_usart2,&byDataBuff[0],sizeof(byDataBuff));
	/* create a battery thread */
  shell_create_dynamic("battery_thread",battery_thread,4);	
	/*return*/
	return FS_OK;
}
/* file interface */
/* file & driver 's interface */
static struct file * bat_fopen (FAR struct file *filp)
{
	return &bat.flip;
}
/* ioctrl for control sbus */
static int bat_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
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
/* void bat thread , run as 100ms */
void battery_thread(void)
{
	/* enc steps */
	static unsigned char byState = 0;
	static unsigned char battery_buffer[128];
	unsigned char tmpbuf[8];
	/* cmd */
  switch(byState)
	{
		case 0:
		{
			/* read data */			
			int len = fs_read(bat_usart2,battery_buffer,sizeof(battery_buffer));
			/* got data */
			if( len == HANDSHAK_LENGTH )
			{
				AckToBat(battery_buffer,tmpbuf); //发送Tee给电池，实现握手
				byState = 1;
				fs_write(bat_usart2,tmpbuf,sizeof(tmpbuf));
			}
			else
			{
				/* send start code */
				fs_write(bat_usart2,&byDataBuff[0],sizeof(byDataBuff));//开机后向电池发送8个字节特定数据
			}					
		}
		break;

		case 1:
		{
			int len = fs_read(bat_usart2,battery_buffer,sizeof(battery_buffer));
			/* got data */
			if( len == HANDSHAK_LENGTH )
			{					
				if(!memcmp(battery_buffer,&byDataBuff[0],sizeof(byDataBuff)))
				{
					byState = 2; //握手成功
				}
			}
		}
		break;
		case 2://握手已经成功,以后跳过握手步骤，只需解析数据
		{
			int len = fs_read(bat_usart2,battery_buffer,sizeof(battery_buffer));
			/* got data */
			if( len == MESG_VALUE_SIZE )
			{
				BatMesgDecode(battery_buffer,&battery_msg);
			}
		}
		break;
		default:
		break;
	}	
}





















