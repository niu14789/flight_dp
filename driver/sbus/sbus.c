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
#include "sbus.h"
#include "f_ops.h"
#include "f_drv.h"
#include "state.h"
/* file interface */
struct file * sbus_usart1 = NULL;
/* static valve channel */
static unsigned short sbus_channel_data[SBUS_CHANNEL_DEFAULT];
/*static data frame */
static unsigned char sbus_frame[50];
/* sbus reciver type */
/* gps define */
static unsigned char sbus_dma_cache[128];
/* define the inode */
FS_INODE_REGISTER("sbus.d",sbus,sbus_heap_init,0);
/* fs thread 20ms */
FS_SHELL_STATIC(sbus_callback,sbus_callback,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD1_ID));
/* data struct define */
struct sbus_bit_pick 
{
	unsigned char byte;
	unsigned char rshift;
	unsigned char mask;
	unsigned char lshift;
};
/* sbus_decoder code */
const struct sbus_bit_pick sbus_decoder[16][3] = 
{
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};
/* heap init */
static int sbus_heap_init(void)
{
	 /* full of zero */
	memset(&sbus,0,sizeof(sbus));
	/* shell base */
	sbus.shell_i = shell_sched_getfiles();
	/* driver config */
	sbus.config = sbus_default_config;
	/* file interface */
	sbus.flip.f_inode = &sbus;
	sbus.flip.f_path = "sbus.d";
	/* file interface */
	sbus.ops.open = sbus_fopen;
	sbus.ops.read = sbus_fread;
	sbus.ops.ioctl = sbus_ioctrl;
	/* heap */

	/* add your own code here */
  sbus.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT|__FS_IS_INODE_NODATA;
	/* ------end of file------ */
	return FS_OK;
}
/* fdg */
static int sbus_default_config(void)
{
	/* open usart 0 */
	sbus_usart1 = open("/UART/1",__FS_OPEN_ALWAYS);
	/* config */
	if( sbus_usart1 == NULL )
	{
		/* can not find some uart */
		return FS_ERR;
	}
	/* config msg */
	uart_config_msg msg;
	/* init param */
	msg.mode = _UART_PAR_EVEN << 8 | _UART_STOP_2 ; //event 2 stop bit
	msg.index = 1;
	msg.rx_dma_buffer = (unsigned int)sbus_dma_cache;
	msg.rx_dma_deepth = sizeof(sbus_dma_cache);
	msg.baudrate = 100000;
	msg.tx_mode = _UART_TX_DMA;
	msg.rx_mode = _UART_RX_IT;
	/* init */
	fs_ioctl(sbus_usart1,0,sizeof(msg),&msg);	
	/* open  */
	/*return*/
	return FS_OK;
}
/* file interface */
/* file & driver 's interface */
static struct file * sbus_fopen (FAR struct file *filp)
{
	return &sbus.flip;
}
/* data read */
static unsigned int sbus_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen)
{
	/* ignore the complier warring */
	(void)filp;
	/* read */
	if( buflen != sizeof(rcs_user_s) || buffer == NULL )
	{
		/* can not supply this format */
		return FS_OK;
	}
	/* read and transfer data */
	if( ! ( sbus.i_flags & __FS_IS_INODE_NODATA ) )
	{
		/* we 've got some data . clear flag */
		sbus.i_flags |= __FS_IS_INODE_NODATA;
		/* copy data */
		memcpy(buffer,sbus_channel_data,sizeof(sbus_channel_data));
		/* set lost flag . 1 = lost . 0 connect */
		(( rcs_user_s *)buffer )->flag = ( sbus.i_flags & __FS_IS_INODE_FAIL ) ? 1 : 0 ;
		/* end of func */
		return buflen;
		/* end of func */
	}
	/* read mag data */
	return FS_OK;
	/* end of data */
}
/* ioctrl for control sbus */
static int sbus_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* write */
		  sbus_reset_channels(pri_data,arg);
		  /* end of cmd */
	 	  break;
		case 1:
			/* decode */
		  sbus_callback();
		  /* end of cmd */
		  break;
		case 2:
			/* copy data */
		  memcpy(pri_data,sbus_channel_data,arg);
		  /* end of cmd */
		  break;	
		default :
      break;
	}
	/* return OK */
	return ret;
}
/* sbus decode */
static int sbus_decode(unsigned short *values, unsigned char *frame_cache)
{
	static unsigned int frame_lost_cnt = 0;
	static unsigned char flag_lost = 0;
	unsigned int channel = 0;
	unsigned int value = 0;
	unsigned int piece = 0;
	unsigned int pick = 0;
	/* use the decoder matrix to extract channel data */
	for( channel = 0; channel < SBUS_CHANNEL_DEFAULT; channel++)
	{
		/* clear */
		value = 0;
		/* data */
		for ( pick = 0; pick < 3; pick++ ) 
		{
			/* some bit */
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];
			/* wherere mask is 0 */
			if( decode->mask != 0) 
			{
				piece = frame_cache[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;
				/* data */
				value |= piece;
			}
		}
		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		values[channel] = (unsigned short)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	}
	/* frame lost */
	if( frame_cache[23] & (1 << 2) )
	{
		/* decremerter */
		frame_lost_cnt++;
		/* decremerter */
		if( frame_lost_cnt > 20 || flag_lost )
		{
			/* decremerter */
			flag_lost = 1;
			/* set to lost */
			sbus.i_flags |= __FS_IS_INODE_FAIL;
			/* bad data */
			sbus_reset_channels(sbus_channel_data,sizeof(sbus_channel_data)/sizeof(sbus_channel_data[0]));
			/* return lost frame */
			return FS_ERR;
			/*-------------------*/
		}
	}else
	{
		/* decremerter */
		frame_lost_cnt = 0;
		/* decremerter */
		flag_lost = 0;
		/* set to lost */
		sbus.i_flags &=~ __FS_IS_INODE_FAIL;	
    /* decremerter */		
	}
	/* return */
	return FS_OK;
}
/* callback */
static void sbus_callback(void)
{
	/* frame data */
	int i,len,frame_count = 0 , frame_position;
	/* buffer */
	unsigned char frame_buffer[25];
	/* unsigned short buffer */
	unsigned short sbus_channel_buffer[10];
	/* sbus error count */
	static unsigned int sbus_error_count = 0;
	/* read */
	len = fs_read(sbus_usart1,sbus_frame,sizeof(sbus_frame));
	/* check 1 */
	if( len != 25 )
	{
		/* error count */
		sbus_error_count++;
		/* give up this package */
		return;
		/* end of func */
	}
	/* search */
	for(  i = 0 ; i < 25 ; i ++ )
	{
		/* loop */
		if(sbus_frame[i] == 0x0f && sbus_frame[(i+24)%25] == 0x0)
		{
			/* get data */
			frame_count++;
			/* get pos */
			frame_position = i;
			/* end of func */
		}
	}
	/* only one frame head */
	if( frame_count != 1 )
	{
		/* head error */
		sbus_error_count++;
		/* get error */
		return;
	}
	/* memcpy */
	for( i = 0 ; i < 25 ; i ++ )
	{ 
		frame_buffer[i] = sbus_frame[ ( i + frame_position ) % 25 ];
	}
	/* decode */
	sbus_decode(sbus_channel_buffer,frame_buffer);
	/* error code min */
	if( sbus_channel_buffer[0] < 900 || sbus_channel_buffer[1] < 900 || 
		sbus_channel_buffer[2] < 900 || sbus_channel_buffer[3] < 900 ||
		sbus_channel_buffer[4] < 900 || sbus_channel_buffer[5] < 900 )
	{
		/* bad data */
		sbus_reset_channels(sbus_channel_data,sizeof(sbus_channel_data)/sizeof(sbus_channel_data[0]));
		/* bad data */
		return;
	}
	/* max */
	if( sbus_channel_buffer[0] > 2100 || sbus_channel_buffer[1] > 2100 || 
		sbus_channel_buffer[2] > 2100 || sbus_channel_buffer[3] > 2100 ||
		sbus_channel_buffer[4] > 2100 || sbus_channel_buffer[5] > 2100 )
	{
		/* bad data */
		sbus_reset_channels(sbus_channel_data,sizeof(sbus_channel_data)/sizeof(sbus_channel_data[0]));
		/* bad data */
		return;
	}
	/* ok we get the right sbus channel data */
	sbus_decode(sbus_channel_data,frame_buffer);
	/* read ok . insert a callback */
	/* good ! We got a good package */
	sbus.i_flags &=~ __FS_IS_INODE_NODATA;
  /* over */	
}
/* set value to mid */
static void sbus_reset_channels(unsigned short * rc,unsigned int len)
{
	/* loop */
	for( int i = 0 ; i < len ; i ++ )
	{
		/* reset */
		rc[i] = SBUS_DEFAULT_VALUE;
	}
	/* end of func */
}









































