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
	* usart is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "fs_config.h"
#include "string.h"
#include "gd32f30x.h"
/* some nesscery macro */
#define USART0_DR_ADDRESS      0x40013804
/* functions declare */
static int usart_heap_init(void);
static int usart_default_config(void);
static int usart_write( unsigned int usart_periph , void * data , unsigned int len );
static int usart_init_global( uart_config_msg * msg );
/* fs inode system register */
FS_INODE_REGISTER("/USART/",usart,usart_heap_init,0);
/* usart code */
const unsigned int usart_msg[5][9] = {
{ USART0,GPIOA,GPIO_PIN_9/* tx pin */,GPIOA,GPIO_PIN_10/* rx pin */,
  DMA0,DMA_CH3/* TX DMA */,DMA0,DMA_CH4/* RX DMA */},
};
/* heap init */
static int usart_heap_init(void)
{
  /* full of zero */
	memset(&usart,0,sizeof(usart));
	/* shell base */
	usart.shell_i = shell_sched_getfiles();
	/* driver config */
	usart.config = usart_default_config;
	/* file interface */
	usart.flip.f_inode = &usart;
	usart.flip.f_path = "/UART/";
	/* heap */
	
	/* add your own code here */
  usart.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;	
}


static unsigned char dma_receive_buffer[256];

/* usart_default_config */
static int usart_default_config(void)
{
  uart_config_msg msg;
	
	msg.baudrate = 115200;
	msg.index = 0;
	msg.rx_dma_buffer = (unsigned int)dma_receive_buffer;
	msg.rx_dma_deepth = sizeof(dma_receive_buffer);
	
	usart_init_global(&msg);
	/* return */
	return FS_OK;
}
/* static usart init global */
static int usart_init_global( uart_config_msg * msg )
{
	/* check param */
	if( msg->index >= 5 )
	{
		return FS_ERR;/* bad index */
	}
	/* enable all gpio clock */
	rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOD);	
	/* enable all of usart ' s clock */
	rcu_periph_clock_enable(RCU_USART0);
  rcu_periph_clock_enable(RCU_USART1);
	rcu_periph_clock_enable(RCU_USART2);
	rcu_periph_clock_enable(RCU_UART3);
	rcu_periph_clock_enable(RCU_UART4);
	
	/* connect port to USARTx_Tx */
	gpio_init(usart_msg[msg->index][1], GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, usart_msg[msg->index][2]);

	/* connect port to USARTx_Rx */
	gpio_init(usart_msg[msg->index][3], GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, usart_msg[msg->index][4]);

	/* UART configure */
	/* reset uart/usart */
	usart_deinit(usart_msg[msg->index][0]); 
  /* configration */
	usart_baudrate_set(usart_msg[msg->index][0], msg->baudrate);
	usart_word_length_set(usart_msg[msg->index][0], USART_WL_8BIT);
	usart_parity_config(usart_msg[msg->index][0], USART_PM_NONE);
	usart_stop_bit_set(usart_msg[msg->index][0], USART_STB_1BIT);
  /* usart mode settings */
	usart_receive_config(usart_msg[msg->index][0], USART_RECEIVE_ENABLE);     // enable receive
	usart_transmit_config(usart_msg[msg->index][0], USART_TRANSMIT_ENABLE);   // enable send 	
	/* enable usart 0 */
	usart_enable(usart_msg[msg->index][0]);
	
  /* dma config blove */
  dma_parameter_struct DmaInitParam;
  /* enable clock */
  rcu_periph_clock_enable(RCU_DMA0);
	/* DMA TX MODE config */
	dma_deinit(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);
	/* init param */
	DmaInitParam.direction = DMA_MEMORY_TO_PERIPHERAL;
	DmaInitParam.memory_addr = 0;
	DmaInitParam.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	DmaInitParam.memory_width = DMA_MEMORY_WIDTH_8BIT;
	DmaInitParam.number = 0;
	DmaInitParam.periph_addr = USART0_DR_ADDRESS;
	DmaInitParam.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	DmaInitParam.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	DmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;
	/* init */
	dma_init(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6], &DmaInitParam);
	/* configure DMA mode */
	dma_circulation_disable(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);
	dma_memory_to_memory_disable(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);
	/* USART DMA enable for transmission */
	usart_dma_transmit_config(usart_msg[msg->index][0], USART_DENT_ENABLE);
	/* enable DMA channel1 */
	dma_channel_disable(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);

	/* DMA RX MODE CONFIG */
	dma_deinit(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);
	/* set param */
	DmaInitParam.direction = DMA_PERIPHERAL_TO_MEMORY;
	/* set receive buffer */
	DmaInitParam.memory_addr = msg->rx_dma_buffer;
	DmaInitParam.number = msg->rx_dma_deepth;
	/* others */
	DmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;
	/* init */	
	dma_init(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8], &DmaInitParam);
	/* configure DMA mode */	
	dma_circulation_enable(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);
	dma_memory_to_memory_disable(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);
	/* enable usart0 dma receive mode */
	usart_dma_receive_config(usart_msg[msg->index][0], USART_DENR_ENABLE);
	/* enable dma */
	dma_channel_enable(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);		
	/* ok . we've finished . now return */
	return FS_OK;
}
/* int usart0 dma tx */
static int usart_write( unsigned int usart_periph , void * data , unsigned int len )
{
	/* disable first */
	dma_channel_disable(DMA0, DMA_CH3);
	/* reset dma */
	DMA_CHMADDR(DMA0,DMA_CH3) = (unsigned int)data;
	DMA_CHCNT(DMA0,DMA_CH3)   = ( len & 0xffff );
	/* enable dma again */
	dma_channel_enable(DMA0,DMA_CH3);	
	/* return ok as uaural */
	return FS_OK;
}
#if 1
static unsigned int dma_pos = 0;

static unsigned char dma_rec_test[256];

/* usart test task */
void usart_test_task(void)
{
	/* get and send */
	unsigned int dma_cnt = sizeof(dma_receive_buffer) - dma_transfer_number_get(DMA0, DMA_CH4);
	/* get data */
	if( dma_cnt != dma_pos )
	{
		/* ok ! we got some data */
		unsigned short dma_counter = ( dma_cnt > dma_pos ) ? ( dma_cnt - dma_pos ) : 
			                           ( sizeof(dma_receive_buffer) - dma_pos + dma_cnt );
		/* copy data */
		for( unsigned int i = 0 ; i < dma_counter ; i ++ )
		{
			dma_rec_test[i] = dma_receive_buffer[(dma_pos+i) %  sizeof(dma_receive_buffer)];
		}
		/* send again */
		usart_write(0,dma_rec_test,dma_counter);
		/* reset pos */
		dma_pos = dma_cnt;
	}
}
/* reg a test task */
FS_SHELL_STATIC(usart_test_task,usart_test_task,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD0_ID)); // 100ms
/* end of file */
#endif












