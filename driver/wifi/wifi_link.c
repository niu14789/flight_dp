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
/* some define */
static unsigned char wifi_receive_buffer[512];
static unsigned char wifi_parse_buffer[256];
/*-------------*/
void wifi_link_data_receive(void)
{
	/* some defines */
	static unsigned char steps = 0;
	static unsigned char read_data_len = 0;
	static unsigned char payload_cnt = 0;
	/* read data from usart3 */
	int len = wifi_receive_bytes(wifi_receive_buffer,sizeof(wifi_receive_buffer));
	/* got some data */
	if( len > 0 )
	{
		for( int i = 0 ; i < len ; i ++ )
		{
			/* parse */
			switch(steps)
			{
				case 0:
					/* head h */
				  if( wifi_receive_buffer[i] == 0xFF /*WIFI_PACKET_HEAD_H */ )
					{
						/* copy data */
						wifi_parse_buffer[0] = wifi_receive_buffer[i];
						/* switch */
						steps = 1;
					}
					break;
				case 1:
					/* head h */
				  if( wifi_receive_buffer[i] == 0xFD /*WIFI_PACKET_HEAD_L */ )
					{
						/* copy data */
						wifi_parse_buffer[1] = wifi_receive_buffer[i];
						/* switch */						
						steps = 2;
					}
					break;	
				case 2:
					/* len = payload + checksum */
				  read_data_len =  wifi_receive_buffer[i];
					/* copy data */
					wifi_parse_buffer[2] = wifi_receive_buffer[i];
				  /* payload cnt clear */
				  payload_cnt = 0;
					/* switch */
					steps = 3;
				  /* end of this cmd */
					break;					
				case 3:
					/* payload and checksum */
				  if( payload_cnt <= read_data_len )
					{
						/* copy data */
						wifi_parse_buffer[2+payload_cnt] = wifi_receive_buffer[i];
						/* over */
						payload_cnt ++;
						/* parse data */
						if( payload_cnt > read_data_len )
						{
#if 0
              wifi_data_packet_decode(wifi_parse_buffer);     							
#endif							
							steps = 0;
						}
					}
					else
					{
						steps = 0;
					}
				  /* end of this cmd */
					break;	
				default:
  				steps = 0;
					break;
			}
			/* end of this */
		}
	}
}
/* void wifi_link_data_send(void) */
void wifi_link_data_send(void)
{
	
}





















