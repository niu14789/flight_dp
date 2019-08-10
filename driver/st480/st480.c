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
#include "gd32f30x.h"
#include "string.h"
#include "fs_config.h"
#include "f_ops.h"
#include "state.h"
#include "st480.h"
/* fs inode system register */
FS_INODE_REGISTER("st480.o",st480,st480_heap_init,0);
/* heap init */
static int st480_heap_init(void)
{
  /* full of zero */
	memset(&st480,0,sizeof(st480));
	/* shell base */
	st480.shell_i = shell_sched_getfiles();
	/* driver config */
	st480.config = st480_default_config;
	/* file interface */
	st480.flip.f_inode = &st480;
	st480.flip.f_path = "st480.o";
	/* st480 file interface */
	st480.ops.open = st480_fopen;
	st480.ops.read = st480_fread;
	st480.ops.ioctl = st480_ioctrl;
	/* heap */
	int ret = st480_basic_init();
	/* add your own code here */
  st480.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return ret;
}
/* st480_default_config st480 init */
static int st480_default_config(void)
{
	/* something need init . write here */
	/* add your own code here */
	
	/* end of file */
	return FS_OK;
}
/* file & driver 's interface */
static struct file * st480_fopen (FAR struct file *filp)
{
	/* return flip data */
	return &st480.flip;
}
/* data read */
static unsigned int st480_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen)
{
	/* ignore the complier warring */
	(void)filp;
	/* read */
	if( buflen != sizeof(ST480_MAG_DEF) || buffer == NULL )
	{
		/* can not supply this format */
		return 0;
	}
	/* read mag data */
	return st480_read_mag(buffer) == FS_OK ? buflen : FS_OK;
	/* end of data */
}
/* icm206 ioctrl */
static int st480_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* temp */
	unsigned int * io_tmp = pri_data;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* st480_i2c_hd_init */
			st480_i2c_hd_init();
		  /* end of cmd */
			break;
		case 1:
			/* read byte */
		  ret = st480_i2c_wr_byte((unsigned char *)io_tmp[0],io_tmp[1],(unsigned char*)io_tmp[2],io_tmp[3]);
		  /* end of cmd */
			break;
		case 2:
			/* read byte */
		  ret = st480_i2c_read_byte((unsigned char *)io_tmp[0],io_tmp[1],io_tmp[2]);
		  /* end of cmd */
			break;
		default :
			break;
	}
	/* return ok */
	return ret;
	/* end of function */
}
/* I2C basic init */
static void st480_i2c_hd_init(void)
{
	/* Enable SDA0/SCL0 peripheral clock device */
	rcu_periph_clock_enable(RCU_GPIOB);      
	/* enable I2C0 clock */
	rcu_periph_clock_enable(RCU_I2C0);       
	/* config gpio . connect PB6 to I2C0_SCL */
	gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
	/* config gpio . connect PB7 to I2C0 SDA */
	gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
	/* release the GPIO source */
	gpio_bit_set(GPIOB, GPIO_PIN_6);
	gpio_bit_set(GPIOB, GPIO_PIN_7);	
	/* I2C basic init */
	/* configure I2C clock to 400K */
	i2c_clock_config( I2C0, 400000, I2C_DTCY_2); 
	/* configure I2C address */
	i2c_mode_addr_config( I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, SELF_MASTER_ADDR);
	/* enable i2c */
	i2c_enable(I2C0);
	/* enable acknowledge */	
	i2c_ack_config(I2C0, I2C_ACK_ENABLE); 
}
/* i2c basic function -> byte read and write */
static int st480_i2c_wr_byte(unsigned char * w_buf, unsigned char w_cnt, unsigned char *r_buf, unsigned char r_cnt)
{
	/* read data */
	int iReadLen;
	/* timeout detecter count */
	unsigned int t = 0;
	/* wait until I2C bus is idle */
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */
	}
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C0);
	/* clear time count */
	t = 0;
	/* wait until SBSEND bit is set */
	while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */		
	}
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C0, MAG_I2C_ADDRESS, I2C_TRANSMITTER);
	/* clear time count */
	t = 0;	
	/* wait until ADDSEND bit is set */
	while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */				
	}
	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	/* clear time count */
	t = 0;	
	/* while there is data to be written */
	while( w_cnt -- )
	{
		/* i2c transmit */
		i2c_data_transmit(I2C0, *w_buf);
		/* point to the next byte to be written */
		w_buf++;
		/* wait until BTC bit is set */
		while(!i2c_flag_get(I2C0, I2C_FLAG_BTC))
		{
			TIMEOUT_DETECTER(t,FS_OK);/* time out */					
		}
	}
	/* send a stop condition to I2C bus */
	i2c_stop_on_bus(I2C0);
	/* clear time count */
	t = 0;	
	/* wait until the stop condition is finished */
	while (I2C_CTL0(I2C0) & 0x0200)
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */				
	}
	/*******************************Read******************************************************************/
	/* clear time count */
	t = 0;	
	/* wait until I2C bus is idle */
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */				
	}
	/* read */
	if( 2 == r_cnt )
	{
		i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);
	}
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C0);
	/* clear time count */
	t = 0;	
	/* wait until SBSEND bit is set */
	while( !i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */			
	}
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C0, MAG_I2C_ADDRESS, I2C_RECEIVER);
	/* read */
	if( r_cnt < 3 )
	{
		/* disable acknowledge */
		i2c_ack_config(I2C0, I2C_ACK_DISABLE);
	}
	/* clear time count */
	t = 0;	
	/* wait until ADDSEND bit is set */
	while( !i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) )
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */			
	}
	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	/* read */
	if( 1 == r_cnt )
	{
		/* send a stop condition to I2C bus */
		i2c_stop_on_bus(I2C0);
	}
	/* set clear */
	iReadLen = 0;
	/* while there is data to be read */
	while(r_cnt)
	{
		/* read data */
		if( 3 == r_cnt )
		{
			/* clear time count */
			t = 0;			
			/* wait until BTC bit is set */
			while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
			{
				TIMEOUT_DETECTER(t,FS_OK);/* time out */					
			}
			/* disable acknowledge */
			i2c_ack_config(I2C0, I2C_ACK_DISABLE);
		}
		if (2 == r_cnt)
		{
			/* clear time count */
			t = 0;			
			/* wait until BTC bit is set */
			while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
			{
				TIMEOUT_DETECTER(t,FS_OK);/* time out */					
			}
			/* send a stop condition to I2C bus */
			i2c_stop_on_bus(I2C0);
		}
		/* wait until the RBNE bit is set and clear it */
		if (i2c_flag_get(I2C0, I2C_FLAG_RBNE))
		{
			/* read a byte from the EEPROM */
			*r_buf = i2c_data_receive(I2C0);
			/* point to the next location where the byte read will be saved */
			r_buf++;
			/* decrement the read bytes counter */
			r_cnt--;
			/* decrment */
			iReadLen++;
		}
	}
	/* clear time count */
	t = 0;	
	/* wait until the stop condition is finished */
	while (I2C_CTL0(I2C0) & I2C_CTL0_STOP)
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */			
	}
	/* enable acknowledge */
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	/* return */
	i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
	/* return */
	return iReadLen;
}
/* i2C read data */
static int st480_i2c_read_byte( unsigned char* r_buf, unsigned char r_offset, unsigned short r_cnt)
{
	/* read len */
	unsigned int dwReadLen = 0;
	/* timeout detecter count */
	unsigned int t = 0;	
	/* wait until I2C bus is idle */
	while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */
	}
	/* if read data is 2 */
	if( 2 == r_cnt )
	{
		i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);
	}
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C0);
	/* timeout detecter count */
	t = 0;		
	/* wait until SBSEND bit is set */
	while( !i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */
	}
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C0, MAG_I2C_ADDRESS, I2C_TRANSMITTER);
	/* timeout detecter count */
	t = 0;	
	/* wait until ADDSEND bit is set */
	while( !i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */
	}
	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	/* timeout detecter count */
	t = 0;	
	/* wait until the transmit data buffer is empty */
	while( SET != i2c_flag_get(I2C0, I2C_FLAG_TBE))
	{     
		TIMEOUT_DETECTER(t,FS_OK);/* time out */		
	}
	/* enable I2C0*/
	i2c_enable(I2C0);
	/* send the EEPROM's internal address to write to */
	i2c_data_transmit(I2C0, r_offset);
	/* timeout detecter count */
	t = 0;	
	/* wait until BTC bit is set */
	while( !i2c_flag_get(I2C0, I2C_FLAG_BTC))
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */
	}
	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C0);
	/* timeout detecter count */
	t = 0;	
	/* wait until SBSEND bit is set */
	while( !i2c_flag_get(I2C0, I2C_FLAG_SBSEND))
	{    
		TIMEOUT_DETECTER(t,FS_OK);/* time out */		
	}
	/* send slave address to I2C bus */
	i2c_master_addressing(I2C0, MAG_I2C_ADDRESS, I2C_RECEIVER);
	/* send data is three */
	if( r_cnt < 3 )
	{
		/* disable acknowledge */
		i2c_ack_config(I2C0, I2C_ACK_DISABLE);
	}
	/* timeout detecter count */
	t = 0;	
	/* wait until ADDSEND bit is set */
	while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND))
	{      
		TIMEOUT_DETECTER(t,FS_OK);/* time out */
	}
	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	/* clear */
	if( 1 == r_cnt )
	{
		/* send a stop condition to I2C bus */
		i2c_stop_on_bus(I2C0);
	}
	/* while there is data to be read */
	while( r_cnt )
	{
		if( 3 == r_cnt )
		{
			/* timeout detecter count */
			t = 0;			
			/* wait until BTC bit is set */
			while( !i2c_flag_get(I2C0, I2C_FLAG_BTC))
			{
				TIMEOUT_DETECTER(t,FS_OK);/* time out */				
			}
			/* disable acknowledge */
			i2c_ack_config(I2C0, I2C_ACK_DISABLE);
		}
		/* next */
		if( 2 == r_cnt) 
		{ 
			/* timeout detecter count */
			t = 0;
			/* wait until BTC bit is set */
			while (!i2c_flag_get(I2C0, I2C_FLAG_BTC))
			{
				TIMEOUT_DETECTER(t,FS_OK);/* time out */				
			}
			/* send a stop condition to I2C bus */
			i2c_stop_on_bus(I2C0);
		}
		/* wait until the RBNE bit is set and clear it */
		if( i2c_flag_get(I2C0, I2C_FLAG_RBNE)) 
		{
			/* read a byte from the EEPROM */
			*r_buf = i2c_data_receive(I2C0);
			/* point to the next location where the byte read will be saved */
			r_buf++;
			/* decrement the read bytes counter */
			r_cnt--;
			/* decrement */
			dwReadLen++;
		}
	}
	/* timeout detecter count */
	t = 0;	
	/* wait until the stop condition is finished */
	while( I2C_CTL0(I2C0) & I2C_CTL0_STOP)
	{
		TIMEOUT_DETECTER(t,FS_OK);/* time out */		
	}
	/* enable acknowledge */
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	/* enable acknowledge */
	i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);
	/* return len */
	return dwReadLen;
}
/* static int st480 basci init */
static int st480_basic_init(void)
{
	/* defines */
	st480_i2c_hd_init();
	/* the addr of register   Note: Address[1:0] = 0x00 */
	unsigned char abyTemp[4] = {0x60,0x00,0x7C,0x00};
	/* wr data */
	int ret = st480_i2c_wr_byte(abyTemp, 4, abyTemp, 1);
  /* the command of write register */
	abyTemp[0] = 0x60;
	abyTemp[1] = 0x00;
	abyTemp[2] = 0x18;
	abyTemp[3] = 0x08;
	/* the addr of register   Note: Address[1:0] = 0x00 */
	ret = st480_i2c_wr_byte( abyTemp , 4, abyTemp, 1 );
	/* get wr ok or timeout */
	if( ret == FS_OK )
	{
		/* time out */
		return FS_ERR;
		/* end of func */
	}
	/* read */
	ret = st480_i2c_read_byte( abyTemp , 0x3e , 1 );  //Single measure mode
	/* get wr ok or timeout */
	if( ret == FS_OK )
	{
		/* time out */
		return FS_ERR;
		/* end of func */
	}
	/* good ! ok */
  return FS_OK;	
	/* end of func */
}
/* static int st480 read mag data */
static int st480_read_mag( ST480_MAG_DEF * st480_d )
{
	/* read buffer */
	unsigned char abyTemp[8];
	/* result */
	int iRet;
	/* read data . Burst measure mode */
	iRet = st480_i2c_read_byte( abyTemp , 0x4e , 7 );  
	/* can not read or not */
	if( 7 != iRet )
	{
		/* st480 error count */
		static unsigned char st480_error_count = 0;
		/* over three times */
		if( st480_error_count ++ > 3 )
		{
			/* clear count flag */
			st480_error_count = 0;
			/* deinit i2c */
			i2c_deinit(I2C0);
			/* init again */
			st480_basic_init();
			/* end of func */
		}
		/* can not read */
		return FS_ERR;
		/* end of func */
	}
	/* is data updata ? */
	if (! ( abyTemp[0] & 0x10 ) )
	{
		/* copy data */
		st480_d->mag[0] = (short)(abyTemp[1] << 8 | abyTemp[2] ) / 667.0f ;
		st480_d->mag[1] = (short)(abyTemp[3] << 8 | abyTemp[4] ) / 667.0f ;
		st480_d->mag[2] = (short)(abyTemp[5] << 8 | abyTemp[6] ) / 600.0f ;
		
		/* read something that I don't know either */
		iRet = st480_i2c_read_byte( abyTemp , 0x3e , 1 );
		/* */
		if( 1 != iRet )
		{
			return FS_ERR;
		}		
		/* OK */
		return FS_OK;
		/* end of file */
	}
	/* have've update yet */
	return FS_ERR;
	/* end of func */
}



















