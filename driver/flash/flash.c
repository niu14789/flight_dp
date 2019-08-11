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
#include "flash.h"
/* fs inode system register */
FS_INODE_REGISTER("flash.o",flash,flash_heap_init,0);
/* heap init */
static int flash_heap_init(void)
{
  /* full of zero */
	memset(&flash,0,sizeof(flash));
	/* shell base */
	flash.shell_i = shell_sched_getfiles();
	/* driver config */
	flash.config = flash_default_config;
	/* file interface */
	flash.flip.f_inode = &flash;
	flash.flip.f_path = "flash.o";
	/* flash file interface */
	flash.ops.open = flash_fopen;
	flash.ops.ioctl = flash_ioctrl;
	/* add your own code here */
  flash.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* flash_default_config flash init */
static int flash_default_config(void)
{
	/* something need init . write here */
	/* add your own code here */
	
	/* end of file */
	return FS_OK;
}
/* file & driver 's interface */
static struct file * flash_fopen (FAR struct file *filp)
{
	/* return flip data */
	return &flash.flip;
	/* return OK */
}
/* static flash io ctrl */
/* icm206 ioctrl */
static int flash_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* temp */
	unsigned int * io_tmp = pri_data;
	/* select a cmd */
	switch(cmd)
	{
		/* select a cmd */ 
		case 0:
			/* flash_program_word */
			ret = flash_program_word(io_tmp[0],io_tmp[1]);
			/* end of cmd */
			break;
		case 1:
			/* flash_program_word */
			ret = flash_write(io_tmp[0],(const void *)io_tmp[1],arg);
			/* end of cmd */
			break;	
		case 2:
			/* flash_program_word */
			ret = flash_read(io_tmp[0],(void *)io_tmp[1],arg);
			/* end of cmd */
			break;		
		case 3:
			/* flash_program_word */
			ret = flash_check_sum(pri_data,arg);
			/* end of cmd */
			break;	
		case 4:
			/* flash_program_word */
			ret = flash_erase_page(arg);
			/* end of cmd */
			break;			
		default:
			break;
	}
	/* return OK */
	return ret;
}
/* static calibrate check sum */
static unsigned int flash_check_sum(const void * src,unsigned int len)
{
	/* transfer data */
	const unsigned char * w_data = src;
  /* sum */
  unsigned int sum = 0;
  /* calibrate */
  for( int i = 0 ; i < len ; i ++ )
	{
		sum += w_data[i];
	}	
	/* return sum */
	return sum;
}
/* static erase the flash zone */
static int flash_erase_page(unsigned int page_addr)
{
	/* flash state */
	fmc_state_enum fmc_state;
	/* unlock the flash program/erase controller */
	fmc_unlock();
	/* clear all pending flags */
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	//erase page
	fmc_state = fmc_page_erase(page_addr);
	//clear flag
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	/* lock the main FMC after the erase operation */
	fmc_lock();	
	/* return */
	return ( fmc_state == FMC_READY ) ? FS_OK : FS_ERR;
}
/* static flash program one word */
static int flash_program_word(unsigned int addr,unsigned int dat)
{
	/* copy data */
	fmc_unlock();
	/* write */		
	fmc_state_enum sta = fmc_word_program(addr,dat);
	/* write */
	fmc_lock();		
	/* return */
	return sta;
	/* end of data */
}
/* flash basic write */
static int flash_write(unsigned int base_addr,const void * src,unsigned int len)
{
	/* assist */
	if( len == 0 || src == NULL || len > 2048 )
	{
		return FS_ERR;/* bad data */
	}
	/* transfer data */
	const unsigned int * w_data = src;
	/* base data */
	const unsigned int * base = (const unsigned int *)base_addr;
	/* calibrate first */
	const unsigned int sum = flash_check_sum( src , len );
	/* word sum */
	const unsigned int word_sum =  ( len % 4 ) ? ( len / 4 + 1 ) : ( len / 4 );
	/* check them */
	if( base[word_sum] == sum && memcmp(base,src,len) == 0 )
	{
		/* all of things are ok . no need to write */
		return FS_OK;
		/* end of data */
	}
	/* erase the page */
	if( flash_erase_page(base_addr) != FS_OK )
	{
		/* occurs some errors */
		return FS_ERR;
	}
	/* write addr */
	unsigned int w_addr = base_addr;
	/* write into flash */
	for( int i = 0 ; i < len / 4 ; i ++ )
	{
		/* wtite into flash */
		flash_program_word(w_addr,w_data[i]);
		/* incremete */
		w_addr += 4;
		/* end of data */
	}
	/* erase ok  word tb */
	unsigned int word_space = 0;
  /* write */
	unsigned char copy_size = len % 4;
	/* if( */
	if( copy_size > 0 )
	{
		/* copy to word space area */
    memcpy(&word_space,&w_data[len / 4] , copy_size );
		/* write */		
		flash_program_word(w_addr,word_space);
		/* incremete */
		w_addr += 4;		
		/* end of data */
	}	
	/* write into check sum */
	flash_program_word(w_addr,sum);		
	/* check them */
	if( base[word_sum] == sum && memcmp(base,src,len) == 0 )
	{
		/* all of things are ok . no need to write */
		return FS_OK;
		/* end of data */
	}	
	/* else reuturn error */
	return FS_ERR;
	/* end of file */
}
/* flash_read */
/* flash basic write */
static int flash_read(unsigned int base_addr,void * src,unsigned int len)
{
	/* assist */
	if( len == 0 || src == NULL || len > 2048 )
	{
		return FS_ERR;/* bad data */
	}
	/* base data */
	const unsigned int * base = (const unsigned int *)base_addr;
	/* calibrate first */
	const unsigned int sum = flash_check_sum( base , len );
	/* word sum */
	const unsigned int word_sum =  ( len % 4 ) ? ( len / 4 + 1 ) : ( len / 4 );
	/* check them */
	if( base[word_sum] != sum )
	{
		/* bad data */
		return FS_ERR;
	}
	/* check ok copy date */
	memcpy( src , base , len );
  /* reutrn FS OK */
	return FS_OK;
}
/* end of files */































