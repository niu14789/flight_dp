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
#include "dps310.h"
/* fs inode system register */
FS_INODE_REGISTER("dps310.o",dps310,dps310_heap_init,0);
/* some params */
static DPS310_CONFIG dps310_cof;
/* gc aiosrsf */
const int gc_aiOsrSfConvert[8] = {524288, 1572864, 3670016, 7864320, 
                                   253952, 516096, 1040384, 2088960};
/* some other data */
const unsigned int g_adwTypeCompare[TYPE_NUM]  = 
{ 0x7,  0x7f, 0x7ff,  0x7fff,  0x7ffff,  0x7fffff,  0x7ffffff};
const unsigned int g_adwTypeSubtract[TYPE_NUM] = 
{0x10, 0x100,0x1000, 0x10000, 0x100000, 0x1000000, 0x10000000};
/* open dps310 dev to get spi interface */
static struct file * spi_handle;
/* heap init */
static int dps310_heap_init(void)
{
  /* full of zero */
	memset(&dps310,0,sizeof(dps310));
	/* shell base */
	dps310.shell_i = shell_sched_getfiles();
	/* driver config */
	dps310.config = dps310_default_config;
	/* file interface */
	dps310.flip.f_inode = &dps310;
	dps310.flip.f_path = "dps310.o";
	/* dps310 file interface */
	dps310.ops.open = dps310_fopen;
	dps310.ops.read = dps310_fread;
	dps310.ops.ioctl = dps310_ioctrl;
	/* heap */
	/* add your own code here */
	
	/* return ok */
	return FS_OK;
}
/* dps310_default_config dps310 init */
static int dps310_default_config(void)
{
	/* something need init . write here */
	/* add your own code here */
	spi_handle = open("icm206.d",__FS_OPEN_ALWAYS);
	/* check */
	if( spi_handle == NULL )
	{
		/* open fail */
		return FS_ERR;
		/* end of file */
	}
	/*  is init ? */
	if( ! ( spi_handle->f_oflags & __FS_IS_INODE_OK ) )
	{
		/* init spi dev */
		fs_ioctl(spi_handle,5,0,0);
		/* end of file  */
	}
	/* basic init */
	dps310_basic_init();
	/* end of file */
	return FS_OK;
}
/* file & driver 's interface */
static struct file * dps310_fopen (FAR struct file *filp)
{
	/* return flip data */
	return &dps310.flip;
}
/* data read */
static unsigned int dps310_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen)
{
	/* ignore the complier warring */
	(void)filp;
	/* read */
	if( buflen != sizeof(BARO_METER_DATA) || buffer == NULL )
	{
		/* can not supply this format */
		return 0;
	}
	/* dps310 OK ? */
	if( dps310.i_flags & __FS_IS_INODE_FAIL )
	{ 
		/* return directer */
		return FS_OK;/* may be occur some errors */
	}
	/* return data */
	dps310_read_press(buffer);
	/* return OK */
	return buflen;
}
/* dps310 ioctrl */
static int dps310_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		/* select a cmd */
    case 0:
			/* dps310_write_reg */
		  ret = dps310_hd_init();
		  /* end of cmd */
			break;
		case 1:
			/* dps310_read_reg */
		  ret = dps310_read(arg>>16,pri_data,arg&0xff);
		  /* end of cmd */
			break;		
		case 2:
			/* dps310_read_reg */
		  ret = dps310_basic_init();
		  /* end of cmd */
			break;		
		case 3:
			/* dps310_read_reg */
		  ret = dps310_read_cof(pri_data);
		  /* end of cmd */
			break;			
		case 4:
			/* dps310_read_reg */
		  ret = dps310_read_cof(pri_data);
		  /* end of cmd */
			break;	
		case 5:
			/* dps310_read_reg */
		  dps310_read_press(pri_data);
		  /* end of cmd */
			break;			
		default :
      break;			
	}
	/* return ok */
	return ret;
}
/* static dps cs pin init */
static int dps310_hd_init(void)
{
	/* enable clock */
  rcu_periph_clock_enable(RCU_GPIOA);
	/* config cs pin PA0 */
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	/* set to high */
	gpio_bit_set(GPIOA,GPIO_PIN_0);
	/* retrurn */
	return FS_OK;
	/* end of func */
}
/* dps 310 read */
static int dps310_read(unsigned char reg,unsigned char * buf,unsigned char cnt)
{
	/* reset cs pin */
	gpio_bit_set(GPIOA, GPIO_PIN_4);
	gpio_bit_reset(GPIOA, GPIO_PIN_0);
	/* write one byte */
	fs_ioctl(spi_handle , 6 , reg | 0x80 , 0 ); 
	/* read  */
	fs_ioctl(spi_handle , 7 , cnt , buf );
	/* set cs pin */
	gpio_bit_set(GPIOA, GPIO_PIN_0);
	/* end of file */
	return FS_OK;
}
/* dps310 write reg */
static void dps310_write_reg(unsigned char addr,unsigned char cmd)
{
	/* reset cs pin */
	gpio_bit_reset(GPIOA, GPIO_PIN_0);
	/* write one byte */
	fs_ioctl(spi_handle , 6 ,addr & 0x7f , 0 );
	/* write one byte */
	fs_ioctl(spi_handle , 6 , cmd , 0 );
  /* set cs pin */
	gpio_bit_set(GPIOA, GPIO_PIN_0);
}
/* dps310 read reg */
static unsigned char dps310_read_reg(unsigned char addr)
{
	/* temp file */
	unsigned char byTemp = 0;
	/* reset cs pin */
	gpio_bit_reset(GPIOA, GPIO_PIN_0);
	/* write cmd */
	fs_ioctl(spi_handle , 6 , addr | 0x80 , 0 );
	/* read */
	byTemp =  fs_ioctl(spi_handle , 6 , 0xff , 0 );
  /* set cs pin */
	gpio_bit_set(GPIOA, GPIO_PIN_0);
	/* return */
	return byTemp;
}
/* static dps310 basic init */
static int dps310_basic_init(void)
{
	/* define some function */
	unsigned char byData;
	unsigned char abyTemp[4];
	/* hardware init */
	dps310_hd_init();
	/* read dps310 */
	dps310_read(IFX_DPS310_PROD_REV_ID_REG_ADDR,&abyTemp[0],1);
	/* dev identify */
	if( abyTemp[0] != 0x10 )
	{
		/* set dps310 to error */
		dps310.i_flags = __FS_IS_INODE_FAIL; 
		/* return directer */
		return FS_ERR;/* may be occur some errors */
	}
	/* s_Dps310ReadCalibCoeffs */
	dps310_read_cof(&dps310_cof);
	/* write reg */
	dps310_write_reg(0x0e,0xa5);
	/* write reg */
	dps310_write_reg(0x0f, 0x96);
	/* write reg */
	dps310_write_reg(0x62, 0x02);
	/* write reg */
	dps310_write_reg(0x0e, 0x00);
	/* write reg */
	dps310_write_reg(0x0f, 0x00);
	/* write reg */
	abyTemp[0] = dps310_read_reg((unsigned char)IFX_DPS310_TMP_COEF_SRCE_REG_ADDR);
  /* write reg */
	abyTemp[0] &= 0x80;
	/* write reg */
	dps310_write_reg(IFX_DPS310_TMP_CFG_REG_ADDR, abyTemp[0]|IFX_DPS310_TEMPERATURE_MR|IFX_DPS310_TEMPERATURE_OSR);
	/* write reg */
	dps310_write_reg(IFX_DPS310_PRS_CFG_REG_ADDR, IFX_DPS310_PRESSURE_MR|IFX_DPS310_PRESSURE_OSR);
	/* write reg */
	byData = 0;
	/*If oversampling rate for temperature is greater than 8 times, then set TMP_SHIFT bit in CFG_REG */
	if (IFX_DPS310_TEMPERATURE_OSR > OSR_8) 
	{
		byData |= (unsigned char)IFX_DPS310_CFG_TMP_SHIFT_EN_SET_VAL;
	}
	/*If oversampling rate for pressure is greater than 8 times, then set P_SHIFT bit in CFG_REG */
	if (IFX_DPS310_PRESSURE_OSR > OSR_8) 
	{
		byData |= (unsigned char)IFX_DPS310_CFG_PRS_SHIFT_EN_SET_VAL;
	}
	/* write CFG_REG */
	dps310_write_reg(IFX_DPS310_CFG_REG_ADDR, byData);
	/* write reg */
	dps310_write_reg(IFX_DPS310_MEAS_CFG_REG_ADDR, (unsigned char)DPS310_MODE_BACKGROUND_ALL);
	/* ok */
	return FS_OK;
}
/* u32 to style */
static int U32ToSType(unsigned int dwIn, dps310_s_type eType)
{
	/* some default */
	int iTemp = dwIn;
	/* transfer */
	if (dwIn > g_adwTypeCompare[eType]) 
	{
		iTemp -= g_adwTypeSubtract[eType];
	}
	/* return */
	return iTemp;
}
/*--------------------------------*/
static int dps310_read_cof(DPS310_CONFIG *poCfg)
{
	/* define some func */
	unsigned char abyRedBuffer[IFX_DPS310_COEF_LEN] = { 0 };
	float fKp, fKt;
	unsigned int dwTemp;
	/* read data from spi */
	dps310_read(IFX_DPS310_COEF_REG_ADDR,abyRedBuffer,IFX_DPS310_COEF_LEN);
	/* transfer start */
	fKp = gc_aiOsrSfConvert[IFX_DPS310_PRESSURE_OSR];
	fKt = gc_aiOsrSfConvert[IFX_DPS310_TEMPERATURE_OSR];
	/* transfer 1 */
	dwTemp = abyRedBuffer[0];
	dwTemp <<= 4;
	dwTemp += (abyRedBuffer[1] >> 4);
	poCfg->fBeta0 = (short)U32ToSType(dwTemp, TYPE_S12);
	poCfg->fBeta0 *= 0.5f;
	/* transfer 1 */
	dwTemp = abyRedBuffer[1]&0xf;
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[2];
	poCfg->fBeta1 = (short)U32ToSType(dwTemp, TYPE_S12);
	poCfg->fBeta1 /= fKt;
	/* transfer 1 */
	dwTemp = abyRedBuffer[3];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[4];
	dwTemp <<= 4;
	dwTemp += ((abyRedBuffer[5] >> 4) & 0x0f);
	poCfg->iAlpha0 = U32ToSType(dwTemp, TYPE_S20);
	/* transfer 1 */
	dwTemp = (abyRedBuffer[5] & 0x0F);
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[6];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[7];
	poCfg->fAlpha1 = U32ToSType(dwTemp, TYPE_S20);
	poCfg->fAlpha1 /= fKp;
	/* transfer 1 */
	dwTemp = abyRedBuffer[8];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[9];
	poCfg->fAlpha4 = U32ToSType(dwTemp, TYPE_S16);
	poCfg->fAlpha4 /= fKt;
	/* transfer 1 */
	dwTemp = abyRedBuffer[10];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[11];
	poCfg->fAlpha5 = U32ToSType(dwTemp, TYPE_S16);
	poCfg->fAlpha5 /= fKt;
	poCfg->fAlpha5 /= fKp;
	/* transfer 1 */
	dwTemp = abyRedBuffer[12];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[13];
	poCfg->fAlpha2 = U32ToSType(dwTemp, TYPE_S16);
	poCfg->fAlpha2 /= fKp;
	poCfg->fAlpha2 /= fKp;
	/* transfer 1 */
	dwTemp = abyRedBuffer[14];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[15];
	poCfg->fAlpha6 = U32ToSType(dwTemp, TYPE_S16);
	poCfg->fAlpha6 /= fKt;
	poCfg->fAlpha6 /= fKp;
	poCfg->fAlpha6 /= fKp;
	/* transfer 1 */
	dwTemp = abyRedBuffer[16];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[17];
	poCfg->fAlpha3 = U32ToSType(dwTemp, TYPE_S16);
	poCfg->fAlpha3 /= fKp;
	poCfg->fAlpha3 /= fKp;
	poCfg->fAlpha3 /= fKp;
	/* return OK */
	return FS_OK;
}
/* dps310 read press and temprature */
static void dps310_read_press(BARO_METER_DATA *poBaroMeterData)
{
	/* default some data */
	int iPressRaw, iTemperatureRaw;
	unsigned char abyRedBuffer[IFX_DPS310_PSR_TMP_READ_LEN] = { 0 };
	unsigned int dwTemp;
	/* read buffer */
	dps310_read(IFX_DPS310_PSR_TMP_READ_REG_ADDR,abyRedBuffer,IFX_DPS310_PSR_TMP_READ_LEN);
	/* read data */
	dwTemp = abyRedBuffer[0];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[1];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[2];
	/* transfer */
	iPressRaw = U32ToSType(dwTemp, TYPE_S24);
	/* read two */
	dwTemp = abyRedBuffer[3];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[4];
	dwTemp <<= 8;
	dwTemp += abyRedBuffer[5];
	/* transfer */
	iTemperatureRaw = U32ToSType(dwTemp, TYPE_S24);
	/* transfer to raw press */
	poBaroMeterData->m_fPress= dps310_cof.iAlpha0
				 + (dps310_cof.fAlpha1 + (dps310_cof.fAlpha2 + dps310_cof.fAlpha3*iPressRaw)*iPressRaw)*iPressRaw
				 + (dps310_cof.fAlpha4 + (dps310_cof.fAlpha5 + dps310_cof.fAlpha6*iPressRaw)*iPressRaw)*iTemperatureRaw;
	/* trasnfer to tempature */
	poBaroMeterData->m_fTempature = dps310_cof.fBeta0 + dps310_cof.fBeta1*iTemperatureRaw;
	/* end of data */
}










