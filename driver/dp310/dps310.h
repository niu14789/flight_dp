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
#ifndef __DPS310_H__
#define __DPS310_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"

/* Meaningful Default Configuration */
#define     IFX_DPS310_TEMPERATURE_OSR                  OSR_1
#define     IFX_DPS310_PRESSURE_OSR                     OSR_1
#define     IFX_DPS310_TEMPERATURE_MR                   TMP_MR_8
#define     IFX_DPS310_PRESSURE_MR                      PM_MR_128
 
/* Struct to hold calibration coefficients read from device*/
typedef struct
{    
	/* calibration coefficient */
	float fBeta0; // Beta0 = C0*0.5f
	float fBeta1; // Beta1 = C1/kt
	int   iAlpha0; // Alpha0 = C00
	float fAlpha1; // Alpha1 = C10/kp
	float fAlpha2; // Alpha2 = C20/(kp*kp)
	float fAlpha3; // Alpha3 = C30/(kp*kp*kp)
	float fAlpha4; // Alpha4 = C01/kt
	float fAlpha5; // Alpha5 = C11/(kt*kp)
	float fAlpha6; // Alpha6 = C21/(kt*kp*kp)
}DPS310_CONFIG;

#define false               0
#define true                1

 /* Attributes: Product identification and version */

#define     VENDOR_NAME                                 "Infineon"
#define     DRIVER_NAME                                 "IFXDD"
#define     DEVICE_NAME                                 "Digital Barometric Pressure Sensor"
#define     DEVICE_MODEL_NAME                           "DPS310"
#define     DEVICE_HW_VERSION                           1.0
#define     DRIVER_VERSION                              1.0
#define     DEVICE_PROD_REV_ID                          0x10

/* Attributes: Device performance :Pressure Sensing */
#define     IFX_DPS310_PROD_REV_ID_REG_ADDR             0x0D
#define     IFX_DPS310_PROD_REV_ID_LEN                  1
#define     IFX_DSPS310_PROD_REV_ID_VAL                 DEVICE_PROD_REV_ID

#define     IFX_DPS310_SOFT_RESET_REG_ADDR              0x0C
#define     IFX_DPS310_SOFT_RESET_REG_DATA              0x09
#define     IFX_DPS310_SOFT_RESET_REG_LEN               1
#define     IFX_DPS310_SOFT_RESET_VERIFY_REG_ADDR       0x06

#define     IFX_DPS310_COEF_REG_ADDR                    0x10
#define     IFX_DPS310_COEF_LEN                         18    // Length in bytes

#define     IFX_DPS310_TMP_COEF_SRCE_REG_ADDR           0x28
#define     IFX_DPS310_TMP_COEF_SRCE_REG_LEN            1    // Length in bytes
#define     IFX_DPS310_TMP_COEF_SRCE_REG_POS_MASK       7    // Length in bytes

#define     IFX_DPS310_PSR_TMP_READ_REG_ADDR            0x00
#define     IFX_DPS310_PSR_TMP_READ_LEN                 6

#define     IFX_DPS310_PRS_CFG_REG_ADDR                 0x06
#define     IFX_DPS310_PRS_CFG_REG_LEN                  1

#define     IFX_DPS310_TMP_CFG_REG_ADDR                 0x07
#define     IFX_DPS310_TMP_CFG_REG_LEN                  1

#define     IFX_DPS310_MEAS_CFG_REG_ADDR                0x08
#define     IFX_DPS310_MEAS_CFG_REG_LEN                 1

#define     IFX_DPS310_CFG_REG_ADDR                     0x09
#define     IFX_DPS310_CFG_REG_LEN                      1

#define     IFX_DPS310_CFG_TMP_SHIFT_EN_SET_VAL         0x08
#define     IFX_DPS310_CFG_PRS_SHIFT_EN_SET_VAL         0x04


#define     IFX_DPS310_FIFO_READ_REG_ADDR               0x00
#define     IFX_DPS310_FIFO_REG_READ_LEN                3
#define     IFX_DPS310_FIFO_BYTES_PER_ENTRY             3

#define     IFX_DPS310_FIFO_FLUSH_REG_ADDR              0x0C
#define     IFX_DPS310_FIFO_FLUSH_REG_VAL               0b1000000U

#define     IFX_DPS310_CFG_SPI_MODE_POS                 0
#define     IFX_DPS310_CFG_SPI_MODE_3_WIRE_VAL          1
#define     IFX_DPS310_CFG_SPI_MODE_4_WIRE_VAL          0

#define     IFX_DPS310_CFG_FIFO_ENABLE_POS              1
#define     IFX_DPS310_CFG_FIFO_ENABLE_VAL              1
#define     IFX_DPS310_CFG_FIFO_DISABLE_VAL             0

#define     IFX_DPS310_CFG_INTR_PRS_ENABLE_POS          4
#define     IFX_DPS310_CFG_INTR_PRS_ENABLE_VAL          1U
#define     IFX_DPS310_CFG_INTR_PRS_DISABLE_VAL         0U

#define     IFX_DPS310_CFG_INTR_TEMP_ENABLE_POS         5
#define     IFX_DPS310_CFG_INTR_TEMP_ENABLE_VAL         1U
#define     IFX_DPS310_CFG_INTR_TEMP_DISABLE_VAL        0U

#define     IFX_DPS310_CFG_INTR_FIFO_FULL_ENABLE_POS    6
#define     IFX_DPS310_CFG_INTR_FIFO_FULL_ENABLE_VAL    1U
#define     IFX_DPS310_CFG_INTR_FIFO_FULL_DISABLE_VAL   0U

#define     IFX_DPS310_CFG_INTR_LEVEL_TYP_SEL_POS       7
#define     IFX_DPS310_CFG_INTR_LEVEL_TYP_ACTIVE_H      1U
#define     IFX_DPS310_CFG_INTR_LEVEL_TYP_ACTIVE_L      0U

#define     IFX_DPS310_INTR_SOURCE_PRESSURE             0
#define     IFX_DPS310_INTR_SOURCE_TEMPERATURE          1
#define     IFX_DPS310_INTR_SOURCE_BOTH                 2

#define     IFX_DPS310_INTR_STATUS_REG_ADDR             0x0A
#define     IFX_DPS310_INTR_STATUS_REG_LEN              1
#define     IFX_DPS310_INTR_DISABLE_ALL                (U8)0b10001111

#define     EINVAL                                      1
#define     EIO                                         2

/* enum for seeting/getting device operating mode*/

typedef enum
{
	DPS310_MODE_IDLE = 0,
	DPS310_MODE_COMMAND_PRESSURE = 1,
	DPS310_MODE_COMMAND_TEMPERATURE = 2,
	DPS310_MODE_BACKGROUND_PRESSURE = 5,
	DPS310_MODE_BACKGROUND_TEMPERATURE = 6,
	DPS310_MODE_BACKGROUND_ALL = 7 
}dps310_operating_modes_e;

/* enum of oversampling rates for pressure and temperature*/
typedef enum
{
	TYPE_S4    = 0x0,
	TYPE_S8    = 0x1,
	TYPE_S12   = 0x2,
	TYPE_S16   = 0x3,
	TYPE_S20   = 0x4,
	TYPE_S24   = 0x5,
	TYPE_S28   = 0x6,
	TYPE_NUM   = 0x7 
} dps310_s_type;

/* enum of scaling coefficients either Kp or Kt*/
typedef enum
{
	OSR_SF_1 = 524288,
	OSR_SF_2 = 1572864,
	OSR_SF_4 = 3670016,
	OSR_SF_8 = 7864320,
	OSR_SF_16 = 253952,
	OSR_SF_32 = 516096,
	OSR_SF_64 = 1040384,
	OSR_SF_128 = 2088960 
} dps310_scaling_coeffs_e;

/* enum of oversampling rates for pressure and temperature*/
typedef enum
{
	OSR_1 = 0,
	OSR_2 = 1,
	OSR_4 = 2,
	OSR_8 = 3,
	OSR_16 = 4,
	OSR_32 = 5,
	OSR_64 = 6,
	OSR_128 = 7 
}dps310_osr_e;

/* enum of measurement rates for pressure*/

typedef enum
{
	PM_MR_1 = 0x00,
	PM_MR_2 = 0x10,
	PM_MR_4 = 0x20,
	PM_MR_8 = 0x30,
	PM_MR_16 = 0x40,
	PM_MR_32 = 0x50,
	PM_MR_64 = 0x60,
	PM_MR_128 = 0x70 
} dps310_pm_rate_e;

/* enum of measurement rates for temperature*/

typedef enum
{
	TMP_MR_1 = 0x00,
	TMP_MR_2 = 0x10,
	TMP_MR_4 = 0x20,
	TMP_MR_8 = 0x30,
	TMP_MR_16 = 0x40,
	TMP_MR_32 = 0x50,
	TMP_MR_64 = 0x60,
	TMP_MR_128 = 0x70 
} dps310_tmp_rate_e;

/* function declare */
static int dps310_heap_init(void);
static int dps310_default_config(void);
static int dps310_hd_init(void);
static int dps310_basic_init(void);
static int dps310_read(unsigned char reg,unsigned char * buf,unsigned char cnt);
static int dps310_read_cof(DPS310_CONFIG *poCfg);
static void dps310_read_press(BARO_METER_DATA *poBaroMeterData);

static struct file * dps310_fopen (FAR struct file *filp);
static unsigned int dps310_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static int dps310_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);

#endif 



