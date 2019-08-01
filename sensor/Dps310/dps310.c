
 /**********************************************************************************************************************
 File Name     :  dps310.cpp
 Copy right     :  Botan
 description    : Driver for Infineon DPS310 Digital Barometric Pressure Sensor
 Author           :   Ryan Huang
 Version          :   V1.0
 Date              :   2019-04-24
 **********************************************************************************************************************/

#include "dps310.h"
#include "..\..\driver\i2c\Bsp_i2c.h"
#include "systick.h"

#include <math.h>

/* Meaningful Default Configuration */
#define     IFX_DPS310_TEMPERATURE_OSR                  OSR_1
#define     IFX_DPS310_PRESSURE_OSR                     OSR_1
#define     IFX_DPS310_TEMPERATURE_MR                   TMP_MR_8
#define     IFX_DPS310_PRESSURE_MR                      PM_MR_128

//#define DPS310_I2C_ADDRESS   0x3b // read addr is 0x3b*2+1, write addr is 0x3b*2
#define DPS310_I2C_ADDRESS   0x77 // read addr is 0x3b*2+1, write addr is 0x3b*2
#define DPS310_I2C_SPEED   100000


#define DPS310_I2C_PORT  I2C_PORT1



/* Struct to hold calibration coefficients read from device*/
typedef struct
{    
    /* calibration coefficient */
    F32 fBeta0; // Beta0 = C0*0.5f
    F32 fBeta1; // Beta1 = C1/kt
    I32 iAlpha0; // Alpha0 = C00
    F32 fAlpha1; // Alpha1 = C10/kp
    F32 fAlpha2; // Alpha2 = C20/(kp*kp)
    F32 fAlpha3; // Alpha3 = C30/(kp*kp*kp)
    F32 fAlpha4; // Alpha4 = C01/kt
    F32 fAlpha5; // Alpha5 = C11/(kt*kp)
    F32 fAlpha6; // Alpha6 = C21/(kt*kp*kp)
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

#ifndef NULL
#define     NULL                                        ((void*)0)
#endif // NULL



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
} dps310_osr_e;



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

//const int gc_aiOsrSfConvert[8] = {0x80000, 0x180000, 0x380000, 0x780000, 0x3e000, 0x7e000, 0xfe000, 0x1fe000};
const int gc_aiOsrSfConvert[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};


static I2cBaseDefine *gs_poI2cHandle = NULL;

static DPS310_CONFIG gs_oDps310Cfg;

////////////////////////////////////////4bits  8bits  12bits   16bits      20bits      24bits        28bits
U32 g_adwTypeCompare[TYPE_NUM]  = { 0x7,  0x7f, 0x7ff,  0x7fff,  0x7ffff,  0x7fffff,  0x7ffffff};
U32 g_adwTypeSubtract[TYPE_NUM] = {0x10, 0x100,0x1000, 0x10000, 0x100000, 0x1000000, 0x10000000};

I32 U32ToSType(U32 dwIn, dps310_s_type eType)
{
    I32 iTemp = dwIn;
    
    if (dwIn > g_adwTypeCompare[eType]) {
        iTemp -= g_adwTypeSubtract[eType];
    }
    return iTemp;
}

// 0:success, other:failed
static int s_Dps310ReadCalibCoeffs(DPS310_CONFIG *poCfg)
{
    I32 iRet;
    U8 abyRedBuffer[IFX_DPS310_COEF_LEN] = { 0 };
    F32 fKp, fKt;
    U32 dwTemp;

    iRet = I2cReadBytes(gs_poI2cHandle, abyRedBuffer, (U8)IFX_DPS310_COEF_REG_ADDR, (U8)IFX_DPS310_COEF_LEN);
    if (IFX_DPS310_COEF_LEN != iRet)
    {
        return iRet;
    }

    fKp = gc_aiOsrSfConvert[IFX_DPS310_PRESSURE_OSR];
    fKt = gc_aiOsrSfConvert[IFX_DPS310_TEMPERATURE_OSR];

    dwTemp = abyRedBuffer[0];
    dwTemp <<= 4;
    dwTemp += (abyRedBuffer[1] >> 4);
    poCfg->fBeta0 = (I16)U32ToSType(dwTemp, TYPE_S12);
    poCfg->fBeta0 *= 0.5f;
    
    dwTemp = abyRedBuffer[1]&0xf;
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[2];
    poCfg->fBeta1 = (I16)U32ToSType(dwTemp, TYPE_S12);
    poCfg->fBeta1 /= fKt;

    dwTemp = abyRedBuffer[3];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[4];
    dwTemp <<= 4;
    dwTemp += ((abyRedBuffer[5] >> 4) & 0x0f);
    poCfg->iAlpha0 = U32ToSType(dwTemp, TYPE_S20);

    dwTemp = (abyRedBuffer[5] & 0x0F);
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[6];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[7];
    poCfg->fAlpha1 = U32ToSType(dwTemp, TYPE_S20);
    poCfg->fAlpha1 /= fKp;

    dwTemp = abyRedBuffer[8];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[9];
    poCfg->fAlpha4 = U32ToSType(dwTemp, TYPE_S16);
    poCfg->fAlpha4 /= fKt;

    dwTemp = abyRedBuffer[10];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[11];
    poCfg->fAlpha5 = U32ToSType(dwTemp, TYPE_S16);
    poCfg->fAlpha5 /= fKt;
    poCfg->fAlpha5 /= fKp;
    
    dwTemp = abyRedBuffer[12];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[13];
    poCfg->fAlpha2 = U32ToSType(dwTemp, TYPE_S16);
    poCfg->fAlpha2 /= fKp;
    poCfg->fAlpha2 /= fKp;
    
    dwTemp = abyRedBuffer[14];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[15];
    poCfg->fAlpha6 = U32ToSType(dwTemp, TYPE_S16);
    poCfg->fAlpha6 /= fKt;
    poCfg->fAlpha6 /= fKp;
    poCfg->fAlpha6 /= fKp;

    
    dwTemp = abyRedBuffer[16];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[17];
    poCfg->fAlpha3 = U32ToSType(dwTemp, TYPE_S16);
    poCfg->fAlpha3 /= fKp;
    poCfg->fAlpha3 /= fKp;
    poCfg->fAlpha3 /= fKp;
    return 0;
}

static int gs_iBaroInitSuccess = 0;

//F32 AirPressureToAltitude(F32 fAirPressure)  // has been tested fine
//{
//    return (44330.73f * (1 - (powf(fAirPressure * 0.00000986923f, 0.19261f))));
//}


int Dps310GetProcessedData(BARO_METER_DATA *poBaroMeterData)
{
    I32 iRet, iPressRaw, iTemperatureRaw;
    U8 abyRedBuffer[IFX_DPS310_PSR_TMP_READ_LEN] = { 0 };
    U32 dwTemp;

    if (0 == gs_iBaroInitSuccess)
    {
        return -1;
    }
    iRet = I2cReadBytes(gs_poI2cHandle, abyRedBuffer, IFX_DPS310_PSR_TMP_READ_REG_ADDR, IFX_DPS310_PSR_TMP_READ_LEN);
    if (iRet < IFX_DPS310_PSR_TMP_READ_LEN)
    {
        return -2;
    }
    dwTemp = abyRedBuffer[0];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[1];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[2];
    iPressRaw = U32ToSType(dwTemp, TYPE_S24);
    
    dwTemp = abyRedBuffer[3];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[4];
    dwTemp <<= 8;
    dwTemp += abyRedBuffer[5];
    iTemperatureRaw = U32ToSType(dwTemp, TYPE_S24);

    poBaroMeterData->m_fPress= gs_oDps310Cfg.iAlpha0
           + (gs_oDps310Cfg.fAlpha1 + (gs_oDps310Cfg.fAlpha2 + gs_oDps310Cfg.fAlpha3*iPressRaw)*iPressRaw)*iPressRaw
           + (gs_oDps310Cfg.fAlpha4 + (gs_oDps310Cfg.fAlpha5 + gs_oDps310Cfg.fAlpha6*iPressRaw)*iPressRaw)*iTemperatureRaw;
//    poBaroMeterData->m_fAltitude = AirPressureToAltitude(poBaroMeterData->m_fPress);
//    poBaroMeterData->m_fPress *= 0.01f;    //to convert it into mBar
    
    poBaroMeterData->m_fTempature = gs_oDps310Cfg.fBeta0 + gs_oDps310Cfg.fBeta1*iTemperatureRaw;

    return 0;
}

int Dps310Init(void)
{
    I32 iRet;
    U8 abyTemp[4];
    U8 byData;

    gs_iBaroInitSuccess = 0;
    gs_poI2cHandle = I2cOpen(BARO_I2C_IDX);
    if (NULL == gs_poI2cHandle)
    {
        return -1;
    }
    DelayMs(3);
    iRet = I2cReadBytes(gs_poI2cHandle, abyTemp, IFX_DPS310_PROD_REV_ID_REG_ADDR, 1);
    if (1 != iRet)
    {
        return -2;
    }
    if (IFX_DSPS310_PROD_REV_ID_VAL != abyTemp[0])
    {
        return -3;
    }
    DelayMs(40);
    iRet = s_Dps310ReadCalibCoeffs(&gs_oDps310Cfg);
    if (0 != iRet)
    {
        return -4;
    }


    /* Now apply ADC Temperature gain settings*/
    /* First write valid signature on 0x0e and 0x0f
         * to unlock address 0x62 */
    iRet = I2cWriteByte(gs_poI2cHandle, 0x0e, 0xa5);
    if (iRet < 0)
    {
        return -5;
    }
    iRet = I2cWriteByte(gs_poI2cHandle, 0x0f, 0x96);
    if (iRet < 0)
    {
        return -6;
    }
    
    /*Then update high gain value for Temperature*/
    iRet = I2cWriteByte(gs_poI2cHandle, 0x62, 0x02);
    if (iRet < 0)
    {
        return -7;
    }
    
    /*Finally lock back the location 0x62*/
    iRet = I2cWriteByte(gs_poI2cHandle, 0x0e, 0x00);
    if (iRet < 0)
    {
        return -8;
    }
    iRet = I2cWriteByte(gs_poI2cHandle, 0x0f, 0x00);
    if (iRet < 0)
    {
        return -9;
    }

    /* lets see which temperature diode is used for calibration and update state accordingly*/
    iRet = I2cReadBytes(gs_poI2cHandle, abyTemp, (U8)IFX_DPS310_TMP_COEF_SRCE_REG_ADDR, 1);
    if (iRet < 0)
    {
        return -10;
    }
    abyTemp[0] &= 0x80;
    iRet = I2cWriteByte(gs_poI2cHandle, IFX_DPS310_TMP_CFG_REG_ADDR, abyTemp[0]|IFX_DPS310_TEMPERATURE_MR|IFX_DPS310_TEMPERATURE_OSR);
    if (iRet < 0)
    {
        return -11;
    }

    iRet = I2cWriteByte(gs_poI2cHandle, IFX_DPS310_PRS_CFG_REG_ADDR, IFX_DPS310_PRESSURE_MR|IFX_DPS310_PRESSURE_OSR);
    if (iRet < 0)
    {
        return -12;
    }

    /* always take configuration word from state*/
    byData = 0;

    /*If oversampling rate for temperature is greater than 8 times, then set TMP_SHIFT bit in CFG_REG */
    if (IFX_DPS310_TEMPERATURE_OSR > OSR_8) 
    {
        byData |= (U8)IFX_DPS310_CFG_TMP_SHIFT_EN_SET_VAL;
    }

    /*If oversampling rate for pressure is greater than 8 times, then set P_SHIFT bit in CFG_REG */
    if (IFX_DPS310_PRESSURE_OSR > OSR_8) 
    {
        byData |= (U8)IFX_DPS310_CFG_PRS_SHIFT_EN_SET_VAL;
    }

    /* write CFG_REG */
    iRet = I2cWriteByte(gs_poI2cHandle, IFX_DPS310_CFG_REG_ADDR, byData);
    if (iRet < 0)
    {
        return -13;
    }

    /* activate sensor*/
    iRet = I2cWriteByte(gs_poI2cHandle, IFX_DPS310_MEAS_CFG_REG_ADDR, (U8)DPS310_MODE_BACKGROUND_ALL);
    if (iRet < 0)
    {
        return -14;
    }
    gs_iBaroInitSuccess = 1;
    return 0;
}


