/*********************************************************************************************************
*
*    模 块 名: IIC Config
*    文 件 名: ICM20600.c
*    版本信息: V1.0
*    注    意：No
*    修改记录：No
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-28               V1.0
*********************************************************************************************************/ 


#define IMU_DEBUG_VERSION

#define IMU_USED_SPI


#include "ICM20600.h"
#include "systick.h"
#include "flash_driver.h"
#include "parameter.h"
#include "..\..\driver\i2c\Bsp_i2c.h"
#include "..\..\driver\spi\Bsp_spi.h"

const int g_aiSensitivitScaleFactor[4] = {16384,8192,4096,2048};
const float g_afGyroScaleFactor[4] = {131.072f, 65.536f, 32.768f, 16.384f};

#define ACC_DLPF_2181_235     0x01
#define ACC_DLPF_990_1213     0x02
#define ACC_DLPF_448_615      0x03
#define ACC_DLPF_212_310      0x04
#define ACC_DLPF_102_155      0x05
#define ACC_DLPF_51_78        0x06
#define ACC_DLPF_4200_4416    0x07
#define ACC_DLPF_10460_11000  0x08



extern PARAMETER_SAVE *g_poParameter;

int g_iImuInitSuccess = 0;

static float g_fAccFactor = 0;
static float g_fGyroFactor = 0;

//extern int gs_iInitFlag;  // 0:not init, 1: has init


#ifdef IMU_USED_SPI
    #define IMU_SPI SPI1
#endif

#ifdef IMU_USED_SPI
SPIBaseDefine *g_poSpiImuHandle = NULL;
#else
I2cBaseDefine *g_poI2cImuHandle = NULL;
#endif

int OpenDevice(void)
{
#ifdef IMU_USED_SPI
    g_poSpiImuHandle = SpiOpen(IMU_SPI_IDX);
    if (NULL == g_poSpiImuHandle)
    {
        return -1;
    }
#else
    g_poI2cImuHandle = I2cOpen(IMU_I2C_IDX);
    if (NULL == g_poI2cImuHandle)
    {
        return -1;
    }
#endif
    return 0;
}

int WriteByte(U8 byOffsetAddr, U8 byData)
{
#ifdef IMU_USED_SPI
    return SpiWriteDataBlock(g_poSpiImuHandle, byOffsetAddr, &byData, 1);
#else
    return I2cWriteByte(g_poI2cImuHandle, byOffsetAddr, byData);
#endif
}

int WriteBytes(U8 byOffsetAddr, U8* pbyBuffer, int iWriteLen)
{
#ifdef IMU_USED_SPI
    return SpiWriteDataBlock(g_poSpiImuHandle, byOffsetAddr, pbyBuffer, iWriteLen);
#else
    return I2cWriteBytes(g_poI2cImuHandle, pbyBuffer, iWriteLen, byOffsetAddr);
#endif
}

int ReadBytes(U8* pbyBuffer, U8 byReadOffset, int iWantReadBytes)
{
#ifdef IMU_USED_SPI
    return SpiReadDataBlock(g_poSpiImuHandle, byReadOffset, pbyBuffer, iWantReadBytes);
#else
    return I2cReadBytes(g_poI2cImuHandle, pbyBuffer, byReadOffset, iWantReadBytes);
#endif
}

/**********************************************************************************************************
*    函数  名： ICM20600_Init
*    功能描述:  ICM20600初始化
*    形    参： 无
*    返 回 值： 无
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-28               V1.0
**********************************************************************************************************/
int ICM20600_Init(void)
{
    int iRet;
#if 1
    U8 abyTemp[4];
#endif

    g_iImuInitSuccess = 0;
    iRet = OpenDevice();
    if (0 != iRet)
    {
        return -1;
    }

    DelayMs(20);
#if 1
    iRet = ReadBytes(abyTemp, WHO_AM_I, 1);
    if (1 != iRet)
    {
        return -2;
    }
    if (0x11 == abyTemp[0]) // Check if ID is ICM20600 or not.
    {
        return -3;
    }
#endif

//    iRet = WriteByte(PWR_MGMT_1, 0x80);//复位 注：此处不能写0x80,原因未知
    iRet = WriteByte(PWR_MGMT_1, 0x00);//复位
    if (iRet < 0)
    {
        return -4;
    }
    DelayMs(20);

    //disables the temperature sensor + Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    iRet = WriteByte(PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ|IMU_ENABLE_TEMPERATURE);// 0x03:enable the tempature, 0x0b:disable the tempature
    if (iRet < 0)
    {
        return -5;
    }
    DelayMs(20);
    
    iRet = WriteByte(SMPLRT_DIV, ICM20600_SAMPLE_RATE_1000HZ);//复位
    if (iRet < 0)
    {
        return -6;
    }
    DelayMs(20);

    iRet = WriteByte(CONFIG, ACC_DLPF_990_1213);//gyro dlpf
    if (iRet < 0)
    {
        return -7;
    }
    DelayMs(20);

    iRet = WriteByte(GYRO_CONFIG, (GYRO_FS_SEL_DEFAULT<<3));// ±1000 dps
    if (iRet < 0)
    {
        return -8;
    }
    DelayMs(20);

    iRet = WriteByte(ACCEL_CONFIG, ACCEL_FS_SEL_DEFAULT<<3);
    if (iRet < 0)
    {
        return -9;
    }
    DelayMs(20);

    iRet = WriteByte(ACCEL_CONFIG2, 0x06);//acc dlpf
    if (iRet < 0)
    {
        return -10;
    }
    DelayMs(20);

    g_iImuInitSuccess = 1;
    g_fAccFactor = GAL_VALUE / g_aiSensitivitScaleFactor[ACCEL_FS_SEL_DEFAULT];
    g_fGyroFactor = 1 / (DEGTORAD * g_afGyroScaleFactor[GYRO_FS_SEL_DEFAULT]);  // convert to radian factor

    return 0;
}

// please disable C99 compile option 
void s_ByteToShort(U8 *pbyDataIn,  I16 *psDataOut)
{
    *psDataOut = pbyDataIn[0];
    *psDataOut <<= 8;
    *psDataOut |= pbyDataIn[1];
}


/******************************************************************************
Function Name : static int s_ICM20600_ReadDataOut(PARAMETER_SAVE *poParameterIn, IMU_DATA *poImuData)
Description   : receive/read data from IMU
Parameter     : 1.poParameterIn[in]:usart device handle
                     2.poImuData[out]: Imu data out
Return        : -2 : read failed
                  0 : read success
History :
Author                               date                   version
Ryan Huang & Filk Lee        2019-05-28               V1.0
******************************************************************************/
static int s_ICM20600_ReadDataOut(PARAMETER_SAVE *poParameterIn, IMU_DATA *poImuData)
{
    U8 abyTemp[14];
    I16 sTemp;
    int iRet;
    
    iRet = ReadBytes(abyTemp, ACCEL_XOUT_H, sizeof(abyTemp));
    if (sizeof(abyTemp) != iRet)
    {
        return -2;
    }

    s_ByteToShort(abyTemp, &sTemp);
    poImuData->m_afAccel[0] = sTemp;
    poImuData->m_afAccel[0] -= poParameterIn->asAccelOffset[0];
    poImuData->m_afAccel[0] *= g_fAccFactor;
    
    s_ByteToShort(&abyTemp[2], &sTemp);
    poImuData->m_afAccel[1] = sTemp;
    poImuData->m_afAccel[1] -= poParameterIn->asAccelOffset[1];
    poImuData->m_afAccel[1] *= g_fAccFactor;
    
    s_ByteToShort(&abyTemp[4], &sTemp);
    poImuData->m_afAccel[2] = sTemp;
    poImuData->m_afAccel[2] -= poParameterIn->asAccelOffset[2];
    poImuData->m_afAccel[2] *= g_fAccFactor;

    // Tempature = (temp/280) + (13200/280) - 13 = (temp/280) + 47.14286f -13 = (temp/280) + 34.14286f;
    s_ByteToShort(&abyTemp[6], &sTemp);
    poImuData->m_fTempature = (float)sTemp;
    poImuData->m_fTempature /= 280.0f;
    poImuData->m_fTempature += 34.14286f;
    
    s_ByteToShort(&abyTemp[8], &sTemp);
    poImuData->m_afGyro[0] = sTemp;
    poImuData->m_afGyro[0] -= poParameterIn->asAccelOffset[0];
    poImuData->m_afGyro[0] *= g_fGyroFactor; // convert to radian
    
    s_ByteToShort(&abyTemp[10], &sTemp);
    poImuData->m_afGyro[1] = sTemp;
    poImuData->m_afGyro[1] -= poParameterIn->asAccelOffset[1];
    poImuData->m_afGyro[1] *= g_fGyroFactor;
    
    s_ByteToShort(&abyTemp[12], &sTemp);
    poImuData->m_afGyro[2] = sTemp;
    poImuData->m_afGyro[2] -= poParameterIn->asAccelOffset[2];
    poImuData->m_afGyro[2] *= g_fGyroFactor;
    return 0;
}

/**********************************************************************************************************
*    函 数 名: ICM20600_GetData
*    功能说明: 读取 ICM20600 加速度，角速度以及温度
*    形    参: 无
*    返 回 值: 0:success, -1:input parameter error
**********************************************************************************************************/
int ICM20600_GetData(IMU_DATA *poImuData)
{
    if (0 == g_iImuInitSuccess)
    {
        return -1;
    }
#ifdef IMU_DEBUG_VERSION
    if (NULL == poImuData)
    {
        return -2;
    }
    if (NULL == g_poParameter)
    {
        ICM20600_Init();
        if (NULL == g_poParameter)
        {
            return -3;
        }
    }
#endif

    return s_ICM20600_ReadDataOut(g_poParameter, poImuData);
}









