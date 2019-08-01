#include "ImuCalibration.h"

#include "icm20600.h"
#include "systick.h"
#include "flash_driver.h"
#include "parameter.h"
#include "Bsp_i2c.h"
#include <math.h>
#include <string.h>


#define WHO_AM_I         0X75
#define PWR_MGMT_1       0x6b
#define SMPLRT_DIV       0x19
#define CONFIG           0x1a
#define GYRO_CONFIG     0x1b
#define ACCEL_CONFIG    0x1c
#define ACCEL_CONFIG2   0x1d 
#define ACCEL_XOUT_H    0x3b
#define TEMP_OUT_H      0x41
#define GYRO_XOUT_H     0x43

extern const int g_aiSensitivitScaleFactor[4];
extern PARAMETER_SAVE *g_poParameter;
extern int g_iImuInitSuccess;

extern int ReadBytes(U8* pbyBuffer, U8 byReadOffset, int iWantReadBytes);

__INLINE void s_ByteToShort(U8 *pbyDataIn,  I16 *psDataOut)
{
    *psDataOut = pbyDataIn[0];
    *psDataOut <<= 8;
    *psDataOut |= pbyDataIn[1];
}

static void s_BytesToShorts(U8 *pbyDataIn, int iLen,  I16 *psDataOut)
{
    int i, j;

    j = 0;
    for (i = 0; i < iLen; i += 2)
    {
        psDataOut[j] = pbyDataIn[i];
        psDataOut[j] <<= 8;
        psDataOut[j] |= pbyDataIn[i+1];
        j++;
    }
}


int ICM20600_CalibrationAndSaveParameter(int iCalibrateTimes)
{
    U8 abyBuff[14];
    I16 asRaw[7];
    I16 asAccMax[3] = {0,0,0};
    I16 asAccMin[3] = {0,0,0};
    I16 asAccTemp[3] = {0,0,0};
    I16 sTemp;
    I32 aiCount[7] = {0,0,0,0,0,0,0};
    int i, j, iRet;
    PARAMETER_SAVE *poParam;
    
    if (0 == g_iImuInitSuccess)
    {
        return -1;
    }

    if (iCalibrateTimes > CALIBRATION_TIMES_MAX)
    {
        iCalibrateTimes = CALIBRATION_TIMES_MAX;
    }
    if (iCalibrateTimes < CALIBRATION_TIMES_MIN)
    {
        iCalibrateTimes = CALIBRATION_TIMES_MIN;
    }
    
    sTemp = g_aiSensitivitScaleFactor[ACCEL_FS_SEL_DEFAULT]/10;
    for (i = 0; i < iCalibrateTimes; i++)
    {
        iRet = ReadBytes(abyBuff, ACCEL_XOUT_H, sizeof(abyBuff));
        if (sizeof(abyBuff) != iRet)
        {
            return -2;
        }
        s_BytesToShorts(abyBuff, sizeof(abyBuff), asRaw);
        for (j = 0; j < 7; j++)
        {
            aiCount[j] += asRaw[j];
            if (j < 3)
            {
                if (0 == i)
                {
                    asAccMin[j] = asRaw[j];
                    asAccMax[j] = asRaw[j];
                }
                else
                {
                    if (asAccMin[j] > asRaw[j])
                    {
                        asAccMin[j] = asRaw[j];
                    }
                    if (asAccMax[j] < asRaw[j])
                    {
                        asAccMax[j] = asRaw[j];
                    }
                    asAccTemp[j] = asAccMax[j] - asAccMin[j];
                    if (asAccTemp[j] > sTemp)
                    {
                        i = -1;
                    }
                }
            }
        }
    }
    
    poParam = LoadParameter();
    poParam->asAccelOffset[0] = (I16)(aiCount[0]/iCalibrateTimes);
    poParam->asAccelOffset[1] = (I16)(aiCount[1]/iCalibrateTimes);
    if (aiCount[2] > 0)
    {
         poParam->asAccelOffset[2] = (I16)(aiCount[2]/iCalibrateTimes - g_aiSensitivitScaleFactor[ACCEL_FS_SEL_DEFAULT]);
    }
    else
    {
         poParam->asAccelOffset[2] = (I16)(aiCount[2]/iCalibrateTimes + g_aiSensitivitScaleFactor[ACCEL_FS_SEL_DEFAULT]);
    }
    poParam->asGyroOffset[0]  = (I16)(aiCount[4]/iCalibrateTimes);
    poParam->asGyroOffset[1]  = (I16)(aiCount[5]/iCalibrateTimes);
    poParam->asGyroOffset[2]  = (I16)(aiCount[6]/iCalibrateTimes);
    SaveParameter();
    return 0;
}



