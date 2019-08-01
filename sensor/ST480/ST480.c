/*
*********************************************************************************************************
*
*    Module         Name : IIC Config
*    File           Name : BSP_IIC.c
*    Version Information : V1.0
*    Attention           : No
*    Amendant Record     : No
*    Author              : DeapSea.Team    
*    Data                : 2019/04/02 
*
*********************************************************************************************************
*/ 
#include "ST480.h"
#include "systick.h"
#include "math.h"
#include "..\..\driver\i2c\Bsp_i2c.h"


static I2cBaseDefine *gs_poI2cMagHandle = NULL;


static int gs_iMagInitSuccess = 0;

/*
*********************************************************************************************************
*    Funcname            :  ST480_Init
*    Function Declaration:  no
*    Formal Parameter    £º no
*    Returned Value      :  no
* Author              :  DeapSea.filk
*********************************************************************************************************
*/
int ST480_Init(void)
{
    U8 abyTemp[4];
    int iRet;

    gs_iMagInitSuccess = 0;
    gs_poI2cMagHandle = I2cOpen(MAG_I2C_IDX);
    if (NULL == gs_poI2cMagHandle)
    {
        return -1;
    }
    DelayMs(20);
    abyTemp[0] = 0x60; //the command of write register
    abyTemp[1] = 0x00;
    abyTemp[2] = 0x7c;
    abyTemp[3] = (0x00 << 2); //the addr of register   Note: Address[1:0] = 0x00
    iRet = I2cWriteBytesAndRead(gs_poI2cMagHandle, abyTemp, 4, abyTemp, 1);
    if (1 != iRet)
    {
        return -2;
    }
    
    abyTemp[0] = 0x60;//the command of write register
    abyTemp[1] = 0x00;
    abyTemp[2] = 0X18;
    abyTemp[3] = (0x02 << 2); //the addr of register   Note: Address[1:0] = 0x00
    iRet = I2cWriteBytesAndRead(gs_poI2cMagHandle, abyTemp, 4, abyTemp, 1);
    if (1 != iRet)
    {
        return -3;
    }
    
    iRet = I2cReadBytes(gs_poI2cMagHandle, abyTemp, 0x3e, 1);  //Single measure mode
    if (1 != iRet)
    {
        return -4;
    }
    gs_iMagInitSuccess = 1;
    DelayMs(30);
    return 0;
}

int ST480_GetData(sCompass *poDataOut)
{
    U8 abyTemp[8];
    I16 sTemp;
    int iRet;

    if (0 == gs_iMagInitSuccess)
    {
        return -1;
    }

    iRet = I2cReadBytes(gs_poI2cMagHandle, abyTemp, 0x4e, 7);  //Burst measure mode
    if (7 != iRet)
    {
        return -2;
    }
    if (!(abyTemp[0] & 0x10))
    {
        sTemp = abyTemp[1];
        sTemp <<= 8;
        sTemp += abyTemp[2];
        poDataOut->afAxisData[0] = sTemp;
        poDataOut->afAxisData[0] /= 667.0f;
        
        sTemp = abyTemp[3];
        sTemp <<= 8;
        sTemp += abyTemp[4];
        poDataOut->afAxisData[1] = sTemp;
        poDataOut->afAxisData[1] /= 667.0f;
        
        sTemp = abyTemp[5];
        sTemp <<= 8;
        sTemp += abyTemp[6];
        poDataOut->afAxisData[2] = sTemp;
        poDataOut->afAxisData[2] /= 400.0f;
#if 1
        iRet = I2cReadBytes(gs_poI2cMagHandle, abyTemp, 0x3e, 1);
        if (1 != iRet)
        {
            return -3;
        }
#endif
        return 0;
    }
    return -4;
}


