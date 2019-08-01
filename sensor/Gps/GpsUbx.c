/**********************************************************************************************************
* Copy right      : Botan high tech.
* Description     : GPS Ublox parse
* File  Name      : GpsUbx.c

History:
Author               Date                  version
Ryan Huang         2019-06-20              V1.0
**********************************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"
#include "GpsUbx.h"
#include "Variables.h"


#define GPS_BUFFER_LENGTH  1024

extern void DelayMs(U32 dwTimeMs);
extern void MyMemCpy(U8 *pbyDst, const U8*pbySrc, int iLen);
extern U64 GetTimeStampUs(void);
extern U32 GetTickCountMs(void);


static SerialPortHandle *g_pGpsPortHandle = NULL;
extern SerialPortHandle g_oUsart0;
static unsigned char gs_pbyGpsReadBuff[GPS_BUFFER_LENGTH];

int s_GpsInit(USART_PORT ePort, int iBauds)
{
    g_oUsart0.oUsartParam.m_dwBauds = iBauds;
    g_pGpsPortHandle = UsartOpen(ePort);
    if (NULL == g_pGpsPortHandle)
    {
        return -1; // open port failed
    }
    else
    {
        return 0; // open port success
    }
}


static U8 gs_byCheckSumA, gs_byCheckSumB;

void s_UbloxCheckSumInit(void)
{
    gs_byCheckSumA = 0;
    gs_byCheckSumB = 0;
}

void s_UbloxCheckSumUpdate(U8 byDataIn)
{
    gs_byCheckSumA += byDataIn;
    gs_byCheckSumB += gs_byCheckSumA;
}


/******************************************************************************
Function Name : UBX_NAV_CMD s_UbloxNavStateMachinePushOneByte(U8 byTemp, UBX_NAV_CMD_PAYLOAD *poPayLoadOut, U16 *pwPayloadLenOut)
Description   : parse gps ublox packet data receive state machine
Parameter     : 1. U8 byTemp[in]:data push the state machine
                      2. UBX_NAV_CMD_PAYLOAD *poPayLoadOut[out]: receive data output as structure.
Return        : UBX_NAV_CMD_PVT/UBX_NAV_CMD_SVINFO:there has one valid ublox packet, 
                   UBX_NAV_CMD_LAST: not valid packet
History :
Author              date                   version
Ryan Huang         2019-06-20               V1.0
******************************************************************************/
UBX_NAV_CMD s_UbloxNavStateMachinePushOneByte(U8 byTemp, UBX_NAV_CMD_PAYLOAD *poPayLoadOut, U16 *pwPayloadLenOut)
{
#define UBX_NAV_PACKET_MAX_LEN    440  //(DOP=26, PVT=100, SVINFO=16+12*N<=16+12*24 = 304. DOP+PVT+SVINFO<=430)
#define UBX_NAV_PACKET_SYNC1      0xb5
#define UBX_NAV_PACKET_SYNC2      0x62
#define UBX_NAV_PACKET_CLASS      0x01
#define UBX_NAV_PACKET_ID_DOP     0x04
#define UBX_NAV_PACKET_ID_PVT     0x07
#define UBX_NAV_PACKET_ID_SVINFO  0x30
#define UBX_NAV_PACKET_LEN_DOP    0x0012  // fixed
#define UBX_NAV_PACKET_LEN_PVT    0x005c  // fixed
#define UBX_NAV_PACKET_LEN_SVINFO 0x0c08  // vary length 8+12*N
    static U8 s_abyPacket[UBX_NAV_PACKET_MAX_LEN];
    static int s_iState = 0;
    static UBX_NAV_CMD s_eIdRecord = UBX_NAV_CMD_LAST;
    static U32 s_dwLastTimeMs = 0;
    static U16 s_wLen = 0, s_wCurLen = 0, s_wTemp;
    static U8 s_byCheckSumA;
    static U8 s_abyUbxNavPacketIdList[UBX_NAV_CMD_LAST] = {UBX_NAV_PACKET_ID_PVT, UBX_NAV_PACKET_ID_SVINFO, UBX_NAV_PACKET_ID_DOP};
    static U16 s_awUbxNavPacketLenList[UBX_NAV_CMD_LAST] = {UBX_NAV_PACKET_LEN_PVT, UBX_NAV_PACKET_LEN_SVINFO, UBX_NAV_PACKET_LEN_DOP};
    U32 dwCurTimeMs;
    int i;

    if (0 == s_dwLastTimeMs)
    {
        s_dwLastTimeMs = GetTickCountMs();
    }

    dwCurTimeMs = GetTickCountMs();
    if ((dwCurTimeMs - s_dwLastTimeMs) > 10)
    {
        s_iState = 0; // clear old data
    }
    s_dwLastTimeMs = dwCurTimeMs;

    switch (s_iState)
    {
    case 0:
        if (UBX_NAV_PACKET_SYNC1 == byTemp)
        {
            s_iState = 1;
        }
        return UBX_NAV_CMD_LAST;
    case 1:
        if (UBX_NAV_PACKET_SYNC2 == byTemp)
        {
            s_iState = 2;
        }
        else
        {
            s_iState = 0;
        }
        return UBX_NAV_CMD_LAST;
    case 2:
        if (UBX_NAV_PACKET_CLASS == byTemp)
        {
            s_UbloxCheckSumInit();
            s_UbloxCheckSumUpdate(byTemp);
            s_iState = 3;
        }
        else
        {
            s_iState = 0;
        }
        return UBX_NAV_CMD_LAST;
    case 3:
        for (i = 0; i < UBX_NAV_CMD_LAST; i++)
        {
            if (byTemp == s_abyUbxNavPacketIdList[i])
            {
                s_eIdRecord = (UBX_NAV_CMD)i;
                s_UbloxCheckSumUpdate(byTemp);
                s_iState = 4;
                return UBX_NAV_CMD_LAST;
            }
        }
        s_iState = 0;
        return UBX_NAV_CMD_LAST;
    case 4:
        s_UbloxCheckSumUpdate(byTemp);
        s_byCheckSumA = byTemp;
        s_iState = 5;
        return UBX_NAV_CMD_LAST;
    case 5:
        s_wLen = byTemp;
        s_wLen <<= 8;
        s_wLen += s_byCheckSumA;
        if (s_wLen == s_awUbxNavPacketLenList[s_eIdRecord])
        {
            s_UbloxCheckSumUpdate(byTemp);
            s_iState = 6;
            s_wCurLen = 0;
            return UBX_NAV_CMD_LAST;
        }
        else if (s_awUbxNavPacketLenList[s_eIdRecord] >= 0x100)
        {
            s_wTemp = s_wLen - (s_awUbxNavPacketLenList[s_eIdRecord] & 0xff);
            if (0 == (s_wTemp % (s_awUbxNavPacketLenList[s_eIdRecord] >> 8)))
            {
                s_UbloxCheckSumUpdate(byTemp);
                s_iState = 6;
                s_wCurLen = 0;
                return UBX_NAV_CMD_LAST;
            }
        }
        s_iState = 0;
        return UBX_NAV_CMD_LAST;
    case 6:
        s_UbloxCheckSumUpdate(byTemp);
        s_abyPacket[s_wCurLen] = byTemp;
        s_wCurLen++;
        if (s_wCurLen == s_wLen)
        {
            s_iState = 7;
        }
        return UBX_NAV_CMD_LAST;
    case 7:
        s_byCheckSumA = byTemp;
        s_iState = 8;
        return UBX_NAV_CMD_LAST;
    case 8:
        s_iState = 0;
        if ((s_byCheckSumA == gs_byCheckSumA) && (byTemp == gs_byCheckSumB))
        {
            MyMemCpy((U8*)poPayLoadOut, s_abyPacket, s_wLen);
            *pwPayloadLenOut = s_wLen;
            return s_eIdRecord;
        }
        return UBX_NAV_CMD_LAST;
    default:
        return UBX_NAV_CMD_LAST;
    }
}

U64 gs_qwTimeStampUs;
U32 gs_dwTimeStamp10Us;

void GpsSetTimeStampUs(void)
{
    gs_qwTimeStampUs = GetTimeStampUs();
    gs_dwTimeStamp10Us = GetTimeStamp10Us();
}

// 0:has update, 1:not update, other:failed
int GpsGetParseUbxInfo(UBX_INFO_PAYLOAD *poUbxInfo)
{
    UBX_NAV_CMD eRet;
    int iRet, i;
    UBX_NAV_CMD_PAYLOAD oPayLoad;
    U16 wLen;
#define SVINFO_FLAG_SET 0x01
#define DOP_FLAG_SET    0x02
#define PVT_FLAG_SET    0x04

    if (NULL == g_pGpsPortHandle)
    {
        return -1; // open port failed
    }
    else
    {
        iRet = UsartRecvNoBlocking(g_pGpsPortHandle, gs_pbyGpsReadBuff, GPS_BUFFER_LENGTH);
        if (iRet > 0)
        {
            for (i = 0; i < iRet; i++)
            {
                eRet = s_UbloxNavStateMachinePushOneByte(gs_pbyGpsReadBuff[i], &oPayLoad, &wLen);
                switch (eRet)
                {
                case UBX_NAV_CMD_SVINFO:
                    if (wLen < sizeof(UBX_NAV_SVINFO_PAYLOAD))
                    {
                        MyMemCpy((U8*)&poUbxInfo->oSvInfo, (U8*)&oPayLoad, wLen);
                    }
                    else
                    {
                        MyMemCpy((U8*)&poUbxInfo->oSvInfo, (U8*)&oPayLoad, sizeof(UBX_NAV_SVINFO_PAYLOAD));
                    }
                    poUbxInfo->iAllFlag |= SVINFO_FLAG_SET;
                    poUbxInfo->iSvInfoFlag= 1;
                    break;
                case UBX_NAV_CMD_PVT:
                    MyMemCpy((U8*)&poUbxInfo->oPvt, (U8*)&oPayLoad, sizeof(UBX_NAV_PVT_PAYLOAD));
                    poUbxInfo->iPvtFlag = 1;
                    poUbxInfo->iAllFlag |= PVT_FLAG_SET;
                    break;
                case UBX_NAV_CMD_DOP:
                    MyMemCpy((U8*)&poUbxInfo->oDop, (U8*)&oPayLoad, sizeof(UBX_NAV_DOP_PAYLOAD));
                    poUbxInfo->iDopFlag = 1;
                    poUbxInfo->iAllFlag |= DOP_FLAG_SET;
                    break;
                default:
                    break;
                }
            }
            if ((0 != (poUbxInfo->iAllFlag&PVT_FLAG_SET)) && (0 != (poUbxInfo->iAllFlag&SVINFO_FLAG_SET)))
            {
                poUbxInfo->qwTimeStampUs = gs_qwTimeStampUs;
                poUbxInfo->dwTimeStamp10Us = gs_dwTimeStamp10Us;
                poUbxInfo->iAllFlag = 0;
                return 0;
            }
        }
        return 1;
    }
}

#if 0
void GpsSendConfig(void)
{
    static U8 s_abyDisableNmeaSol[11]      = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x00, 0x11, 0x4e};
    static U8 s_abyDisableNmeaPosllh[11]   = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x00, 0x0d, 0x46};
    static U8 s_abyDisableNmeaVelned[11]   = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x00, 0x1d, 0x66};
    
    static U8 s_abyEnableUbxNavDop[11]     = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x01, 0x10, 0x4b};
    static U8 s_abyEnableUbxNavPvt[11]     = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
    static U8 s_abyEnableUbxNavSvInfo[11]  = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3};
    
    static U8 s_abySetReportRate10Hz[14]   = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 
                                       0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaSol, sizeof(s_abyDisableNmeaSol));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaPosllh, sizeof(s_abyDisableNmeaPosllh));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaVelned, sizeof(s_abyDisableNmeaVelned));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyEnableUbxNavDop, sizeof(s_abyEnableUbxNavDop));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyEnableUbxNavPvt, sizeof(s_abyEnableUbxNavPvt));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyEnableUbxNavSvInfo, sizeof(s_abyEnableUbxNavSvInfo));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abySetReportRate10Hz, sizeof(s_abySetReportRate10Hz));
    DelayMs(30);
}


// Other GPS module
int GpsInit(void)
{
    int iRet, iTryTimes;
    U32 dwStartTimeMs, dwCurTimeMs;
    static U8 s_abySetBauds115200bps[28]   = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 
                                       0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 
                                       0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 
                                       0x00, 0x00, 0xC0, 0x7E, 0xB5};
    UBX_INFO_PAYLOAD oPayLoad;
    
    for (iTryTimes = 0; iTryTimes < 3; iTryTimes++)
    {
        iRet = s_GpsInit(USART_PORT0, 38400);
        if (0 != iRet)
        {
            return -1;
        }
        UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abySetBauds115200bps, sizeof(s_abySetBauds115200bps));
        DelayMs(100);
        UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abySetBauds115200bps, sizeof(s_abySetBauds115200bps));
        DelayMs(100);
        iRet = s_GpsInit(USART_PORT0, 115200);
        if (0 != iRet)
        {
            return -2;
        }
        GpsSendConfig();
        UsartCleanReceiveBuffer(g_pGpsPortHandle);
        dwStartTimeMs = GetTickCountMs();
        while (1)
        {
            dwCurTimeMs = GetTickCountMs();
            if ((dwCurTimeMs - dwStartTimeMs) > 5000)
            {
                return -3;
            }
            iRet = GpsGetUbxInfo(&oPayLoad);
            if (0 == iRet)
            {
                 return 0;
            }
        }
    }
    return -4;
}

#else

void GpsSendConfig(void)
{
    static U8 s_abyDisableNmeaGNGGA[11]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F};
    static U8 s_abyDisableNmeaGNGLL[11]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
    static U8 s_abyDisableNmeaGNGSA[11]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
    static U8 s_abyDisableNmeaGLGSV[11]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    static U8 s_abyDisableNmeaGPGSV[11]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    static U8 s_abyDisableNmeaGNRMC[11]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
    static U8 s_abyDisableNmeaGNVTG[11]    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};
    static U8 s_abyEnableUbxNavDop[11]     = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x01, 0x10, 0x4b};
    static U8 s_abyEnableUbxNavPvt[11]     = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
    static U8 s_abyEnableUbxNavSvInfo[11]  = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3};
    static U8 s_abySetReportRate10Hz[14]   = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 
                                       0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaGNGGA, sizeof(s_abyDisableNmeaGNGGA));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaGNGLL, sizeof(s_abyDisableNmeaGNGLL));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaGNGSA, sizeof(s_abyDisableNmeaGNGSA));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaGLGSV, sizeof(s_abyDisableNmeaGLGSV));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaGPGSV, sizeof(s_abyDisableNmeaGPGSV));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaGNRMC, sizeof(s_abyDisableNmeaGNRMC));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyDisableNmeaGNVTG, sizeof(s_abyDisableNmeaGNVTG));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyEnableUbxNavDop, sizeof(s_abyEnableUbxNavDop));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyEnableUbxNavPvt, sizeof(s_abyEnableUbxNavPvt));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abyEnableUbxNavSvInfo, sizeof(s_abyEnableUbxNavSvInfo));
    DelayMs(30);
    UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abySetReportRate10Hz, sizeof(s_abySetReportRate10Hz));
    DelayMs(30);
}

// 0:success, other:failed
int GpsInit(void)
{
    int iRet, iTryTimes;
    U32 dwStartTimeMs, dwCurTimeMs;
    static U8 s_abySetBauds115200bps[28]   = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 
                                       0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 
                                       0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 
                                       0x00, 0x00, 0xC0, 0x7E};
    UBX_INFO_PAYLOAD oPayLoad;
    
    for (iTryTimes = 0; iTryTimes < 3; iTryTimes++)
    {
        iRet = s_GpsInit(USART_PORT0, 9600);
        if (0 != iRet)
        {
            return -1;
        }
        UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abySetBauds115200bps, sizeof(s_abySetBauds115200bps));
        DelayMs(100);
        UsartSendNoBlocking(g_pGpsPortHandle, (const U8*)s_abySetBauds115200bps, sizeof(s_abySetBauds115200bps));
        DelayMs(100);
        iRet = s_GpsInit(USART_PORT0, 115200);
        if (0 != iRet)
        {
            return -2;
        }
        GpsSendConfig();
        UsartCleanReceiveBuffer(g_pGpsPortHandle);
        dwStartTimeMs = GetTickCountMs();
        while (1)
        {
            dwCurTimeMs = GetTickCountMs();
            if ((dwCurTimeMs - dwStartTimeMs) > 5000)
            {
                return -3;
            }
            iRet = GpsGetUbxInfo(&oPayLoad);
            if (0 == iRet)
            {
                 return 0;
            }
        }
    }
    return -4;
}
#endif


