/*!
    \file  main.c
    \brief USART DMA transmitter receiver
    
    \version 2017-02-10, V1.0.0, demo for GD32F30x
    \version 2018-10-10, V1.1.0, demo for GD32F30x
    \version 2018-12-25, V2.0.0, demo for GD32F30x
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "systick.h"
#include "BaseDefine.h"
#include "SerialPort.h"
#include <stdio.h>
//#include "BSP_TEST.h"
#include "ICM20600.h"
#include "Dps310.h"
#include "ST480.h"
#include "parameter.h"
#include "GpsUbx.h"
#include "pwm_driver.h"
#include "led.h"
#include "adc_driver.h"
#include "sbus_usart.h"
#include "functions.h"
#include "Variables.h"
//#include "cdc_acm_core.h"


#define SENSOR_DATA_HAS_UPDATE  1
#define SENSOR_DATA_NOT_UPDATE  0
//#define _PWM_DEBUG_ 0




PARAMETER_SAVE *g_poParameter = NULL;


//extern uint16_t Decode_Finish_Flag;
//extern uint16_t Channel_received_value[16];

void task_1ms(float dti)
{
}

void task_4ms(void)
{
}

void task_20ms(void)
{
    Count = 1;
}

void task_500ms(void)
{
}

#define PERFORMANCE_TEST



//float Motortmp0=1000.0f,Motortmp1=1000.0f,Motortmp2=1000.0f,Motortmp3=1000.0f;
U16 awMotorTest[4] = {1000,1000,1000,1000};

int g_iCanSendLogFlag = 0;  // 0:can not, 1:can send

static SerialPortBaseDefine *g_pLogPortHandle = NULL;

static int s_LogInit(void)
{
    g_pLogPortHandle = Uart4Open(460800);
    if (NULL == g_pLogPortHandle)
    {
        return -1; // open port failed
    }
    else
    {
        return 0; // open port success
    }
}

int SendLog(U8 *pbyData, int iLen)
{
    if (NULL != g_pLogPortHandle)
    {
        return Uart4SendNoBlocking(g_pLogPortHandle, pbyData, iLen);
    }
    return -1;
}

// [0]:Left Head, [1]:Right Head, [2]:Left Tail, [3]:Right Tail
void SetMotor(U16 *pwMotorValue)
{
    motors_set_ratio(MOTOR_M3, pwMotorValue[0]);//Right Head-->Left Head[0]
    motors_set_ratio(MOTOR_M1, pwMotorValue[1]);//Left Tail-->Right Head[1]
    motors_set_ratio(MOTOR_M2, pwMotorValue[2]);//Left Head-->Left Tail[2]
    motors_set_ratio(MOTOR_M4, pwMotorValue[3]);//Right Tail-->Right Tail[3]
}

void Usart0Test(void)
{
    SerialPortHandle *poHandle;
    int iRet;
    U8 abySend[28] = "    Send response data...\r\n";  // 27 bytes
    U8 abyReceive[256];
    U32 dwStartTimeMs, dwCurTimeMs;
    
    poHandle = UsartOpen(USART_PORT0);
    if (NULL != poHandle)
    {
        while (1)
        {
            abySend[0] = ' ';
            abySend[1] = ' ';
            abySend[2] = ' ';
            dwStartTimeMs = GetTickCountMs();
            iRet = UsartGetRecvBufferlen(poHandle);
            if (iRet > 0)
            {
                iRet = UsartRecvNoBlocking(poHandle, abyReceive, sizeof(abyReceive));
                if (iRet > 0)
                {
                    abySend[0] = '0' + (iRet/100)%10;
                    abySend[1] = '0' + (iRet/10)%10;
                    abySend[2] = '0' + (iRet)%10;
                }
            }
            UsartSendNoBlocking(poHandle, abySend, 27);
            while (SEND_FINISHED!=UsartGetSendStatus(poHandle))
            {
            }
            dwCurTimeMs = GetTickCountMs();
            while ((dwCurTimeMs-dwStartTimeMs) < 1000)
            {
                dwCurTimeMs = GetTickCountMs();
            }
        }
    }
}

void Uart3Test(void)
{
    SerialPortHandle *poHandle;
    int iRet;
    U8 abySend[28] = "    Send response data...\r\n";  // 27 bytes
    U8 abyReceive[256];
    U32 dwStartTimeMs, dwCurTimeMs;
    
    poHandle = UsartOpen(USART_PORT1);
    if (NULL != poHandle)
    {
        while (1)
        {
            abySend[0] = ' ';
            abySend[1] = ' ';
            abySend[2] = ' ';
            dwStartTimeMs = GetTickCountMs();
            iRet = UsartGetRecvBufferlen(poHandle);
            if (iRet > 0)
            {
                iRet = UsartRecvNoBlocking(poHandle, abyReceive, sizeof(abyReceive));
                if (iRet > 0)
                {
                    abySend[0] = '0' + (iRet/100)%10;
                    abySend[1] = '0' + (iRet/10)%10;
                    abySend[2] = '0' + (iRet)%10;
                }
            }
            UsartSendNoBlocking(poHandle, abySend, 27);
            while (SEND_FINISHED!=UsartGetSendStatus(poHandle))
            {
            }
            dwCurTimeMs = GetTickCountMs();
            while ((dwCurTimeMs-dwStartTimeMs) < 1000)
            {
                dwCurTimeMs = GetTickCountMs();
            }
        }
    }
}

int GpsStructTest(SENSOR_RAW_DATA *poSensor)
{
#if 0
    UBX_NAV_CMD_PAYLOAD oPayLoad;
    UBX_NAV_PVT_PAYLOAD oPvt = {39500,2019,7,3,9,19,54,3,12345678,7562,3,1,1,12,1132544400,235433300,
    79340,79340,150,150,1000,1200,3400,1700,231,400,32,45,16,{455,457,461}};
    U8 abyBuff[128];
    
    MyMemCpy(abyBuff, &oPvt, sizeof(UBX_NAV_PVT_PAYLOAD));
#endif
    return 0;
}

SENSOR_RAW_DATA oSensorData;

int gs_iGpsReceiveFinishFlag = 0;
void GpsParseCall(void)
{
    int iRet;
    iRet = GpsGetParseUbxInfo(&oSensorData.oGpsInfo);
    if (0 == iRet)
    {
         gs_iGpsReceiveFinishFlag = 1;
    }
}

int GpsGetUbxInfo(UBX_INFO_PAYLOAD *poUbxInfo)
{
    if (1 == gs_iGpsReceiveFinishFlag)
    {
        gs_iGpsReceiveFinishFlag = 0;
        return 0;
    }
    return 1;
}


LogData g_oLog;  
extern int ICM20600_CalibrationAndSaveParameter(int iCalibrateTimes);
/*extern usbd_core_handle_struct  usb_device_dev;

// usb send data, request iDataLen <= 64
void UsbSendData(U8 *pbyDataIn, int iDataLen)
{
    cdc_acm_data_send(&usb_device_dev, pbyDataIn, iDataLen);
}

void UsbSendCount(U32 dwCount)
{
    U8 abyTemp[4];
    
    abyTemp[0] = (U8)(dwCount>>24);
    abyTemp[1] = (U8)(dwCount>>16);
    abyTemp[2] = (U8)(dwCount>>8);
    abyTemp[3] = (U8)(dwCount);
    cdc_acm_data_send(&usb_device_dev, abyTemp, 4);
}
*/
float CPUT12;
long Count;
uint32_t cnt = 0;
U16 g_awGpsTestBuff[1024];
int g_iTestCount = 0;
int iStampErrorCount = -1;
int iPacketNumber = 0;
int main(void)
{
    U32 dwLastTimeMs, dwCurTimeMs, dwCycleCount;
    int i, iRet, iImuCalibrationCount = 0;// iMagCabrationCount = 0;
    float dt;
    long long _imu_data_timestamp, Nowtimestamp;
    static int s_iImuCalibrationFlag = 0;
    U64 qwStart, qwEnd;
    U32 dwGps,dwTemp,dwMax=0;
    float fTemp;

    float Acctmp, Gyrotmp;
    float Acc_K[3], Acc_bias[3];
#if 0  
    // for app address start at 0x08008000
    __enable_irq();
    __enable_fault_irq();
#endif

    /* configure systick */
    systick_config();

    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

    DelayMs(100);

//    Usart0Test();
//    Uart3Test();
//    GpsStructTest(&oSensorData);

    g_poParameter = LoadParameter();

    //    gs_iInitFlag = 1;

    for (i = 0; i < 128; i++)
    {
        g_oLog.DataBuff[i] = 0;
    }

    motor_init();
    led_init();
    adc_init();
#if 1
    SbusInit();
#else

    RF_Init(); 
    rf_link_init();  
    rf_binding();
    wifi_link_init();
#endif
    
//    usb_init();

    //    UsartInit(USART_PORT1); //filk add 2019/04/29

    iRet = s_LogInit();
    iRet = Dps310Init();
    iRet = ST480_Init();
    iRet = ICM20600_Init();

    oSensorData.m_iCompassUpdateFlag = SENSOR_DATA_NOT_UPDATE;
    oSensorData.m_iGpsUpdateFlag = SENSOR_DATA_NOT_UPDATE;
    oSensorData.m_iImuUpdateFlag = SENSOR_DATA_NOT_UPDATE;
    dwLastTimeMs = GetTickCountMs();
    //   dwBeginTime = dwLastTimeMs;
    dwCycleCount = 0;

    //灯及电机测试
    set_led_scram_flicker();
//    DelayMs(5000);
    iRet = GpsInit();

    CPUT12 = GetTimeStampUs();
    _imu_data_timestamp = CPUT12;
    Nowtimestamp = CPUT12;
    CPUT12 = CPUT12*1.0e-6f;
    while (1)
    {
        CPUT12 = GetTimeStampUs();
        CPUT12 = CPUT12*1.0e-6f;
        // Step 1.check GPS update with 10 Hz and call with 20Hz
        // Step 1.check GPS update with 10 Hz and call with 2Hz
        GpsParseCall();
        
        if (0 == (dwCycleCount % 100))
        {
///            UsbSendCount(dwCycleCount);
            iRet = GpsGetUbxInfo(&oSensorData.oGpsInfo);
            if (0 == iRet)
            {
#if 0
                dwGps = GetTimeStamp10Us();
                if (dwGps > oSensorData.oGpsInfo.dwTimeStamp10Us)
                {
                    dwTemp = dwGps - oSensorData.oGpsInfo.dwTimeStamp10Us;
                }
                g_awGpsTestBuff[g_iTestCount] = (U16)(dwTemp&0xffff);
                g_iTestCount++;
                g_iTestCount %= 1024;
                if (dwTemp > dwMax)
                {
                    dwMax = dwTemp;
                }
                CPUT12 = dwMax;
#else
                if (-1 == iStampErrorCount)
                {
                    dwGps = oSensorData.oGpsInfo.oPvt.dwITow;
                    iStampErrorCount = 0;
                }
                else
                {
                    if ((oSensorData.oGpsInfo.oPvt.dwITow - dwGps) > 120)
                    {
                        iStampErrorCount++;
                    }
                    iPacketNumber++;
                    dwGps = oSensorData.oGpsInfo.oPvt.dwITow;
                CPUT12 = iStampErrorCount;
                }
#endif
                oSensorData.m_iGpsUpdateFlag = SENSOR_DATA_HAS_UPDATE;
            }
        }

#if 1 
        // For Send Log test
        if (0 == (dwCycleCount % 2)) // 
        {
            //            SendLog("Hello!", 6);

        }
#endif

        // Step 2.check app data from WIFI
//        wifi_link_data_receive();
//        wifi_link_data_send();


        // Step 3.check 2.4G Hz module receive control from remote control

#if 1
        // Step 4.check data update from compass ST480 with 100Hz
        if (0 == (dwCycleCount % 30)) // read data cost 336us
        {
            iRet = ST480_GetData(&oSensorData.oCompass);
            if (0 == iRet)
            {
                oSensorData.m_iCompassUpdateFlag = SENSOR_DATA_HAS_UPDATE;
            }
        }
#endif

#if 1
        // Step 5.check data update from baro meter DPS310
        if (0 == (dwCycleCount % 50)) // read data cost 214us
        {
            iRet = Dps310GetProcessedData(&oSensorData.oBaroMeter);
            if (0 == iRet)
            {
                oSensorData.m_iBaroMeterUpdateFlag = SENSOR_DATA_HAS_UPDATE;
            }
        }
#endif

#if 1
        // Step 6.check data update from IMU ICM20600 with 333Hz
        if (0 == (dwCycleCount % 2))
        {
 //           qwStart = GetTimeStampUs();
            iRet = ICM20600_GetData(&oSensorData.oImu);  // I2C with 400Kbps costed 397us and SPI with 7.5Mbps costed 38 us
 //           qwEnd = GetTimeStampUs();
//            Acc_K[0] = qwEnd;
//            Acc_K[1] = qwStart;
            if (0 == iRet)
            {
                oSensorData.m_iImuUpdateFlag = SENSOR_DATA_HAS_UPDATE;
    /*            Acctmp = oSensorData.oImu.m_afAccel[1];
                oSensorData.oImu.m_afAccel[0] = -oSensorData.oImu.m_afAccel[0];
                oSensorData.oImu.m_afAccel[1] = oSensorData.oImu.m_afAccel[2];
                oSensorData.oImu.m_afAccel[2] = Acctmp;
                Acc_K[0] = 1.0008f;
                Acc_K[1] = 0.9882f;
                Acc_K[2] = 0.9988f;
                Acc_bias[0] = 0.12f;
                Acc_bias[1] = 0.975f;
                Acc_bias[2] = 0.03f;
                oSensorData.oImu.m_afAccel[0] = Acc_K[0] * oSensorData.oImu.m_afAccel[0] + Acc_bias[0];
                oSensorData.oImu.m_afAccel[1] = Acc_K[1] * oSensorData.oImu.m_afAccel[1] + Acc_bias[1];
                oSensorData.oImu.m_afAccel[2] = Acc_K[2] * oSensorData.oImu.m_afAccel[2] + Acc_bias[2];
                
                fTemp = oSensorData.oImu.m_afAccel[0]*oSensorData.oImu.m_afAccel[0]+oSensorData.oImu.m_afAccel[1]*oSensorData.oImu.m_afAccel[1]+oSensorData.oImu.m_afAccel[2]*oSensorData.oImu.m_afAccel[2];
                dt = fTemp;
                oSensorData.oImu.m_afGyro[1] = fTemp;
                Gyrotmp = oSensorData.oImu.m_afGyro[1];
                oSensorData.oImu.m_afGyro[0] = -oSensorData.oImu.m_afGyro[0];
                oSensorData.oImu.m_afGyro[1] = oSensorData.oImu.m_afGyro[2];
                oSensorData.oImu.m_afGyro[2] = Gyrotmp;
*/
                Nowtimestamp = GetTimeStampUs();
                dt = (float)(Nowtimestamp - _imu_data_timestamp)*1.0e-6f;
                if (dwCycleCount < 5)
                {
                    dt = 0.002f;
                }
                task_1ms(dt);
                _imu_data_timestamp = GetTimeStampUs();
            }
        }
#endif



        if (0 == (dwCycleCount % 100))
        {
            // Check battery voltage  
            oSensorData.m_fBatteryVoltage = get_battery_voltage();

            //LED 闪烁功能
            led_flicker();
        }

#if 1
        // sbus receive check
        oSensorData.m_iSbusUpdateFlag = SbusReceiveData(oSensorData.m_awSbusData); // update cycle is 7ms/14ms.
#else
        rf_link_function();        
#endif
        
        // Step 7.call fusion algorithm function


        // Step 8.call fly control proccessing alorithm function

        if (((dwCycleCount % 4) == 0))
        {
 //           qwStart = GetTimeStampUs();
            task_4ms(); // 11us
 //           qwEnd = GetTimeStampUs();
 //           Acc_K[0] = qwEnd;
//            Acc_K[1] = qwStart;
//            Acc_K[2] = Acc_K[0] + Acc_K[1];
        }
        if (((dwCycleCount % 500) == 0))
        {
            qwStart = GetTimeStampUs();
            task_500ms();
            qwEnd = GetTimeStampUs();
            Acc_K[0] = qwEnd;
            Acc_K[0] = qwStart;
            Acc_K[1] = qwEnd - Acc_K[0];
        }
        if (((dwCycleCount % 12) == 0))
        {
            qwStart = GetTimeStampUs();
            task_20ms();
            qwEnd = GetTimeStampUs();
            Acc_K[0] = qwEnd;
            Acc_K[0] = qwStart;
            Acc_K[2] = qwEnd - Acc_K[0];
            if (Count > 0)
            {
                EMMcSave();
                SendLog((uint8_t *)&g_oLog, sizeof(g_oLog));
            }
        }



        // Step 9.pwm out set, please call the APIs void motors_set_ratio(motor_type_enum motor,uint32_t pulse) to set PWM output
#if _PWM_DEBUG_
        SetMotor(awMotorTest);
#endif


        // Step 10. call calibratione funtion if the calibration option was actived.
        if ((0 == s_iImuCalibrationFlag) && (1 == oSensorData.m_iSbusUpdateFlag))
        {
            if (oSensorData.m_awSbusData[13] > 1500)
            {
                iImuCalibrationCount++;
                if (iImuCalibrationCount >= 20)
                {
                    ICM20600_CalibrationAndSaveParameter(1000);
                    iImuCalibrationCount = 0;
                    oSensorData.m_iSbusUpdateFlag = 0;
                    s_iImuCalibrationFlag = 1; // permit call one time
                }
            }
            else
            {
                iImuCalibrationCount = 0;
            }
        }


        dwCurTimeMs = GetTickCountMs();
        while (dwCurTimeMs == dwLastTimeMs) // wait until one cycle run at least 1ms. 
        {
            dwCurTimeMs = GetTickCountMs();
        }
        dwLastTimeMs = GetTickCountMs();

        dwCycleCount++;
    }
}




