#include <stdlib.h>
#include <math.h>  
#include "Variables.h"
#include "functions.h"
#include <stdio.h>

unsigned short Float2Short(float a, float b)
{
	double c;
	c = (a / b) * 32768;
	if (c >= 32767)
	{
		c = 32767;
	}
	else if (c <= -32768)
	{
		c = -32768;
	}
	return (unsigned short)c;

}

void EMMcSave(void)
{
	extern float PSI_out, h_baro;
	extern IMU_DATA _IMU_data_in;
	extern unsigned char IsCalGyroD;
	extern float AccBias[3];
	extern float phi_pre, the_pre, psi_pre;
	extern LogData g_oLog;  
	//unsigned short *EMMCData = &g_oLog.DataBuff[0];
	
	

	g_oLog.DataBuff[0] = 10;//(unsigned short)Count;
	g_oLog.DataBuff[1] = 11;//Float2Short(PSI, 3.15f);
	g_oLog.DataBuff[2] = 12;//Float2Short(PSI_out, 3.15f);
	g_oLog.DataBuff[3] = 13;//Float2Short(TETA, 3.15f);
	g_oLog.DataBuff[4] = 14;//Float2Short(GAMMA, 3.15f);
/*
	EMMCData[5] = Float2Short(dAB[0], 0.3f);
	EMMCData[6] = Float2Short(dAB[1], 0.3f);
	EMMCData[7] = Float2Short(dAB[2], 0.3f);
	EMMCData[8] = Float2Short(dWBt[0], 0.3f);
	EMMCData[9] = Float2Short(dWBt[1], 0.3f);
	EMMCData[10] = Float2Short(dWBt[2], 0.3f);

	EMMCData[11] = Float2Short(0, 20.0f);
	EMMCData[12] = Float2Short(0, 20.0f);
	EMMCData[13] = Float2Short(0, 20.0f);

	EMMCData[14] = (IsCalGyroD << 15) | (AttChanged << 14) | ((SJCRTL & 0xF) << 10) | (TakeoffOK << 9) | (IsTakeOff << 8) | (SwitchMode << 7) | ((IsAutoMode & 0x1F) << 2) | (FusionSource << 1) | FusionStart;
	EMMCData[15] = Float2Short(VRX, 30.0f);
	EMMCData[16] = Float2Short(VRY, 30.0f);
	EMMCData[17] = Float2Short(VRZ, 30.0f);
	EMMCData[18] = Float2Short(RX, 100.0f);
	EMMCData[19] = Float2Short(RY, 100.0f);
	EMMCData[20] = Float2Short(RZ, 100.0f);

	EMMCData[21] = Float2Short(Lift, 200.0f);
	EMMCData[22] = Float2Short(dABYc, 20.0f);
	EMMCData[23] = Float2Short(VRY_S, 30.0f);
	EMMCData[24] = Float2Short(RY_S, 100.0f);

	EMMCData[25] = Float2Short(VGPSX, 30.0f);
	EMMCData[26] = Float2Short(VGPSY, 30.0f);
	EMMCData[27] = Float2Short(VGPSZ, 30.0f);

	EMMCData[28] = Motor_OUT[0];
	EMMCData[29] = Motor_OUT[1];
	EMMCData[30] = Motor_OUT[2];
	EMMCData[31] = Motor_OUT[3];

	EMMCData[32] = pRCCommand[0];
	EMMCData[33] = Float2Short(VRX_S, 30.0f);
	EMMCData[34] = Float2Short(VRZ_S, 30.0f);

	EMMCData[35] = Float2Short(gamma_s, 3.15f);
	EMMCData[36] = Float2Short(psi_s, 3.15f);
	EMMCData[37] = Float2Short(teta_s, 3.15f);

	EMMCData[38] = Float2Short(Rate_OUTX, 20.0f);
	EMMCData[39] = Float2Short(Rate_OUTY, 20.0f);
	EMMCData[40] = Float2Short(Rate_OUTZ, 20.0f);

	EMMCData[41] = Float2Short(Att_OUTX, 20.0f);
	EMMCData[42] = Float2Short(Att_OUTY, 20.0f);
	EMMCData[43] = Float2Short(Att_OUTZ, 20.0f);

	EMMCData[44] = Float2Short(dWB[0], 3.0f);
	EMMCData[45] = Float2Short(dWB[1], 3.0f);
	EMMCData[46] = Float2Short(dWB[2], 3.0f);
	
	EMMCData[47] = Float2Short(phi_pre, 3.15f);
	EMMCData[48] = Float2Short(the_pre, 3.15f);
	EMMCData[49] = Float2Short(psi_pre, 3.15f);

	EMMCData[50] = Float2Short(h_baro, 10.0f);
	EMMCData[51] = Float2Short(0, 10.0f);
	EMMCData[52] = Float2Short(0, 10.0f);

	EMMCData[53] = (0 << 8) | (FirstLocated_P << 7) | (FirstLocated << 6) | ((GPSErrOK & 0x3) << 4) | (dv1 << 3) | (dr1 << 2) | (dv << 1) | dr;

	EMMCData[54] = Float2Short(RGPSX, 100.0f);
	EMMCData[55] = Float2Short(RGPSY, 100.0f);
	EMMCData[56] = Float2Short(RGPSZ, 100.0f);


	EMMCData[57] = Float2Short(MagM[0], 5.0f);
	EMMCData[58] = Float2Short(MagM[1], 5.0f);
	EMMCData[59] = Float2Short(MagM[2], 5.0f);

	EMMCData[60] = Float2Short(0, 5.0f);
	EMMCData[61] = Float2Short(0, 5.0f);
	EMMCData[62] = Float2Short(0, 5.0f);

	EMMCData[63] = (NumCorrect & 0xFF);

	EMMCData[64] = Float2Short(EPSX, 5.0f);
	EMMCData[65] = Float2Short(EPMY, 5.0f);
	EMMCData[66] = Float2Short(EPSZ, 5.0f);
	

	EMMCData[67] = Float2Short(AccBias[0], 5.0f);
	EMMCData[68] = Float2Short(AccBias[1], 5.0f);
	EMMCData[69] = Float2Short(AccBias[2], 5.0f);*/
	

//	PrintEMMC(EMMCData, sizeof(EMMCData[0]) * 128);
}

