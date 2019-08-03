#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#include "ICM20600.h"
#include "Dps310.h"
#include "ST480.h"
#include "GpsNmea.h"

#define TAI ((float)0.012)
#define TAJ ((float)0.024)
#define DetaA ((float)0.024)
#define VelHor_MAX ((float)20)
#define VelVer_MAX ((float)10)
#define AccHor_MAX ((float)4.9) //9.788*sin(30)
#define AccVer_MAX ((float)4.9)
#define Rate_MAX ((float)2.62)
#define M        ((float)0.63)//drone quality
#define g        ((float)9.788)
#define Lx       ((float)0.8)//0.12//distance between 4and 3
#define Lz       ((float)0.4)//0.12//distance between 1and 2
#define q_t      ((float)0.67)//Yaw 

/*==============Attitude===============*/
extern float psi_s, gamma_s, teta_s;
extern float dpsi_s;
extern float PSI, GAMMA, TETA;
extern float dWB[3], dAB[3];
extern float Cbn[3][3], Cnb[3][3];
//PID parameters
extern float AttP[3], AttI[3], AttD[3];
extern float RateP[3], RateI[3], RateD[3];

//PID output
extern float Att_OUTX, Att_OUTY, Att_OUTZ;
extern float Rate_OUTX, Rate_OUTY, Rate_OUTZ;

/*==============Position==============*/
extern float RX_S, RY_S, RZ_S;
extern float RX, RY, RZ;
extern float VRX, VRY, VRZ;
extern float VGPSX, VGPSY, VGPSZ;
//PID parameters
extern float KRP[3], KRI[3], KRD[3];
extern float KVP[3], KVI[3], KVD[3];
//PID output
extern float PosHor_OUTX, PosHor_OUTZ, PosHor_IX, PosHor_IZ;
extern float VelHor_OUTX, VelHor_OUTZ, VelHor_IX, VelHor_IZ;
extern float PosVer_OUTY, PosVer_IY;
extern float VelVer_OUTY, VelVer_IY;

extern float VRX_S, VRY_S, VRZ_S;
extern unsigned char TCChanged, PitchChanged, RollChanged, YawChanged, YawChanged1, AttChanged;
/*===================Acc==================*/
//PID parameters
extern float AccP[3], AccI[3], AccD[3];

//PID output
extern float Acc_OUTX, Acc_OUTY, Acc_OUTZ;

//system
extern long Count;
extern unsigned char IsAutoMode, OneKeyLanding;
extern unsigned char SwitchMode, TakeoffOK;
extern unsigned char SwitchMode1, OptStart1, GPSStart1, FusionStart1, FusionSource1;
extern unsigned char OptStart, FusionStart, FusionSource;
extern unsigned char GSconnected, GSUnconnected;
extern float takeoff_time;
extern unsigned char RCConnected;
extern unsigned short pRCCommand[10];
//datafusion
extern float RX0, RY0, RZ0;
extern float VRXm[20], VRYm[20], VRZm[20], VRYm_h[20];
extern float VRX1, VRY1, VRZ1;
extern float RXm[20], RYm[20], RZm[20], RYm_h[20];
extern float RXP, RYP, RZP, RY_h_gps;
extern float VRXP, VRYP, VRZP;
extern float RXP_GPS, RYP_GPS, RZP_GPS, VRXP_GPS, VRYP_GPS, VRZP_GPS;
extern float VRXIncm[20], VRYIncm[20], VRZIncm[20];
extern float RXIncm[20], RYIncm[20], RZIncm[20];
extern float dWBtm[20][3], VN3X, VN3Y, VN3Z, VGPSX_ARM, VGPSY_ARM, VGPSZ_ARM;
extern float DISX, DISY, DISZ, DIST;
extern float DVSX, DVSY, DVSZ;
extern float DKSX, DKSY, DKSZ;
extern float DVPX, DVPY, DVPZ;
extern unsigned short NumCorrect, NumCorrect1;

/*======================GPS=============================*/
extern float CPU_GPS_SYNC, gps_time_ms_s;
extern float GPSTStamp, CPUSonar;
extern float fixMode, pdop, GPSSVs, SVs;
extern double Lat_GPS, Lon_GPS;
extern float VGPSX, VGPSY, VGPSZ;
extern unsigned char gps_ready;
extern float Alt;
extern unsigned char IsGPSOK, GPSStart;
extern float SonarTStamp;
extern unsigned char sg, sg1;
extern float RGPSX, RGPSY, RGPSZ, RGPSX_V, RGPSY_V, RGPSZ_V;
extern double Lati0, Loni0;
extern float H0;
extern double Pos0[3];
extern float mag_decl;
/*========================flow==========================*/
extern unsigned char sopt;
extern unsigned char IsOptOK, IsOptOK1, VO_Hflag;
extern float OptTStamp, OptTStamp1, OptTimeStamp;
extern float tmp11, dt_opt;
extern float OptT1, OptT0;
extern float PixelX, PixelZ, Q_opt, OptFlowDistX, OptFlowDistZ;
extern unsigned char OPTWorkingMode1, OPTWorkingMode;
extern float dx_opt1, dx_opt, dz_opt1, dz_opt;
extern float delta_opt;
extern float dPosX_opt1, dPosX_opt, dPosY_opt1, dPosY_opt, dPosZ_opt1, dPosZ_opt;

extern float V_Acc_0[3], Att_Gyro_0[3], Att_Gyro_04[3];
extern float MagnetTime;
extern unsigned short MagnetUp;
extern float EPSX, EPMY, EPSZ;
extern float dWBt[3];

//Mag
extern float MagM[3];

extern float AttQ[4];
extern float AttQ1[4];
extern float AttQ2[4];
extern float AttQ3[4];
extern float AttQ4[4];
extern float AttQ5[4];



extern unsigned char SwitchCount1;
extern float Lift_takeoffover, Lift;
extern unsigned char IsTakeOff;
extern float MotorCtrl[4];
extern unsigned short Motor_OUT[4];


extern float EPSX, EPMY, EPSZ;
extern float dWBt[3];

extern float dABX_filter, ddABX_filter;
extern float dABY_filter, ddABY_filter;
extern float dABZ_filter, ddABZ_filter;
extern float dABYC, dABY_Sum;
extern unsigned int dABY_Cnt, YawChangedCnt;
extern unsigned short TakeoffCnt;
extern unsigned short LandingCnt;
extern unsigned char OptDataSentCnt, GPSDataSentCnt, GPSDataSentCnt_BD930;
extern float LiftRecord[50], WYRecord[50], VRYRecord[50], Cbn11[50];
extern unsigned long LiftRecCnt;
extern float SumLift2, SumLiftVRY, SumVRY2, SumVRYWY, SumLiftWY;
extern float KLift, KVRY, WeightofCopter, KLFilter;
extern unsigned char Approaching, ApproachingCnt;
extern unsigned int downsonarbad_flag;
extern float Lift_takeoff;
extern unsigned char belowsonar, upsonar;
extern unsigned char FirstLanding, LandingToEarth;
extern unsigned int Count_Kal;

extern unsigned char IsCaliMode_D200;
extern unsigned char IMUNoData, MagNoData;
extern float radii[3], evecs[3][3], MCenter[3];
extern float Cbu[3][3], MagMMean;
extern float psi_spin, psi_spin_abs;
extern unsigned char SpinStart;
extern float var_hbn;
extern float MagDcln;
extern double g0;
extern unsigned char lockFlag;
extern unsigned char YawCtlMode, IsDELTCorrect, VOPTCorrect, CoptDelay, OPTData;
extern unsigned char IMUReady;
extern unsigned char AltReach;
extern float DOPTKX, DOPTKZ;
extern unsigned char ManualChoose;
extern unsigned char oiz_gps, oiz_gps1, oiz, oiz_os;
extern unsigned char Allign_Flag;


extern float MotorBalance[4];
extern float PSI_Declination, PSI_Decline;//PSIÆ«²î
extern float BalanceC;
extern unsigned char IsTakeOff1, IsLanding, ZeroFlag;
extern unsigned char LastAutoMode, AttClearZero, AttChanged1, YawAjust;
extern unsigned short Opt_Invalid_Cnt;
extern unsigned char HeadingFreeMode;
extern float VSIntgY;
extern unsigned char MotorIndex, MotorIndex1;
extern float EMMCEst;
extern unsigned char EMCCalibOK;
extern unsigned char WhichGPS, PosSource;
extern unsigned char GPSErrOK, OptErrOK;

extern float Magn[3];
extern unsigned char Fly_Flag;
extern float dAB1[3], dNB[3], dNB1[3];

extern unsigned char MagInteqCnt;
extern unsigned char YawBegin, EleBegin, RollBegin;
extern float YawCalibAngle, EleCalibAngle, RollCalibAngle;
extern float VRYm[20], VRYm_h[20], VRY_h;
extern float RXm[20], RYm[20], RZm[20], RYm_h[20], RY_h;
extern float AttQm[20][4];
extern float Tv, Tr;
extern float dQ1[3], dQ[3];
extern float DKSX, DKSY, DKSZ, DKSX_OS, DKSY_OS, DKSZ_OS;
extern unsigned char sn, dv, sof;
extern unsigned char dv_OS, oiz_os1, FirstLocated, FirstLocated_P;
extern float DIST, DIST_OS;
extern unsigned char SOB, BaroUpdate, IsCalibMode, MagCorrectStep, MagDataUpdate;
extern float VRY1, VRY1_h, RY1, RY1_h;
extern float RIntg[3];
extern float DTI;
extern unsigned char VIntgrFlag, VIntgrFlag1;
extern float RoptX, RoptZ, RX1, RZ1;
extern unsigned char FirstLocated_os, SJC;
extern float dVXg[10], dVYg[10], dVZg[10], dRXg[10], dRYg[10], dRZg[10];
extern unsigned char gm_acc_est, acc_est_status;
extern unsigned char DriftCorrected;
extern float MagMB[3];
extern unsigned char CloseGPS;
extern unsigned int IsMagChg;
extern unsigned char SpinComplete, TakeoffTest;
extern float PSI_MagChkStart;
extern unsigned char IMUErrCnt;
extern unsigned char SenSorChecking;

extern unsigned short LandingStopFlag;
extern unsigned long Count_Hbias;
extern unsigned char H0Set, MagDclnUpdate1;
extern unsigned char Vctol;
extern unsigned char SourceChangedCnt, SourceChanged;
extern unsigned char ps_heightkal[2];
extern float dVX_OPT[10], dVZ_OPT[10];
extern unsigned char oiz1, Count_Calib, SaveMagS, SaveMagSAzi, Count_OT_MagCL, EleIndex, YawIndex, RollIndex, Err_OT_MagCL, SaveMagCenter, SaveMagChanged;
extern unsigned char SJCPOS, LandingOK;
extern unsigned char RMSShootLimit;
extern unsigned char IsTakeoff1;
extern unsigned char StableCn;
extern unsigned int movecnt;
extern unsigned char IsVehicleMoving, IsCaliMode;
extern float MagDAzi[120][3];
extern float MagSAzi[120][3];
extern float dwbfilter[3];
extern float h_sonar, h_sonar1;
extern float RY0;
extern unsigned char MagDclnUpdated, MagDclnUpdated1;
extern unsigned char OriginUpdate;

extern double Cen[3][3], Pos0[3];
extern double Lati0, Loni0;
extern float H0;
extern unsigned char Baro_Judge_step, MagChkFailed;
extern unsigned char Baro_Judge_flag, Baro_Judge_ok;
extern float Baro_Judge_time, Baro_Judge_time1, Baro_Judge_time2, Baro_Judge_k;
extern unsigned char AttCalCnt;
extern unsigned int AttCount;
extern long ATTCnt;
extern float teta_sum, gamma_sum;
extern short EMCCnt;
extern unsigned char EMCCheckStart, IsMotorEMCMode, MotorIndexLast;
extern unsigned int EMCCheckOK, EMCCheckCnt;

//============================================================
extern unsigned int mSonarDelay;
extern float RYP_OS, VRXP_OS, VRYP_OS, VRZP_OS, RYP_Sonar, VRYP_Sonar;
extern unsigned long OFIntgCnt;
extern unsigned char WKmode;
extern unsigned char OptWKCnt;
extern float OFIntgX, OFIntgZ;
extern float S13X_m[5], S13X_Ave;
extern float S13Z_m[5], S13Z_Ave;
extern float dwx, dwz;
extern float PosXn_opt, PosZn_opt, PosYn_opt;
extern float PosXn_opt1, PosZn_opt1, PosYn_opt1;
extern float VoptYn;
extern float DTI_OS;
extern float VoptXb, VoptZb, VoptX, VoptZ, VoptX1, VoptZ1, VoptX2, VoptZ2;
extern float delta_opt;
extern float dVX_OS, dVZ_OS;
extern float averageX, averageZ;
extern float S13X, S13Z, S13X_, S13Z_, S13X_1, S13Z_1;
extern float VoptXm[10], VoptZm[10], dt_optm[10];
extern float dRoptX, dRoptZ;
extern unsigned short dHerrcnt;
extern float HGround;
extern float dt_sonar;
extern unsigned char sn_OS, Sonarok, IsSonarOK, Copt;
extern float VOSY, VOS3Y;
extern float dh_s, dh_s1, dh_s2;
extern float SonarTStamp1;
extern float S13Y, S13Y_;
extern float ROSX, ROSY, ROSZ;
extern float VOSX, VOSY, VOSZ;
extern float dRY_OS, dVY_OS;

extern short mBaroDelay;
extern float BaroTStamp, BaroTStamp1;
extern float RYP_Baro, VRYP_Baro, RY_h_Baro;
extern float BaroAltFilter;
extern float BaroVFilter;
extern unsigned char Approaching, ApproachingCnt;
extern unsigned char belowsonar;
extern float BaroAlt;
extern unsigned char RTntgrFlag;
extern unsigned char mOptDelay;
extern float Hbias;
extern unsigned long Count_Hbias;
extern float k_gain_GO[4];
extern float HP_OPT_save;
extern float RX_OPT, RZ_OPT;
extern float VRYP_hvo_Inc_p;
extern float CPUT12;
extern float dWBtp[3];
extern float AttQp[4];
extern float HP_OPT;
extern float VRXP_OPT, VRYP_OPT, VRZP_OPT;
extern float VRXP_OPTm, VRYP_OPTm, VRZP_OPTm;
extern float DISX_OS, DISY_OS, DISZ_OS;
extern float DVPX_OS, DVPY_OS, DVPZ_OS;
extern float Alt_OS;
extern float DVOX, DVOZ;
extern unsigned char MagDclnUpdated, MagDclnUpdated1;
extern unsigned char g0_write;
extern double Cen[3][3];
extern unsigned char OriginUpdate;
extern unsigned char Unlocated, Unlocated_Count;
extern unsigned char GPSData;
extern unsigned char NonFirst;
extern unsigned char VIntgrFlag, VIntgrFlag1, RIntgrFlag, RIntgrFlag1;
extern unsigned char dv, dr, dv1, dr1, dv_1, dr_1;
extern unsigned char PosSource1;
extern float dVX, dVY, dVZ;
extern float dRX, dRY, dRZ;
extern float dVXg[10], dVYg[10], dVZg[10];
extern float dRXg[10], dRYg[10], dRZg[10];
extern float VGPSAvgX, VGPSAvgY, VGPSAvgZ;
extern float VGPSRMSX, VGPSRMSY, VGPSRMSZ;
extern float RGPSAvgX, RGPSAvgY, RGPSAvgZ;
extern float RGPSRMSX, RGPSRMSY, RGPSRMSZ;
extern float VGRMSL, RGRMSL;
extern long Count_GPS;
extern unsigned char sn;
extern float DTI_GPS;
extern float RX_offset, RY_offset, RZ_offset;
extern float RX_offset1, RY_offset1, RZ_offset1;
extern float Alt_Used;
extern short QoptCnt;
extern float VRYP_GPS_Inc, VRXP_GPS_Inc, VRZP_GPS_Inc, VRYP_Baro_Inc, VRYP_Sonar_Inc;
extern float HP_Update;
extern float dABYc;
extern unsigned char SJCRTL;
extern unsigned char AltAjust, HorzAjust, flag_col;
extern unsigned char IsAutoMode1, FlyMode1;
extern unsigned char Pos_Hori_Vctrl;
extern float psi_s_pre;


typedef struct _SENSOR_RAW_DATA_ 
{
    IMU_DATA oImu;
    int m_iImuUpdateFlag; // 1:update, 0:not update yet
    
    sCompass oCompass;
    int m_iCompassUpdateFlag; // 1:update, 0:not update yet
    
    NMEA_INFO oGpsInfo;
    int m_iGpsUpdateFlag; // 1:update, 0:not update yet

    BARO_METER_DATA oBaroMeter;
    int m_iBaroMeterUpdateFlag; // 1:update, 0:not update yet

    float m_fBatteryVoltage;

    U16 m_awSbusData[16];
    int m_iSbusUpdateFlag; // 1:update, 0:not update yet
}SENSOR_RAW_DATA;

#pragma pack (4)
typedef struct _LogData_
{
    unsigned short DataBuff[128];
}LogData;
#pragma pack ()

typedef struct
{
	//volatile 
	float Input_Butter[3];
	//volatile 
	float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
	float a[3];
	float b[3];
}Butter_Parameter;


#endif
