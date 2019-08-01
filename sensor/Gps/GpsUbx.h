/**********************************************************************************************************
* Copy right      : Botan high tech.
*	Description     : GPS ubx protocol parse
*	File  Name      : GpsUbx.h

History:
Author               Date                  version
Ryan Huang         2019-04-03              V1.0
**********************************************************************************************************/

#ifndef _GPS_UBX_H_
#define _GPS_UBX_H_
// ubx protocol parse 

#define UBX_MAXSAT         30

#define UBX_SATINPACK      4

#ifndef U8
#define U8 unsigned char
#endif

#ifndef U16
#define U16 unsigned short
#endif


#ifndef U32
#define U32 unsigned int
#endif

#ifndef U64
#define U64 uint64_t
#endif

#ifndef I8
#define I8 char
#endif

#ifndef I16
#define I16 short
#endif


#ifndef I32
#define I32 int
#endif

 
#ifndef F32
#define F32 float
#endif
 
 
#ifndef F64
#define F64 double
#endif

#ifndef NULL
#define NULL 0
#endif


typedef struct _UBX_NAV_SVINFO_SATELLITE_INFO_
{
    U8 byChn;     // Channel number, 255 for SVs not assigned to a channel
    U8 bySvid;    // Satellite ID, 
    U8 byFlags;   // Bitmask
    U8 byQuality; // Bitfield 
    U8 byCno;     // Carrier to Noise Ratio (Signal Strength)---unit:dBHz
    I8 cElev;     // Elevation in integer degrees---unit:deg
    I16 sAzim;    // Azimuth in integer degrees---unit:deg
    I32 iPrRes;   // Pseudo range residual in centimetres---unit:cm
} UBX_NAV_SVINFO_SATELLITE_INFO; // len = 12


typedef struct _UBX_NAV_SVINFO_PAYLOAD_
{
    U32 dwITow;       // GPS time of week of the navigation epoch.
    U8 byNumCh;       // Number of channels
    U8 byGlobalFlags; // Bitmask
    U16 wRev2;
    UBX_NAV_SVINFO_SATELLITE_INFO oSateLlite[UBX_MAXSAT]; // 30*12
} UBX_NAV_SVINFO_PAYLOAD; // len = 8+12*N


typedef struct _UBX_NAV_PVT_PAYLOAD_
{
    U32 dwITow;       // GPS time of week of the navigation epoch.
    U16 wYear;        // Year (UTC)  --unit:year
    U8 byMonth;       // Month, range 1..12 (UTC)  --unit:month
    U8 byDay;         // Day of month, range 1..31 (UTC)  --unit:day
    U8 byHour;        // Hour of day, range 0..23 (UTC)  --unit:hour
    U8 byMin;         // Minute of hour, range 0..59 (UTC)  --unit:min
    U8 bySec;         // Seconds of minute, range 0..60 (UTC)  --unit:s
    U8 byValid;       // Validity Flags (see graphic below)
    U32 dwTAcc;       // Time accuracy estimate (UTC)  --unit:ns
    I32 iNano;        // Fraction of second, range -1e9 .. 1e9 (UTC)  --unit:ns
    U8 byFixType;     // GPSfix Type, range 0..5: 0x00 = No Fix;0x01 = Dead Reckoning only;0x02 = 2D-Fix;0x03 = 3D-Fix;0x04 = GPS + dead reckoning combined;0x05 = Time only fix;0x06..0xff: reserved
    U8 byFlags;       // Fix Status Flags (see graphic below)
    U8 byRev1;
    U8 byNumSv;       // Number of satellites used in Nav Solution
    I32 iLon;         // Longitude--unit:1e-7 deg
    I32 iLat;         // Latitude--unit:1e-7 deg
    I32 iHeight;      // Height above Ellipsoid--unit:mm
    I32 iHMsl;        // Height above mean sea level--unit:mm
    U32 dwHAcc;       // Horizontal Accuracy Estimate--unit:mm
    U32 dwVAcc;       // Vertical Accuracy Estimate--unit:mm
    I32 iVelN;        // NED north velocity--unit:mm/s
    I32 iVelE;        // NED east velocity--unit:mm/s
    I32 iVelD;        // NED down velocity--unit:mm/s
    I32 iGSpeed;      // Ground Speed (2-D)--unit:mm/s
    I32 iHeading;     // Heading of motion 2-D--unit:1e-5 deg
    U32 dwSAcc;       // Speed Accuracy Estimate--unit:mm/s
    U32 dwHeadingAcc; // Heading Accuracy Estimate--unit:1e-5 deg
    U16 wPDop;        // Position DOP--unit:0.01
    U16 wRev2;
    U32 adwRev3[3];
} UBX_NAV_PVT_PAYLOAD; // len = 84(document) / 92(test result)

typedef struct _UBX_NAV_DOP_PAYLOAD_
{
    U32 dwITow; // GPS time of week of the navigation epoch.
    U16 wGDop;  //  Geometric DOP--unit:0.01
    U16 wPDop;  //   Position DOP--unit:0.01
    U16 wTDop;  //       Time DOP--unit:0.01
    U16 wVDop;  //   Vertical DOP--unit:0.01
    U16 wHDop;  // Horizontal DOP--unit:0.01
    U16 wNDop;  //   Northing DOP--unit:0.01
    U16 wEDop;  //    Easting DOP--unit:0.01
} UBX_NAV_DOP_PAYLOAD; // len = 18


typedef struct _UBX_NAV_POSLLH_PAYLOAD_
{
    U32 dwITow; // GPS time of week of the navigation epoch.
    I32 iLon;  // Longitude
    I32 iLat;  // Latitude
    I32 iHeight;  // Height above ellipsoid
    I32 iHMSL;  // Height above mean sea level
    U32 dwHAcc;  // Horizontal accuracy estimate
    U32 dwVAcc;  // Vertical accuracy estimate
} UBX_NAV_POSLLH_PAYLOAD; // len = 28

typedef struct _UBX_NAV_SOL_PAYLOAD_
{
    U32 dwITow; // GPS time of week of the navigation epoch.
    I32 iFTow; // Fractional part of iTOW (range: +/-500000). The precise GPS time of week in seconds is: (iTOW * 1e-3) + (fTOW * 1e-9)
    I16 sWeek;  // GPS week number of the navigation epoch
    U8 byGpsFix;  // GPSfix Type, range 0..5: 0x00 = No Fix;0x01 = Dead Reckoning only;0x02 = 2D-Fix;0x03 = 3D-Fix;0x04 = GPS + dead reckoning combined;0x05 = Time only fix;0x06..0xff: reserved
    U8 byFlags;  // Fix Status Flags (see graphic below)
    I32 iEcefX;  // ECEF X coordinate
    I32 iEcefY;  // ECEF Y coordinate
    I32 iEcefZ;  // ECEF Z coordinate
    U32 dwPAcc;  // 3D Position Accuracy Estimate
    I32 iEcefVX;  // ECEF X velocity
    I32 iEcefVY;  // ECEF Y velocity
    I32 iEcefVZ;  // ECEF Z velocity
    U32 dwSAcc;  // Speed Accuracy Estimate
    U16 wPDop;  // Position DOP
    U8 byRev1;  // Reserved
    U8 byNumSV;  // Number of SVs used in Nav Solution
    U32 dwRev2;  // Reserved 2
} UBX_NAV_SOL_PAYLOAD; // len = 52

typedef struct _UBX_NAV_VELNED_PAYLOAD_
{
    U32 dwITow;       // GPS time of week of the navigation epoch.
    I32 iVelN;        // NED north velocity--unit:cm/s
    I32 iVelE;        // NED east velocity--unit:cm/s
    I32 iVelD;        // NED down velocity--unit:cm/s
    U32 dwSpeed;      // Speed (3-D)--unit:cm/s
    U32 dwGSpeed;      // Ground Speed (2-D)--unit:cm/s
    I32 iHeading;     // Heading of motion 2-D--unit:1e-5 deg
    U32 dwSAcc;       // Speed Accuracy Estimate--unit:cm/s
    U32 dwCAcc;       // Course / Heading accuracy estimate--unit:1e-5 deg
} UBX_NAV_VELNED_PAYLOAD; // len = 36

typedef struct _UBX_NAV_VELECEF_PAYLOAD_
{
    U32 dwITow;       // GPS time of week of the navigation epoch.
    I32 iEcefVX;        // ECEF X velocity--unit:cm/s
    I32 iEcefVY;        // ECEF Y velocity--unit:cm/s
    I32 iEcefVZ;        // ECEF Z velocity--unit:cm/s
    U32 dwSAcc;       // Speed Accuracy Estimate--unit:cm/s
} UBX_NAV_VELECEF_PAYLOAD; // len = 20

typedef struct _UBX_NAV_DGPS_SATELLITE_INFO_
{
    U8 bySvid;
    U8 byFlags;
    U16 wAgeC;
    F32 fPrc;
    F32 fPrrc;
} UBX_NAV_DGPS_SATELLITE_INFO; // len = 12

typedef struct _UBX_NAV_DGPS_PAYLOAD_
{
    U32 dwITow; // GPS time of week of the navigation epoch.
    I32 iITow;
    I16 sBaseId;
    I16 sBaseHealth;
    U8 byNumChn;
    U8 byStatus;
    U16 wRev1;
    UBX_NAV_DGPS_SATELLITE_INFO oSateLlite[16];
} UBX_NAV_DGPS_PAYLOAD; // len = 16+12*N

typedef struct _UBX_INFO_PAYLOAD_
{
    UBX_NAV_PVT_PAYLOAD oPvt;
    int iPvtFlag; // 1: data update, 0:date not update
    UBX_NAV_SVINFO_PAYLOAD oSvInfo;
    int iSvInfoFlag; // 1: data update, 0:date not update
    UBX_NAV_DOP_PAYLOAD oDop;
    int iDopFlag; // 1: data update, 0:date not update
    int iAllFlag;
    U64 qwTimeStampUs;
    U32 dwTimeStamp10Us;
} UBX_INFO_PAYLOAD;


typedef struct _UBX_NAV_ALL_
{
    union {
        UBX_NAV_PVT_PAYLOAD oPvt;
        UBX_NAV_SVINFO_PAYLOAD oSvInfo;
        UBX_NAV_DOP_PAYLOAD oDop;
    }oCmd;
} UBX_NAV_CMD_PAYLOAD;

typedef enum _UBX_NAV_CMD_ {
    UBX_NAV_CMD_PVT = 0,
    UBX_NAV_CMD_SVINFO,
    UBX_NAV_CMD_DOP,
    UBX_NAV_CMD_LAST, // NONE
}UBX_NAV_CMD;



int GpsInit(void);

int GpsGetParseUbxInfo(UBX_INFO_PAYLOAD *poUbxInfo);


#endif

