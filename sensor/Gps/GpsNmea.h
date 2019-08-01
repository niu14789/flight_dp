/**********************************************************************************************************
* Copy right      : Botan high tech.
*	Description     : GPS NMEA 4.0 parse
*	File  Name      : GpsNmea.h

History:
Author               Date                  version
Ryan Huang         2019-04-03              V1.0
**********************************************************************************************************/

#ifndef _GPS_NMEA4_H_
#define _GPS_NMEA4_H_
// nmea4.0 protocol parse 

#define NMEA_MAXSAT         70

#define NMEA_SATINPACK      4

#ifndef U8
#define U8 unsigned char
#endif

#ifndef U16
#define U16 unsigned short
#endif


#ifndef U32
#define U32 unsigned int
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

#ifndef NULL
#define NULL 0
#endif



//#define _GPS_DEBUG_VERSION_




// nmea4.0 protocol parse 

#define MAX_LINE_NUMBER   20

/**
* Information about satellite
* @see NMEA_SATINFO
* @see NMEA_GSV
*/
typedef struct _NMEA_SATELLITE_
{
    U32 m_dwID : 8;        /**< Satellite PRN number */
    U32 m_dwUseFlag : 1;   /**< Used in position fix 参与定位卫星标志，1参与，0未参与*/
    U32 m_dwElv : 7;       /**< Elevation in degrees, 90 maximum 升高度数，最大90度*/
    U32 m_dwSignal : 7;    /**< Signal, 00-99 dB */
    U32 m_dwAzimuth : 9;   /**< Azimuth, degrees from true north, 000 to 359 */
} NMEA_SATELLITE; // 4 bytes

/**
* Information about all satellites in view
* @see nmeaINFO
* @see nmeaGPGSV
*/
typedef struct _NMEA_SAT_INFO
{
    U16 m_wInuse;       // 参与计算方位和速度的卫星数
    U16 m_wVisible;     // 可见卫星数 
    NMEA_SATELLITE m_aoList[NMEA_MAXSAT]; /**可见卫星列表。*/
} NMEA_SAT_INFO; // 84 bytes

/**
* Date and time data
* @see nmea_time_now
*/
typedef struct _NMEA_TIME_
{
    U8 m_byYear;       /**< Years [0,99] */
    U8 m_byMon;        /**< Months since January - [1,12] */
    U8 m_byDay;        /**< Day of the month - [1,31] */
    U8 m_byHour;       /**< Hours since midnight - [0,23] */
    U8 m_byMin;        /**< Minutes after the hour - [0,59] */
    U8 m_bySec;        /**< Seconds after the minute - [0,59] */
    U8 m_byHsec;       /**< Hundredth part of second - [0,99] */
} NMEA_TIME; // 7 bytes

/**
* Summary GPS information from all parsed packets,
* used also for generating NMEA stream
* @see nmea_parse
* @see nmea_GPGGA2info,  nmea_...2info
*/
typedef struct _NMEA_INFO
{
    U32 m_dwMask;      //< Mask specifying types of packages from which data have been obtained 

    NMEA_TIME m_oUtcDateTime; // UTC日期时间

    U8 m_byPositioningQuality;  // 当前定位质量，0：定位不可用或无效，1：SPS定位模式，定位有效，6：估算模式（航位推算）
    U8 m_byPositionStateFlag;   // 定位状态标志，可取值1/2/3，初始值0 (1 = Fix not available; 2 = 2D; 3 = 3D) 

    I32 m_iPDOP;         // 位置精度因子。转换前取值范围0.000到99.999；转换后为32为无符号整数后且取值范围为0~99999，单位为“0.001”。
    I32 m_iHDOP;         // 垂直精度因子。转换前取值范围0.000到99.999；转换后为32为无符号整数后且取值范围为0~99999，单位为“0.001”。
    I32 m_iVDOP;         // 水平精度因子。转换前取值范围0.000到99.999；转换后为32为无符号整数后且取值范围为0~99999，单位为“0.001”。

    I32 m_iLatitude;     // latitude  纬度。转换前格式为 ddmm.mmmmmm，转换后为32位有符号整数mmmmmmmmm且单位为“0.00001分”。
    I32 m_iLongitude;    // longitude 经度。转换前格式为dddmm.mmmmmm，转换后为32位有符号整数mmmmmmmmm且单位为“0.00001分”。
    I32 m_iAltitude;     // Altitude海拔高度(原elv)，< Antenna altitude above/below mean sea level (geoid) in meters 海拔高度，即接收机天线相对于大地水准面的高度，单位为“厘米”
    I32 m_iSpeedKnot;    // 对地速度，单位为“0.001节” 
    I32 m_iSpeedMH;      // 対地速度，单位为“米/小时”。可使用iSpeedKnot*1.852得到
    I32 m_iDirection;    // < Track angle in degrees True 
    I32 m_iDeclination;  // < Magnetic variation degrees (Easterly var. subtracts from true course)
    I8 m_cMode;        // 定位模式标志 (A = 自主模式, D = 差分模式, E = 估算模式, N = 数据无效)
    NMEA_SAT_INFO m_oSatInfo; // 存放卫星信息列表，北斗卫星的ID编号会被加100再保存
} NMEA_INFO;


typedef struct _NMEA_SRING_LIST_
{
    I32 m_iLineNumber;
    I32 m_aiLineType[MAX_LINE_NUMBER];
    I32 m_aiSubType[MAX_LINE_NUMBER];
    I8 *strLineAddr[MAX_LINE_NUMBER];
} NMEA_SRING_LIST;



int GpsInit(void);

#ifdef _GPS_DEBUG_VERSION_
int GpsGetNmeaInfo(NMEA_INFO *poNmeaInfo, SerialPortBaseDefine *pDebugPort);
#else
// 0:success, 1:not update, other:failed
// Notice:call this API at lease one time per second.
int GpsGetNmeaInfo(NMEA_INFO *poNmeaInfo);
#endif


#endif

