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
    U32 m_dwUseFlag : 1;   /**< Used in position fix ���붨λ���Ǳ�־��1���룬0δ����*/
    U32 m_dwElv : 7;       /**< Elevation in degrees, 90 maximum ���߶��������90��*/
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
    U16 m_wInuse;       // ������㷽λ���ٶȵ�������
    U16 m_wVisible;     // �ɼ������� 
    NMEA_SATELLITE m_aoList[NMEA_MAXSAT]; /**�ɼ������б�*/
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

    NMEA_TIME m_oUtcDateTime; // UTC����ʱ��

    U8 m_byPositioningQuality;  // ��ǰ��λ������0����λ�����û���Ч��1��SPS��λģʽ����λ��Ч��6������ģʽ����λ���㣩
    U8 m_byPositionStateFlag;   // ��λ״̬��־����ȡֵ1/2/3����ʼֵ0 (1 = Fix not available; 2 = 2D; 3 = 3D) 

    I32 m_iPDOP;         // λ�þ������ӡ�ת��ǰȡֵ��Χ0.000��99.999��ת����Ϊ32Ϊ�޷�����������ȡֵ��ΧΪ0~99999����λΪ��0.001����
    I32 m_iHDOP;         // ��ֱ�������ӡ�ת��ǰȡֵ��Χ0.000��99.999��ת����Ϊ32Ϊ�޷�����������ȡֵ��ΧΪ0~99999����λΪ��0.001����
    I32 m_iVDOP;         // ˮƽ�������ӡ�ת��ǰȡֵ��Χ0.000��99.999��ת����Ϊ32Ϊ�޷�����������ȡֵ��ΧΪ0~99999����λΪ��0.001����

    I32 m_iLatitude;     // latitude  γ�ȡ�ת��ǰ��ʽΪ ddmm.mmmmmm��ת����Ϊ32λ�з�������mmmmmmmmm�ҵ�λΪ��0.00001�֡���
    I32 m_iLongitude;    // longitude ���ȡ�ת��ǰ��ʽΪdddmm.mmmmmm��ת����Ϊ32λ�з�������mmmmmmmmm�ҵ�λΪ��0.00001�֡���
    I32 m_iAltitude;     // Altitude���θ߶�(ԭelv)��< Antenna altitude above/below mean sea level (geoid) in meters ���θ߶ȣ������ջ���������ڴ��ˮ׼��ĸ߶ȣ���λΪ�����ס�
    I32 m_iSpeedKnot;    // �Ե��ٶȣ���λΪ��0.001�ڡ� 
    I32 m_iSpeedMH;      // �����ٶȣ���λΪ����/Сʱ������ʹ��iSpeedKnot*1.852�õ�
    I32 m_iDirection;    // < Track angle in degrees True 
    I32 m_iDeclination;  // < Magnetic variation degrees (Easterly var. subtracts from true course)
    I8 m_cMode;        // ��λģʽ��־ (A = ����ģʽ, D = ���ģʽ, E = ����ģʽ, N = ������Ч)
    NMEA_SAT_INFO m_oSatInfo; // ���������Ϣ�б��������ǵ�ID��Żᱻ��100�ٱ���
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

