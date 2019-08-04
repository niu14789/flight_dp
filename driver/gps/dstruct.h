/*
 * dstruct.h
 *
 *  Created on: 2017骞?5鏈?24鏃?
 *      Author: YJ-User17
 */

#ifndef __DSTRUCT_H__
#define __DSTRUCT_H__

#define IMU_SELECT (0)// 0 is adi , 1 is 20609

//#if IMU_SELECT
typedef struct
{
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;	
	float temperature;
	unsigned int timestamp;
}IMU_DATA_ICM;
//#else
typedef struct
{
	float x;
	float y;
	float z;
	float temperature;

	unsigned int	x_raw;
	unsigned int	y_raw;
	unsigned int	z_raw;
	unsigned int	temperature_raw;
	unsigned int 	timestamp;
}IMU_DATA;
//#endif
typedef struct{
	float satellite_elevation;	//satellite elevation
	float satellite_azimuth;	//satellite azimuth
	short satellite_num;		//satellite number
	short satellite_SNR;		//satellite signal to noise ratio
}GPS_GSV_t;
//#endif

//add by zxd @2016-11-24
typedef struct{
	int		fixType;
	int		NumUsedSv;
	float		pDOP;		/**&lt; Position DOP [0.01] */
	float		hDOP;		/**&lt; h DOP [0.01] */
	float		vDOP;		/**&lt; v DOP [0.01] */
}GPS_UsedInf_t;

typedef struct{
	GPS_GSV_t gpgsv[12];		//satellite information,there're most 12 GPS satellite
	GPS_GSV_t glgsv[9];		//satellite information,there're most 9 GLNOSS satellite
	GPS_GSV_t bdgsv[8];		//satellite information,there're most 8 BeiDou satellite

	GPS_UsedInf_t usedBdInf;
	GPS_UsedInf_t usedGlInf;

	unsigned int	position_timestamp;
	double		lon;			/**&lt; Longitude [deg] */
	double		lat;			/**&lt; Latitude [deg] */
	float		height;		/**&lt; Height above mean sea level [m] */

	float		velN;		/**&lt; NED north velocity [m/s]*/
	float		velE;		/**&lt; NED east velocity [m/s]*/
	float		velU;		/**&lt; NED up velocity [m/s]*/

	float		pDOP;		/**&lt; Position DOP [0.01] */
	float		hDOP;		/**&lt; h DOP [0.01] */
	float		vDOP;		/**&lt; v DOP [0.01] */

	float   	quality;		/**&lt; GPS quality */
	unsigned int	intervalTimeMs;

	int		nano;		/**&lt; Fraction of second (UTC) [-1e9...1e9 ns] */
	unsigned short	positon_delayMs;	//the time difference between satellite positon time and data recieve time
	unsigned short	t_north;		/* ture north */
	unsigned short	gpsdatchecksum; //gps data check sum
	unsigned char		fixType;		/**&lt; GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	unsigned char		numSV;			/**&lt; Number of SVs used in Nav Solution */
	unsigned char		NumUsedgpsSv;	/**&lt; Number of used gps SVs  */
	unsigned char		NumgpsSv;		/**&lt; Number of gps SVs  */
	unsigned char		NumglSv;		/**&lt; Number of glonss SVs  */
	unsigned char		NumbdSv;		/**&lt; Number of beidou SVs  */
	unsigned char		hour; 			/**&lt; Hour of day, range 0..23 (UTC) */
	unsigned char		min; 			/**&lt; Minute of hour, range 0..59 (UTC) */
	unsigned char		sec;			/**&lt; Seconds of minute, range 0..60 (UTC) */

	//add year-moth-day@2017-1-11
	unsigned char		year;
	unsigned char		month;
	unsigned char		date;
	//end
}GPS_DATA;

typedef struct
{
	float pressure;
	float altitude;
	float temperature;
    unsigned int timestamp;
}BARO_DATA;

typedef struct __mavlink_fm_optical_flow_t
{
	unsigned int time_usec; /*&lt; time delay in ns (UNIX)*/
	float flow_x; /*&lt; Flow in pixels * 10 in x-sensor direction (dezi-pixels)*/
	float flow_y; /*&lt; Flow in pixels * 10 in y-sensor direction (dezi-pixels)*/
	float flow_comp_m_x; /*&lt; Flow in meters in x-sensor direction, angular-speed compensated*/
	float flow_comp_m_y; /*&lt; Flow in meters in y-sensor direction, angular-speed compensated*/
	float ground_distance; /*&lt; Ground distance in meters. Positive value: distance known. Negative value: Unknown distance*/
	char sensor_id; /*&lt; Sensor ID*/
	char quality; /*&lt; Optical flow quality / confidence. 0: bad, 255: maximum quality*/
	char light_quality; //澧炲姞涓€鏉″厜绾挎槑鏆楃殑鏍囧織锛岃嫢涓?1鍒欐槸鏄庝寒鐜锛屼负0鍒欒鏄庢槸鍏夌嚎鏆楃殑鑰佺幆澧?
	char reserve2;
	unsigned int timestamp; /*&lt; Timestamp in ns (UNIX)*/
	float distance; /*ObsAvoidance distance*/
	float posx;
	float posy;
	float posz;
	int reserve;
} mavlink_fm_optical_flow_t;

typedef struct {
	unsigned int TimeStamp;				  /*!&lt; 32-bit time stamp. */
	unsigned int MeasurementTimeUsec;
		/*!&lt; Give the Measurement time needed by the device to do the
		 * measurement.*/


	unsigned short RangeMilliMeter;		  /*!&lt; range distance in millimeter. */

	unsigned short RangeDMaxMilliMeter;
		/*!&lt; Tells what is the maximum detection distance of the device
		 * in current setup and environment conditions (Filled when
		 *	applicable) */

	unsigned int SignalRateRtnMegaCps;
		/*!&lt; Return signal rate (MCPS)\n these is a 16.16 fix point
		 *	value, which is effectively a measure of target
		 *	 reflectance.*/
	unsigned int AmbientRateRtnMegaCps;
		/*!&lt; Return ambient rate (MCPS)\n these is a 16.16 fix point
		 *	value, which is effectively a measure of the ambien
		 *	t light.*/

	unsigned short EffectiveSpadRtnCount;
		/*!&lt; Return the effective SPAD count for the return signal.
		 *	To obtain Real value it should be divided by 256 */

	unsigned char ZoneId;
		/*!&lt; Denotes which zone and range scheduler stage the range
		 *	data relates to. */
	unsigned char RangeFractionalPart;
		/*!&lt; Fractional part of range distance. Final value is a
		 *	FixPoint168 value. */
	unsigned char RangeStatus;
		/*!&lt; Range Status for the current measurement. This is device
		 *	dependent. Value = 0 means value is valid.
		 *	See \ref RangeStatusPage */
} VL53L0_RangingMeasurementData_t;


typedef struct battery_status
{
	volatile float 	  temperature;
	volatile unsigned int cell_voltage1;
	volatile unsigned int cell_voltage2;
	volatile unsigned int cell_voltage3;
	volatile unsigned int cell_voltage4;
	volatile float 	  voltage;
	volatile float 	  current;
	volatile float 	  avgcurrent;
	volatile unsigned int resOChrPer;
	volatile unsigned int absOChrPer;
	volatile unsigned int remaining;
	volatile unsigned int remainRunTime;
	volatile unsigned int remainAvgTime;
	volatile unsigned int safeyStatus;
	volatile unsigned char voiceMode;
	volatile unsigned char dsdState;

	volatile unsigned int full_capacity;
	volatile unsigned int cycle_time;
	volatile unsigned int design_cap;
	volatile unsigned int design_vol;
	volatile unsigned int manu_data;
	volatile unsigned int ser_num;
	volatile unsigned int life_percent;
	volatile unsigned int fw_version;
	volatile unsigned int total_cycles;
	volatile unsigned char  auth_result;
}BATTERY_STATUS;

typedef struct fence
{
	float REL_CLIMB_MIN;
	float FENCE_ENABLE;
	float FENCE_ALT_MAX;
	float FENCE_RADIUS;
	float HOME_LAT;
	float HOME_LON;
	float CONTROL_TIME;
}FENCE;

typedef struct notification
{
	unsigned char notification_type;
	unsigned int  notification_result;
	char 	notification_msg[16];
}NOTIFICATION;

//typedef enum{
//	DISABLE=0,
//	ENABLE=1,
//	UPDATE=2,
//	OTHER=3,
//}STATUS;

typedef union{
	unsigned int data;
	struct{
		unsigned char imuStartOkF:1;
		unsigned char imuBeReadF:1;

		unsigned char ms5611StartOkF:1;
		unsigned char ms5611BeReadF:1;

		unsigned char hmcStartOkF:1;
		unsigned char hmcBeReadF:1;

		unsigned char m3SonarUpF:1;
		unsigned char m3ExtGpsUpF:1;
		unsigned char m3DatReadCnt:8;

		unsigned char gimbalStartOkF:1;
		unsigned char gimbalDatUpF:1;
	}bits;
}sensorSta_u;

typedef struct track_start
{
	char type;
	int input_x;
	int input_y;
	int width;
	int height;
	unsigned int param_1;
	unsigned int param_2;
	unsigned char Start:2;
	unsigned char VisualTrackParallel:2;
	unsigned char VisualRound:2;
	unsigned char VisualDistantView:2;
	unsigned char VisualTrackGeneral;
}TRACK_START;

typedef struct track_pos
{
	char type;
	char tracker_flag;
	char quality;
	char other_flag;
	unsigned int time;
	int rt_x;
	int rt_y;
	int width;
	int height;
	unsigned long long time_pos;
}TRACK_POS;

typedef struct track_stop
{
	char type;
	char tracker_flag;
}TRACK_STOP;

typedef struct{
	unsigned char  ringTime;
	unsigned char  calringTime;
	unsigned char  measureMode;
	unsigned char  errCode;
	unsigned short disOrig;
	unsigned short disProc;
}sonarInf_t;


//GM_NORMAL
typedef struct gm_normal
{
	short posture_pitch;
	short posture_roll;
	unsigned int motor_time;
	unsigned int flight_time;
	unsigned long long rec_time;
	unsigned int gm_statues;
}GM_NORMAL;

//GM_ERROR
typedef struct gm_error
{
	unsigned char imu_status;
	unsigned char pitch_status;
	unsigned char roll_status;
	unsigned char obligate;
	unsigned long long rec_time;
}GM_ERROR;

typedef struct heartbeat_t
{
	unsigned int custom;
	unsigned char type;
	unsigned char autopilot;
	unsigned char base_mode;
	unsigned char system_status;
	unsigned char mavlink_version;
	unsigned int time_boot_ms;
}HEARTBEAT;

typedef struct rc_channels
{
	unsigned short chan1_raw;
	unsigned short chan2_raw;
	unsigned short chan3_raw;
	unsigned short chan4_raw;
	unsigned short chan5_raw;
	unsigned short chan6_raw;
	unsigned short chan7_raw;
	unsigned short chan8_raw;
	unsigned char	 target_system;
	unsigned char	 target_component;
}RC_CHANNELS;

__packed typedef struct
{
	short motor_pitch;
	short motor_roll;
	short flight_pitch;
	short flight_roll;
	short flight_yaw;
	short speed_pitch;
	short speed_roll;
	short speed_yaw;
	short gyro_pitch_bias;
	short gyro_roll_bias;
	short gyro_yaw_bias;
	unsigned int time_stamp;
	unsigned char flightstatus;
	unsigned char board_version;
	short flight_target_pitch;
	short flight_target_roll;
        unsigned short gm_work_status;
}flight_control_msgTypeDef;

//typedef struct{
//	unsigned int	    iTOW;		/**&lt; GPS Time of Week [ms] */
//	unsigned short	    year; 		/**&lt; Year (UTC)*/
//	unsigned char		month; 		/**&lt; Month, range 1..12 (UTC) */
//	unsigned char		day; 		/**&lt; Day of month, range 1..31 (UTC) */
//	unsigned char		hour; 		/**&lt; Hour of day, range 0..23 (UTC) */
//	unsigned char		min; 		/**&lt; Minute of hour, range 0..59 (UTC) */
//	unsigned char		sec;		/**&lt; Seconds of minute, range 0..60 (UTC) */
//	unsigned char		valid; 		/**&lt; Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
//	unsigned int	    tAcc; 		/**&lt; Time accuracy estimate (UTC) [ns] */
//	int		            nano;		/**&lt; Fraction of second (UTC) [-1e9...1e9 ns] */
//	unsigned char		fixType;	/**&lt; GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
//	unsigned char		flags;		/**&lt; Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
//	unsigned char		reserved1;
//	unsigned char		numSV;		/**&lt; Number of SVs used in Nav Solution */
//	float		        lon;		/**&lt; Longitude [1e-7 deg] */
//	float		        lat;		/**&lt; Latitude [1e-7 deg] */
//	float		        height;		/**&lt; Height above ellipsoid [mm] */
//	int		        hMSL;		/**&lt; Height above mean sea level [mm] */
//	unsigned int	hAcc;  		/**&lt; Horizontal accuracy estimate [mm] */
//	unsigned int	vAcc;  		/**&lt; Vertical accuracy estimate [mm] */
//	float		        velN;		/**&lt; NED north velocity [mm/s]*/
//	float		        velE;		/**&lt; NED east velocity [mm/s]*/
//	float		        velD;		/**&lt; NED down velocity [mm/s]*/
//	int		        gSpeed;		/**&lt; Ground Speed (2-D) [mm/s] */
//	int		        headMot;	/**&lt; Heading of motion (2-D) [1e-5 deg] */
//	unsigned int	sAcc;		/**&lt; Speed accuracy estimate [mm/s] */
//	unsigned int	headAcc;	/**&lt; Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
//	unsigned short	pDOP;		/**&lt; Position DOP [0.01] */
//	unsigned short	reserved2;
//	unsigned int	reserved3;
//	int		        headVeh;    	/**&lt; (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
//	unsigned int	reserved4;	/**&lt; (ubx8+ only) */
//	unsigned int	timeplus_timestamp;
//	unsigned int	position_timestamp;
//	unsigned int	error_timestamp;
//}gps_data;

extern BATTERY_STATUS batteryData;

#endif /* __DSTRUCT_H__ */




















