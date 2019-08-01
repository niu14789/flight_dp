#ifndef _IMU_CALIBRATION_H_
#define _IMU_CALIBRATION_H_


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


#define CALIBRATION_TIMES_MAX 2000
#define CALIBRATION_TIMES_MIN 100

int ICM20600_CalibrationAndSaveParameter(int iCalibrateTimes);

#endif

