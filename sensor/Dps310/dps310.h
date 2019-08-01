/*
 * Copyright (c) 2015-2016 Infineon Technologies AG
 *
 * Driver for Infineon DPS310 Digital Barometric Pressure Sensor
 *
 *
 */

#ifndef DPS310_H_INCLUDED
#define DPS310_H_INCLUDED
 
 /* _______________________________________________________ */
 
 /*Some aliases*/
 
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
 
 
#ifndef F32
#define F32 float
#endif
 
 
#ifndef F64
#define F64 double
#endif
 
#ifndef bool
#define bool int
#endif
 
 

typedef struct _BARO_METER_DATA_
{
    F32 m_fTempature;
    F32 m_fPress;
    F32 m_fAltitude;
}BARO_METER_DATA;



/* public function prototypes */
int Dps310Init(void);

int Dps310GetProcessedData(BARO_METER_DATA *poBaroMeterData);

F32 AirPressureToAltitude(F32 fAirPressure);


#endif // DPS310_H_INCLUDED
