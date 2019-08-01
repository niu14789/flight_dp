#ifndef _ST480_H_
#define _ST480_H_


#include "gd32f30x.h"

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
 

#define IIC_SCL_PERIPH_CLOCK  RCU_GPIOB
#define IIC_SDA_PERIPH_CLOCK  RCU_GPIOB

#define IIC_SCL_GPIO_PORT  GPIOB
#define IIC_SCL_GPIO_PIN   GPIO_PIN_10
#define IIC_SDA_GPIO_PORT  GPIOB
#define IIC_SDA_GPIO_PIN   GPIO_PIN_11

#define IIC_PERIPH_CLOCK   RCU_I2C1
#define IIC_NUMBER          I2C1



#define MAG_IIC_PORT       I2C_PORT1

#define MAG_IIC_SPEED        100000
#define IIC_MASTER_ADDR  (0XE0)
#define MAG_IIC_SLAVE_ADDR   0x18//(0X0C <<1 )




#define P_ST480_RESET       P56
#define D_ST480_ADDR       0x18
#define CalThreshold       0


typedef struct sCompass_
{
    F32 afAxisData[3];
}sCompass;



int ST480_Init(void);
int ST480_GetData(sCompass *poDataOut);


#endif












