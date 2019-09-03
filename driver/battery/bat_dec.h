#ifndef _BSP_TEST_H_
#define _BSP_TEST_H_

#include "gd32f30x.h"
#include "stdio.h"
#include "string.h"

#define SWAP(a,b) { uint32_t t = a; a = b; b = t; t = 0; }
#define MBEDTLS_DES_KEY_SIZE    8
#define GET_UINT32_BE(n,b,i)                            \
{                                                       \
    (n) = ( (uint32_t) (b)[(i)    ] << 24 )             \
        | ( (uint32_t) (b)[(i) + 1] << 16 )             \
        | ( (uint32_t) (b)[(i) + 2] <<  8 )             \
        | ( (uint32_t) (b)[(i) + 3]       );            \
}


#define HANDSHAK_LENGTH  8
#define PASSWORD_BUF_SIZE 8
#define MESG_VALUE_SIZE   49
#define HANDSHAKE_SUCCESS_SIZE 3

typedef struct BatMes
{
	uint8_t  m_byFrameTitleCorrectFlag;
	uint16_t m_wBatCapacity;
	uint16_t m_wBatVoltage;
	uint16_t m_wBatCurrent;
	
}oBatMes;



typedef struct mbedtls_des_context
{
    uint32_t sk[32];            /*!<  DES subkeys       */
}mbedtls_des_context;


typedef struct _HandShakePacket_
{
	uint8_t m_byHandShakeSuccessFlag;
	uint8_t m_abyHandShakeBuf[HANDSHAK_LENGTH];
	uint8_t m_abyKeyDataBuf[PASSWORD_BUF_SIZE];
	uint8_t m_abyOriginalValueBuf[PASSWORD_BUF_SIZE];
	uint8_t m_abyEncryptValueBuf[PASSWORD_BUF_SIZE];
	uint8_t m_byCrcOutValue;
}HandShakePacket;

typedef struct 
{
	unsigned short bat_vol[4];
	unsigned short status;
	unsigned short cycles;
	unsigned short remine_cap;
	unsigned short full_cap;
	unsigned short pre_time;
	unsigned short temperature;
	unsigned short cap;
	unsigned short av_current;
	unsigned short total_voltage;
	unsigned short rt_current;
	unsigned short health_status;
	/* other fucking data */
	unsigned short others[6];
}battery_msg_def;



extern HandShakePacket g_oHandShakePacket;
extern const uint8_t Mkey[8];
extern const uint8_t Skey[8];
extern unsigned char g_abyValidKey[8];

void DesEncrypt(const unsigned char *pbyPlainTextIn, unsigned char *pbyCipherTextOut, const unsigned char *pbyKeyIn);
void DesDecrypt(const unsigned char *pbCipherTextIn, unsigned char *pbyPlainTextOut, const unsigned char *pbyKeyIn);

void AckToBat(const unsigned char * Packet,unsigned char * dst);

int BatMesgDecode(unsigned char *pbyBatMesgInput,battery_msg_def * msg);

#endif











