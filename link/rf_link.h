#ifndef   _RF_LINK_H_
#define   _RF_LINK_H_
#include "stdint.h"
#include "stdbool.h"
#include "XN297L.h"


#define TX_MODE         0
#define RX_MODE         1
#define RF_WORK_MODE    RX_MODE

#define RF_RELINK_CH        81
#define RF_BINDONG_CH       77
#define FREQ_CH_NUMBER      10

#if (RF_WORK_MODE == TX_MODE)
#define RF_LINK_TX_TIME     2
#define RF_LINK_RX_TIME     8
#define RF_LINK_CYCLE       10
#define RF_HOPPING_TIME     10
#else
#define RF_LINK_TX_TIME     8
#define RF_LINK_RX_TIME     2
#define RF_HOPPING_TIME     9
#define RF_LINK_CYCLE       10 
#endif

#define RF_LINK_BUFFER_HEAD     0x20
#define RF_LINK_BUFFER_LEN      18

enum e_link_step
{
    RF_STEP_NULL = 0,
    RF_STEP_TX =1,
    RF_STEP_RX =2,
    RF_STEP_HOPPING =3,
};

enum e_link_status
{
    DISCONNECCTING  =   0,
    CONNECCTING   =   1,
};

enum e_key_status
{
    STATUS_OFF  =   0,
    STATUS_ON   =   1,
};

enum e_remote_function_control
{
    //POWER_FUNCTION_BIT = 1U << 0,
    VIDEO_FUNCTION_BIT = 1U << 1,
    PHOTO_FUNCTION_BIT = 1U << 2,
    RETURN_HOME_FUNCTION_BIT = 1U << 3,
    RISE_LAND_FUNCTION_BIT = 1U << 4,
    MODE_FUNCTION_BIT = 1U << 5,
    SCRAM_FUNCTION_BIT = 1U << 6,
    //STICK_RIGHT_FUNCTION_BIT = 1U << 6,
    //STICK_LEFT_FUNCTION_BIT = 1U << 7,   
};

#pragma pack (1) /*ָ����1�ֽڶ���*/
typedef struct REMOTE_PACKET_STRU
{
//    uint8_t     head;
//    uint8_t     len;
    uint8_t    key_status;          //ң��������״̬ 
    uint8_t    function_status;     //ң�������� 
    uint16_t    throttle;           //����    1000~2000 �м�ֵ1500
    uint16_t    aileron;            //��ת    1000~2000 �м�ֵ1500
    uint16_t    elevator;           //ǰ��    1000~2000 �м�ֵ1500
    uint16_t    rudder;             //����    1000~2000 �м�ֵ1500
    uint16_t    yuntai;             //��̨����  1000~2000 �м�ֵ1500
    uint16_t    camera_zoom;        //�������  1000~2000 �м�ֵ1500
    uint8_t     battery_voltage;    //ң�ص�ѹֵ*10
//    uint8_t     checksum;    
}remote_packet_def;
#pragma pack () /*ȡ��ָ�����룬�ָ�ȱʡ����*/

extern remote_packet_def remote_packet;

enum e_rf_rx_buffer_index 
{
    KEY_STATUS_INDEX = 3,
    FUNCTION_STATUS_INDEX = 4,
    THROTTLE_INDEX = 5,
    AILERON_INDEX  = 7,
    ELEVATOR_INDEX = 9,
    RUDDER_INDEX = 11,
    YUNTAI_INDEX = 13,
    CAMERA_INDEX = 15,
    BATTERY_INDEX = 17,    
};

typedef struct PLANE_PACKET_STRU
{
    uint8_t     head;
    uint8_t     len;
    uint16_t    plane_staus;        //�ɻ�״̬
    
    uint8_t     battery_voltage;    //��ѹֵ*10
    uint8_t     checksum;    
}plane_packer_def;


typedef struct LINK_STRU
{
    volatile    uint32_t        link_timer_counter;
    volatile    uint8_t         hopping_counter;
    bool            link_status;
    bool            binding_status;
    uint8_t         loss_link_counter;
    uint8_t         link_step;
    uint8_t         tx_buffer[PAYLOAD_WIDTH];       //���ͻ���
    uint8_t         rx_buffer[PAYLOAD_WIDTH];       //���ջ���
    uint8_t         rf_new_address[5];              //RF ID 
    uint8_t         remoter_version[3];             //ң�ذ汾��
    uint16_t        plane_function_flag;            //�ɻ����ܱ�־λ
}link_def;
extern link_def s_rf_link;

extern const uint8_t frequency_channal_tab[10];
//RF ��ʼ��
void rf_link_init(void);
//ʵʱ���ã�����Ӱ��ͨѶ
void rf_link_function(void);
void rf_binding(void);
uint8_t get_u8data_checksum(uint8_t *pbuf,uint8_t len);
bool read_rf_receive_data(uint8_t *pbuf,uint8_t len);
void rf_id_store(void);
bool rf_id_read(void);
//����Ƶ��Ϊ1000HZ
void rf_link_timer(void);
//��ȡң����ģʽ���� ����Ϊ0������Ϊ1
uint8_t get_remote_mode_function_control(void);
//��ȡң�����������ܿ��� ����Ϊ1���ٰ�Ϊ0
uint8_t get_remote_return_home_function_control(void);
//��ȡң�������չ��ܿ��� ���չ��ܣ�����Ϊ1���ٰ�Ϊ0
uint8_t get_remote_photo_function_control(void);
//��ȡң����¼���ܿ��� ���չ��ܣ�����Ϊ1���ٰ�Ϊ0
uint8_t get_remote_record_function_control(void);
//��ȡң����һ����ɽ��书�ܿ��ƣ�����Ϊ1���ٰ�Ϊ0
uint8_t get_remote_rise_land_function_control(void);

//���·ɻ�����״̬
void update_plane_photo_function_to_remote(void);
//���·ɻ�¼��״̬
void update_plane_video_function_to_remote(void);

#endif


