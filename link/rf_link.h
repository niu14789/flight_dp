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

#pragma pack (1) /*指定按1字节对齐*/
typedef struct REMOTE_PACKET_STRU
{
//    uint8_t     head;
//    uint8_t     len;
    uint8_t    key_status;          //遥控器按键状态 
    uint8_t    function_status;     //遥控器功能 
    uint16_t    throttle;           //油门    1000~2000 中间值1500
    uint16_t    aileron;            //旋转    1000~2000 中间值1500
    uint16_t    elevator;           //前后    1000~2000 中间值1500
    uint16_t    rudder;             //左右    1000~2000 中间值1500
    uint16_t    yuntai;             //云台控制  1000~2000 中间值1500
    uint16_t    camera_zoom;        //相机控制  1000~2000 中间值1500
    uint8_t     battery_voltage;    //遥控电压值*10
//    uint8_t     checksum;    
}remote_packet_def;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/

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
    uint16_t    plane_staus;        //飞机状态
    
    uint8_t     battery_voltage;    //电压值*10
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
    uint8_t         tx_buffer[PAYLOAD_WIDTH];       //发送缓存
    uint8_t         rx_buffer[PAYLOAD_WIDTH];       //接收缓存
    uint8_t         rf_new_address[5];              //RF ID 
    uint8_t         remoter_version[3];             //遥控版本号
    uint16_t        plane_function_flag;            //飞机功能标志位
}link_def;
extern link_def s_rf_link;

extern const uint8_t frequency_channal_tab[10];
//RF 初始化
void rf_link_init(void);
//实时调用，否则影响通讯
void rf_link_function(void);
void rf_binding(void);
uint8_t get_u8data_checksum(uint8_t *pbuf,uint8_t len);
bool read_rf_receive_data(uint8_t *pbuf,uint8_t len);
void rf_id_store(void);
bool rf_id_read(void);
//调用频率为1000HZ
void rf_link_timer(void);
//获取遥控器模式控制 正常为0，其他为1
uint8_t get_remote_mode_function_control(void);
//获取遥控器返航功能控制 按下为1，再按为0
uint8_t get_remote_return_home_function_control(void);
//获取遥控器拍照功能控制 拍照功能，按下为1，再按为0
uint8_t get_remote_photo_function_control(void);
//获取遥控器录像功能控制 拍照功能，按下为1，再按为0
uint8_t get_remote_record_function_control(void);
//获取遥控器一键起飞降落功能控制，按下为1，再按为0
uint8_t get_remote_rise_land_function_control(void);

//更新飞机拍照状态
void update_plane_photo_function_to_remote(void);
//更新飞机录像状态
void update_plane_video_function_to_remote(void);

#endif


