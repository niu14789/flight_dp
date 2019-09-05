#ifndef   WIFI_LINK_H
#define   WIFI_LINK_H
#include "stdint.h"
#include "stdbool.h"

#define MANUFACTURER            0XE1
#define PLANE_VERSION_HIGH      1
#define PLANE_VERSION_MEDIAN    0
#define PLANE_VERSION_LOW       1

#define AMERICA_HAND            0
#define JAPAN_HAND              1

#define NORMAL_MODE             0X00
#define GREEN_HANDS_MODE        0XFF

enum e_command
{
    CMD0    = 0,
    CMD1    = 1,  
    CMD2    = 2,
    CMD3    = 3,
    CMD4    = 4,
    CMD5    = 5,
    CMD6    = 6, 
    CMD7    = 7,
    CMD8    = 8,
    CMD9    = 9,
};

enum e_bit
{
    BIT0    = 0X01,
    BIT1    = 0X02,
    BIT2    = 0X04,
    BIT3    = 0X08,
    BIT4    = 0X10,
    BIT5    = 0X20,
    BIT6    = 0X40,
    BIT7    = 0X80,   
    BIT8    = 0X0100,
    BIT9    = 0X0200,
    BIT10    = 0X0400,
    BIT11    = 0X0800,
    BIT12    = 0X1000,
    BIT13    = 0X2000,
    BIT14    = 0X4000,
    BIT15    = 0X8000,  
};

//plnae->wifi����Э�鶨��
#define PLANE_PACKET_HEAD       0xFFFE
#define PLANE_PACKET_HEAD_H     0xFF
#define PLANE_PACKET_HEAD_L     0xFE

enum e_command_packet_len
{
    CMD0_PACKET_LEN    = 31,
    CMD1_PACKET_LEN    = 1,
    CMD2_PACKET_LEN    = 5,
    CMD3_PACKET_LEN    = 5+2,
    CMD4_PACKET_LEN    = 9,
    CMD5_PACKET_LEN    = 1,
    CMD6_PACKET_LEN    = 16,
    CMD7_PACKET_LEN    = 0,
    CMD8_PACKET_LEN    = 0,
    CMD9_PACKET_LEN    = 2,
};



//��ѹ��gps��Ϣ�������ٶȡ��������ٶ�(500ms����һ��)
#pragma pack (1) /*ָ����1�ֽڶ���*/
typedef struct  
{
    uint16_t    plane_voltage;              //��ѹֵ*100
    uint16_t    remoter_voltage;            //��ѹֵ*100
    int32_t     gps_longitude;              //1e-7 ���
    int32_t     gps_latitude;               //1e-7 ���
    uint8_t     gps_NumSv;                  //������  //GPS�ź�ǿ�� 0�������ǣ�1��GPS�ź��� 2���������� 3�����Ƿǳ�ǿ
    uint16_t    heading;                    //0-360 ���
    int16_t     distance;                   //����    ��ʵ����*10
    int16_t     altitude;                   //�߶�    ��ʵ�߶�*10
    uint16_t    horizontal_speed;           //ˮƽ�ٶ� *10
    uint16_t    vertical_speed;             //��ֱ�ٶ� *10  
    uint8_t     plane_battery_remaining_capacity;   //�ɻ����ʣ�����
    uint8_t     plane_battery_remaining_time;       //�ɻ����ʣ�����ʱ��
    int16_t     plane_pitch_angle;                  //�ɻ�������
    int16_t     plane_roll_angle;                   //�ɻ�������
    int16_t     plane_remain;                       //Ԥ��
}plane_cmd0_packet_stru;
#pragma pack () /*ȡ��ָ�����룬�ָ�ȱʡ����*/

extern plane_cmd0_packet_stru plane_cmd0_packet;

//����¼��
typedef struct 
{
    uint8_t    photo_record;                //0x01��ʾ���գ�0x02��ʼ¼��0x03ֹͣ¼��(��Ҫʱ��)
}plane_cmd1_packet_stru;

extern plane_cmd1_packet_stru plane_cmd1_packet;

//��ʾ�룬����ɻ���ң��������ʾ��Ϣ������ɡ�ֹͣ��ң������Ϣ��ʧ500msˢ��һ��
typedef struct 
{
    uint8_t    plane_status1;   //bit0��0����  1����
                                //bit1��0δ���  1���
                                //bit2��0δ�յ�GPS   1�յ�GPS
                                //bit3������
                                //bit4������
                                //bit5��ָ�����
                                //bit6��������
                                //bit7��������
    
    uint8_t     plane_status2;  //bit0�� 0 ������У׼���  1 ������У׼��
                                //bit1�� 0 �Ǵ�����У׼    1 ������ˮƽУ׼��
                                //bit2�� 0 �Ǵ�����У׼    1 �����ƴ�ֱУ׼��
                                //bit3�� 0 ң����δ����    1 ң���������ӣ�ң��������ʱapp��Ҫҡ�ˣ�
                                //bit4:  ����� 0��δ���� 1��������
                                //bit5�� 0 tbd            1 tbd    
                                //bit6�� 0 ����ģʽ       1��ͷģʽ
                                //bit7�� 0 ����ģʽ       1��ͣģʽ     
            
    uint8_t     plane_status3;  //bit0��  0����ģʽ    1����ģʽ
                                //bit1:   0����ģʽ    1�͵�ģʽ
                                //bit2:   0����ģʽ    1����ģʽ
                                //bit3:   0 ��̬ģʽ 1��gpsģʽ
                                //bit4:   0
                                //bit5:   0
                                //bit6:   0
                                //bit7:   0                    1 octant

    uint8_t     plane_status4;  //bit0:   0 MPU����������     1MPU����������
                                //bit1:   0 ��ѹ������        1��ѹ�ƴ���
                                //bit2:   0 GPS��������       1GPS���Ӵ���
                                //bit3:   0 �شŸ�������      1�شŸ��Ź���
                                //bit4:   0 ��С             1 ��󾯸�
                                //bit5��  0 GPS �źź�        1  GPS �ź�������
    
    uint8_t     plane_status5;  //bit0��  ���գ�����Ϊ1���ٰ�Ϊ0��bit7 =1 ��Ч��
                                //bit1:   ¼�񣬰���Ϊ1���ٰ�Ϊ0��bit7 =1 ��Ч��
                                //bit2:   
                                //bit3:   
                                //bit4:   
                                //bit5:   
                                //bit6:   
                                //bit7:   0���м�ң�ذ棬1��2.4Gң�ذ�   
}plane_cmd2_packet_stru;
extern plane_cmd2_packet_stru plane_cmd2_packet;

//plane_cmd2_packet.plane_status1����
void set_plane_cmd2_lock_status(void);
void set_plane_cmd2_unlock_status(void);
void set_plane_cmd2_stop_fly_status(void);
void set_plane_cmd2_flying_status(void);    
void set_plane_cmd2_no_gps_status(void);
void set_plane_cmd2_got_gps_status(void);
void clear_plane_cmd2_follow_mode(void);
void set_plane_cmd2_follow_mode(void);
void clear_plane_cmd2_circle_mode(void);
void set_plane_cmd2_circle_mode(void);
void clear_plane_cmd2_point_mode(void);
void set_plane_cmd2_point_mode(void);
void clear_plane_cmd2_return_noome_mode(void);
void set_plane_cmd2_return_home_mode(void);
void clear_plane_cmd2_landing_mode(void);
void set_plane_cmd2_landing_mode(void);

//plane_cmd2_packet.plane_status2����
void clear_plane_cmd2_gyro_calibrate_status(void);
void set_plane_cmd2_gyro_calibrate_status(void);
void clear_plane_cmd2_compass_horizontal_calibrate_status(void);
void set_plane_cmd2_compass_horizontal_calibrate_status(void);
void clear_plane_cmd2_compass_vertical_calibrate_status(void);
void set_plane_cmd2_compass_vertical_calibrate_status(void);
void set_plane_cmd2_unlink_remoter_status(void);
void set_plane_cmd2_link_remoter_status(void);
void clear_plane_cmd2_nohead_mode(void);
void set_plane_cmd2_nohead_mode(void);
void clear_plane_cmd2_scram_mode(void);
void set_plane_cmd2_scram_mode(void);

//plane_cmd2_packet.plane_status3����
void set_plane_cmd2_low_speed_status(void);
void set_plane_cmd2_high_speed_status(void);
void clear_plane_cmd2_low_voltage_status(void);
void set_plane_cmd2_low_voltage_status(void);
void clear_plane_cmd2_green_hands_mode(void);
void set_plane_cmd2_green_hands_mode(void);
void clear_plane_cmd2_front_left_motor_overload(void);
void set_plane_cmd2_front_left_motor_overload(void);
void clear_plane_cmd2_front_right_motor_overload(void);
void set_plane_cmd2_front_right_motor_overload(void);
void clear_plane_cmd2_back_right_motor_overload(void);
void set_plane_cmd2_back_right_motor_overload(void);
void clear_plane_cmd2_back_left_motor_overload(void);
void set_plane_cmd2_back_left_motor_overload(void);

//plane_cmd2_packet.plane_status4����
void clear_plane_cmd2_mpu_erro(void);
void set_plane_cmd2_mpu_erro(void);
void clear_plane_cmd2_baro_erro(void);
void set_plane_cmd2_baro_erro(void);
void clear_plane_cmd2_gps_erro(void);
void set_plane_cmd2_gps_erro(void);
void clear_plane_cmd2_compass_erro(void);
void set_plane_cmd2_compass_erro(void);
void clear_plane_cmd2_wind_erro(void);
void set_plane_cmd2_wind_erro(void);
void clear_plane_cmd2_gps_rssi_erro(void);
void set_plane_cmd2_gps_rssi_erro(void);
//plane_cmd2_packet.plane_status5����
void update_plane_cmd2_photo_function_to_wifi(void);
void update_plane_cmd2_video_function_to_wifi(void);

void set_plane_cmd2_photo_ev_inc(void) ;
void clear_plane_cmd2_photo_ev_inc(void);
void set_plane_cmd2_photo_ev_dec(void) ;
void clear_plane_cmd2_photo_ev_dec(void);



//�ɻ����ص�ǰ��������
typedef struct 
{
    uint8_t     fly_altitude_limit;         //���Ʒ��и߶�(10---120m)
    uint8_t     fly_distance_limit;         //���Ʒ��о��� (10---250)*2Ĭ��35m1�ֽ�(��λ2m��С�仯2m��ʾ20---500��ʵ�ʷ�������(10-250) 
    uint8_t     return_altitude_limit;      //�����߶�(10---130)
    uint8_t     control_mode;               //����ģʽ0xff ����ģʽ0x00  
    uint8_t     stick_mode;                 //ҡ��ģʽ 0�������֣�1���ձ���
    uint16_t    circle_radius;              //�Ʒɰ뾶
}plane_cmd3_packet_stru;
extern plane_cmd3_packet_stru plane_cmd3_packet;
 
//���ţ����ת�٣��������
typedef struct 
{
    uint8_t     throttle;                   //����
    uint8_t     front_left_rotate_speed;    //ǰ��ת�٣�0-250��
    uint8_t     front_right_rotate_speed;   //ǰ��ת�٣�0-250��
    uint8_t     back_left_rotate_speed;     //����ת�٣�0-250�� 
    uint8_t     back_lright_rotate_speed;   //����ת�٣�0-250��
    uint8_t     front_left_currents;        //ǰ�������0-250��
    uint8_t     front_right_currents;       //ǰ�ҵ�����0-250��
    uint8_t     back_left_currents;         //���������0-250�� 
    uint8_t     back_lright_currents;       //���ҵ�����0-250��
}plane_cmd4_packet_stru;
extern plane_cmd4_packet_stru plane_cmd4_packet;

//�����������ݣ��ɻ��ķ��������룬���߶ȣ������߶ȣ�
//��ǰ�Ƿ�Ϊ����ģʽ��һֱ����ֱ���յ�0x01�յ��������ݣ�
typedef struct 
{
    uint8_t     setting_request;             //0x00 ����������0x01:�յ��������ݣ��ֻ��������
}plane_cmd5_packet_stru;
extern plane_cmd5_packet_stru plane_cmd5_packet;

//���Ҽ��汾
typedef struct 
{
    uint8_t     manufacturer_id;            //������ID
    uint8_t     plane_version_h;            //�ɻ��汾�� ��λ
    uint8_t     plane_version_m;            //�ɻ��汾�� ��λ
    uint8_t     plane_version_l;            //�ɻ��汾�� ��λ
    uint8_t     remoter_version_h;          //ң�����汾�� ��λ
    uint8_t     remoter_version_m;          //ң�����汾�� ��λ
    uint8_t     remoter_version_l;          //ң�����汾�� ��λ  
    uint8_t     gimbal_version_h;           //�ɻ��汾�� ��λ
    uint8_t     gimbal_version_m;           //�ɻ��汾�� ��λ
    uint8_t     gimbal_version_l;           //�ɻ��汾�� ��λ
    uint8_t     camera_version_h;            //ң�����汾�� ��λ
    uint8_t     camera_version_m;            //ң�����汾�� ��λ
    uint8_t     camera_version_l;            //ң�����汾�� ��λ 
    uint8_t     image_version_h;            //ң�����汾�� ��λ
    uint8_t     image_version_m;            //ң�����汾�� ��λ
    uint8_t     image_version_l;            //ң�����汾�� ��λ 
}plane_cmd6_packet_stru;
extern plane_cmd6_packet_stru plane_cmd6_packet;

void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd);

//������Ӧ
typedef struct 
{
    uint8_t     from_device;        //��Դ�豸
    uint8_t     to_device;          //Ŀ������
}plane_cmd9_packet_stru;
extern plane_cmd9_packet_stru plane_cmd9_packet;

//wifi->plane����Э�鶨��
#define WIFI_PACKET_HEAD        0xFFFD
#define WIFI_PACKET_HEAD_H      0xFF
#define WIFI_PACKET_HEAD_L      0xFD

enum e_command_wifi_packet_len
{
    WIFI_CMD0_PACKET_LEN    = 14+2,
    WIFI_CMD1_PACKET_LEN    = 11,
    WIFI_CMD2_PACKET_LEN    = 2,
    WIFI_CMD3_PACKET_LEN    = 5+2,
    WIFI_CMD4_PACKET_LEN    = 2,
    WIFI_CMD5_PACKET_LEN    = 1,
    WIFI_CMD6_PACKET_LEN    = 257, 
    WIFI_CMD7_PACKET_LEN    = 3,
    WIFI_CMD8_PACKET_LEN    = 2,
    WIFI_CMD9_PACKET_LEN    = 2,
};

//�ֻ�gps ָ��������
typedef struct 
{
    int32_t     phone_latitude;             //�ֻ�γ�� int32/1e-7(0x xxxxxxxx��)
    int32_t     phone_longitude;            //�ֻ�����
    uint16_t    phone_gps_accuracy;         //�ֻ�GPS����
    uint16_t    phone_compass_direction;    //�ֻ�ָ���뷽�� 
    uint16_t    follow_altitude;            //����߶�*10 ����0.1M
    uint16_t    follow_me_flag;             //û�и���ʱ����0x0000,����ʱ����0xffff��200ms����һ�ν�����ƽ���һֱ����
}wifi_cmd0_packet_stru;
extern wifi_cmd0_packet_stru wifi_cmd0_packet;

//�ֻ��������ݰ�
typedef struct 
{
    uint8_t     channel_number;             //����ͨ����
    uint8_t     throttle;                   //���� 0 Ϊ��С��250 ��� ,�м�ֵΪ125
    uint8_t     aileron;                    //0 Ϊ��С��250 ��� ,�м�ֵΪ125
    uint8_t     elevator;                   //0 Ϊ��С��250 ��� ,�м�ֵΪ125
    uint8_t     rudder;                     //0 Ϊ��С��250 ��� ,�м�ֵΪ125
    uint8_t     gimbal_pitch;               //0 Ϊ��С��250 ��� ,�м�ֵΪ125
    uint8_t     camera_zoom;                //0 Ϊ��С��250 ��� ,�м�ֵΪ125
    uint8_t     led_control;                //LED�ƿ��� bit6:7 0-3
    uint8_t     control_info;               //������Ϣ
    uint8_t     calibrate_info;             //У׼��Ϣ
    uint8_t     camera_info;                //�����Ϣ
}wifi_cmd1_packet_stru;
extern wifi_cmd1_packet_stru wifi_cmd1_packet;

//�ֻ��������ݰ�
typedef struct 
{
    uint8_t     control_info;               //������Ϣ
    uint8_t     calibrate_info;             //У׼��Ϣ
    uint8_t     camera_info;                //�����Ϣ
}wifi_cmd1_packet_backup_stru;

uint16_t get_phone_control_follow_mode(void);     //0xffff �������棬0x0000 δ����
uint8_t get_phone_control_speed_mode(void); //��С��ģʽ��0/1/2�����и�
uint8_t get_phone_control_roll_mode(void);  //����ģʽ��1:��Ч
void clear_phone_control_roll_mode(void);
uint8_t get_phone_control_return_home_mode(void);   //����ģʽ��1:��Ч
void clear_phone_control_return_home_mode(void);
uint8_t get_phone_control_headless_mode(void);    //��ͷģʽ��1:��Ч
void clear_phone_control_headless_mode(void);
uint8_t get_phone_control_scram_mode(void);     //��ͣģʽ��1:��Ч
void clear_phone_control_scram_mode(void);
uint8_t get_phone_control_rising_landing(void);  //һ������/��½��1:��Ч
void clear_phone_control_rising_landing(void);
//uint8_t get_phone_control_landing(void);
uint8_t get_phone_control_compass_calibrate(void); //�ش�У׼��1:��Ч
void clear_phone_control_compass_calibrate(void);
uint8_t get_phone_control_level_calibrate(void);//ˮƽУ׼��1:��Ч
void clear_phone_control_level_calibrate(void);
uint8_t get_phone_control_lock_info(void);//������1:��Ч
void clear_phone_control_lock_info(void);
uint8_t get_phone_control_hot_spot_around(void);//�ȵ㻷�ƣ�1:��Ч
void clear_phone_control_circle(void);
uint8_t get_phone_control_point_fly(void);//ָ����У�1:��Ч
void clear_phone_control_point_fly(void);
uint8_t get_phone_control_altitude_info(void);  //���߱�־��1:��Ч
void clear_phone_control_altitude_info(void);
uint8_t get_phone_control_one_key_calibrate_flag(void); //һ��У׼��1:��Ч
void clear_phone_control_one_key_calibrate_flag(void);
uint8_t get_phone_control_camera_up_left(void); //����ͷ�����������־��1:��Ч
uint8_t get_phone_control_camera_down_right(void); //����ͷ�½������ұ�־��1:��Ч
uint8_t get_phone_control_led(void); //�ƿ���λ����ʼλ0���ٰ�Ϊ1���ٰ�Ϊ2��0/1/2 ѭ��
//��ȡҡ��ֵ
uint16_t get_throttle_value(void);  
uint16_t get_aileron_value(void);
uint16_t get_elevator_value(void);
uint16_t get_rudder_value(void);
uint16_t get_yuntai_value(void);

//�� ��̨����
typedef struct 
{
    uint8_t     function;                   //0x00�����صƣ� ����0x01(��̨����)
    uint8_t     function_value;             //0x00:�ص� 0x01�����ƣ���ʢ�̣�  0x00��̨���ϣ�0x01��̨����  
}wifi_cmd2_packet_stru;
extern wifi_cmd2_packet_stru wifi_cmd2_packet;

//�ɻ����ã�����ģʽ����Ĭ�����ݣ����ɸı��С��
typedef struct 
{
    uint8_t     altitude_limit_setting;     //���Ʒ��и߶�(10---120m)Ĭ��30m
    uint8_t     distance_limit_setting;     //���Ʒ��о��� (10---250)*2Ĭ��30m1�ֽ�(��λ2m��С�仯2m��ʾ20---500��ʵ�ʷ�������(10-250) 
    uint8_t     return_altitude_setting;    //�����߶�(10---130)    Ĭ��30m  1�ֽ�//�����߶�С�������и߶�
    uint8_t     green_hands_setting;        //����ģʽ����ģʽ0xff ����ģʽ0x00
    uint8_t     stick_mode;                 //ҡ��ģʽ 0�������֣�1���ձ���
    uint16_t    circle_radius;              //�Ʒɰ뾶
}wifi_cmd3_packet_stru;
extern wifi_cmd3_packet_stru wifi_cmd3_packet;

//app����������������
typedef struct 
{
    uint16_t     get_plane_setting_data;     //app���ӷɻ�ʱ������0xffff��ȡ��ǰ�ɻ��������� 
}wifi_cmd4_packet_stru;
extern wifi_cmd4_packet_stru wifi_cmd4_packet;

//У׼ ��������
typedef struct 
{
    uint8_t     control_info;               //Bit0: Ĭ��0�� 1�ų�У׼
                                            //Bit1: Ĭ��0��1ˮƽУ׼
                                            //Bit2: Ĭ��0��1����
}wifi_cmd5_packet_stru;
extern wifi_cmd5_packet_stru wifi_cmd5_packet;

uint8_t get_phone_control_compass_calibrate2(void); //Ĭ��0���ų�У׼Ϊ1
uint8_t get_phone_control_level_calibrate2(void); //Ĭ��0��ˮƽУ׼Ϊ1
uint8_t get_phone_control_lock_info2(void); //Ĭ��0������Ϊ1

//������У���㣩��
#pragma pack (1) /*ָ����1�ֽڶ���*/
typedef struct 
{
    uint8_t     waypoint_number;            //������ 1---32
    uint32_t    longitude_latitude[32*2];   //����ľ�γ�� ע��γ����ǰ�������ں� ��Ҫָ���ͼ��γ��(wgs84����) (���) 4�ֽ�(int32/1e-7(0x xxxxxxxx��))
                                            //ע��һ��������Ϣ����γ�ȣ�8byte, �ܹ������32���㣩256byte, ����ֵ��û��Ϊ�գ����ȹ̶���
                                            //����ʱ�������500�׻�С��һ����Χʱ�ɻ���Ҫ������ʾ���ʺϷ��У���������֮��ɻ�������ʾ��ʼ���У�
                                            //����Ŀ��ص�ʱ�ɻ�������ʾָ�������ɣ����ڷ���ʱ�ٴη��������൱��ָֹͣ����зɻ���ʾ����δ��ɣ�ҡ�˲���Ҳ����ʹ�ɻ�ֹͣ��ǰָ�����
                                            //��������8�Σ����100ms��ǰ�Ĵ�У����ǰһλΪ0xff ���Ĵ�У����ǰһλΪ0x00��ʼһ��ָ�����
}wifi_cmd6_packet_stru;
#pragma pack () /*ȡ��ָ�����룬�ָ�ȱʡ����*/
extern wifi_cmd6_packet_stru wifi_cmd6_packet;


//�Ƶ���У��Ƶ�İ뾶2�ֽ�,�����ȷ����ť���������Ͱ˴μ��100ms ǰ�Ĵ�У����ǰһλΪ0xff ���Ĵ�У����ǰһλΪ0x00��ʼһ���Ƶ����
//�ٴε�����ư�ť����Ϊȡ������ͬ����������� ����8�λ��ư�ť״̬�ı� ����fffd05070005fff8  fffd0507000500f8
typedef struct 
{ 
    uint16_t        radius;                              
}wifi_cmd7_packet_stru;
extern wifi_cmd7_packet_stru wifi_cmd7_packet;

#define SYNCHRONOUS_CMD     0XAA55
//ͬ������
typedef struct 
{
    uint16_t    synchronous_request;        //ͬ������                                       
}wifi_cmd8_packet_stru;
extern wifi_cmd8_packet_stru wifi_cmd8_packet;

//ͬ������
typedef struct 
{
    uint8_t     from_device;        //��Դ�豸
    uint8_t     to_device;          //Ŀ������
}wifi_cmd9_packet_stru;
extern wifi_cmd9_packet_stru wifi_cmd9_packet;


typedef enum
{
    ROLLING_MODE = 1<<2,        //����ģʽ
    RETURN_HOME_MODE = 1<<3,    //����ģʽ
    HEADLESS_MODE = 1<<4,       //��ͷģʽ
    SCRAM_MODE = 1<<5,          //��ͣģʽ
    RISGING_LANDING_MODE = 1<<6,//һ�����/��½ģʽ
} plane_control_info_e;

typedef enum
{
    COMPASS_CALIBRATE = 1<<0,
    LEVEL_CALIBRATE = 1<<1,
    UNLOCK_PLANE = 1<<2,
    HOT_SPOT_AROUND = 1<<3,
    POINT_FLY = 1<<4,
    ONE_KEY_CALIBRATE = 1<<5,
    ALTITUDE_FUCTION = 1<<6,
} plane_calibrate_info_e;

//typedef enum
//{

//}plane_camero_info_e;

//�ɻ����ƹ��ܴ洢�Ĵ���
typedef struct 
{
    uint16_t w_lose_control_timer;
    uint8_t by_lose_control;   
    uint8_t by_control_info;    //Bit 0��1	��С��ģʽ��0/1/2�����и�
                                //Bit2	����ģʽ������Ϊ1���ٰ�Ϊ0
                                //Bit3	����ģʽ������Ϊ1���ٰ�Ϊ0
                                //Bit4	��ͷģʽ������Ϊ1���ٰ�Ϊ0
                                //Bit5	��ͣģʽ������Ϊ1���ٰ�Ϊ0
                                //Bit6	һ������/��½������Ϊ1���ٰ�Ϊ0
                                //Bit7	һ����½������Ϊ1���ٰ�Ϊ0
    uint8_t by_calibrate_info;  //Bit0	�ش�У׼������Ϊ1���ٰ�Ϊ0
                                //Bit1	ˮƽУ׼������Ϊ1���ٰ�Ϊ0
                                //Bit2	����������Ϊ1���ٰ�Ϊ0
                                //Bit3	�ȵ㻷�ƣ�����Ϊ1���ٰ�Ϊ0
                                //Bit4	ָ����У�����Ϊ1���ٰ�Ϊ0
                                //Bit5	һ��У׼������Ϊ1���ٰ�Ϊ0
                                //Bit6	���߱�־������Ϊ1���ٰ�Ϊ0
                                //Bit7  
    uint8_t by_camero_info;     //Bit0	���գ�����Ϊ1���ٰ�Ϊ0��bit7 = 1 ʱ��remoter����Ч��
                                //Bit1	¼�񣬰���Ϊ1���ٰ�Ϊ0��bit7 = 1 ʱ��remoter����Ч��
                                //Bit2	����״̬����������Ϊ1������Ϊ0
                                //Bit3	¼��״̬������¼����Ϊ1���ر�Ϊ0
                                //Bit4	����ͷ�½������ұ�־������Ϊ1���ɿ�Ϊ0
                                //Bit5	����ͷ�����������־������Ϊ1���ɿ�Ϊ0
                                //Bit6	
                                //Bit7	0��phone��1��remoter
}plane_function_control_info_stru;

extern plane_function_control_info_stru m_plane_function_control_info;

typedef enum
{
    STATUS_OFF = 0,
    STATUS_ON = 1,
}status_def;

//�ɿ��㷨����״̬
typedef enum
{
    PLANE_LOCK_STATUS  = 0,                                     //0����   1������
    PLANE_FLYING_STATUS = 1,                                    //0��ͣ��  1��������
    PLANE_GYRO_CALIBRATE_STATUS = 2,                            //1��ˮƽУ׼��
    PLANE_COMPASS_HORIZONTAL_CALIBRATE_STATUS = 3,              //1���ش�ˮƽУ׼ horizontal
    PLANE_COMPASS_VERTICAL_CALIBRATE_STATUS = 4,                //1���شŴ�ֱУ׼ vertical
    PLANE_FOLLOW_MODE_STATUS = 5,                               //����ģʽ 0��δ��   1������
    PLANE_CIRCLE_MODE_STATUS = 6,                               //����ģʽ 0��δ��   1������
    PLANE_POINT_MODE_STATUS = 7,                                //ָ�����ģʽ 0��δ��   1������
    PLANE_RETURN_HOME_MODE_STATUS = 8,                          //������ 1Ϊ���� 0Ϊδ����
    PLANE_LANDING_STATUS = 9,                                   //���� 0��δ��   1������
    PLANE_LINK_REMOTE_STATUS = 10,                              //0ң����δ����    1ң���������ӣ�ң��������ʱapp��Ҫҡ�ˣ�
    PLANE_HEADLESS_MODE_STATUS = 11,                            //0����ģʽ    1��ͷģʽ
    PLANE_SCRAM_MODE_STATUS = 12,                               //0����ģʽ    1��ͣģʽ
    PLANE_SPEED_MODE_STATUS = 13,                               //0����ģʽ    1����ģʽ
    PLANE_LOW_POWER_MODE_STATUS = 14,                           //0����ģʽ    1�͵�ģʽ
    PLANE_GREEN_HANDS_MODE_STATUS = 15,                         //0����ģʽ    1����ģʽ
//    PLANE_FRONT_LEFT_MOTOR_OVERLOAD_STATUS = 16,                //0�ɻ�ǰ��������    1�ɻ�ǰ��������
//    PLANE_FRONT_RIGHT_MOTOR_OVERLOAD_STATUS = 17,               //0�ɻ�ǰ�ҵ������    1�ɻ�ǰ�ҵ������
//    PLANE_BACK_LEFT_MOTOR_OVERLOAD_STATUS = 18,                 //0�ɻ�����������    1�ɻ�����������
//    PLANE_BACK_RIGHT_MOTOR_OVERLOAD_STATUS = 19,                //0�ɻ����ҵ������    1�ɻ����ҵ������
    PLANE_MPU_ERRO_STATUS = 20,                                 //0 MPU����������      1 MPU����������
    PLANE_BARO_ERRO_STATUS = 21,                                //0 ��ѹ������         1 ��ѹ�ƴ���
    PLANE_GPS_ERRO_STATUS = 22,                                 //0 GPS��������        1 GPS���Ӵ���
    PLANE_COMPASS_ERRO_STATUS = 23,                             //0 �شŸ�������       1 �شŸ��Ź���
    PLANE_WIND_ERRO_STATUS = 24,                                //0 ��С               1 ��󾯸�
    PLANE_GPS_RSSI_STATUS = 25,                                 //0 GPS �źź�         1 GPS �ź�������   
    PLANE_REMOTE_CHOICE_STATUS = 26,							//0 �м�ң����           1 2.4Gң����
    PLANE_RISING_STATUS = 27,									//һ����� 0��δ��    1������
    PLANE_IMU_GPS_MODE_STATUS = 28,                             //0:��̬ģʽ 1��GPSģʽ

}plane_status_def;

//�ɿ��㷨����״̬
typedef enum
{
    PLANE_LOCK_FLAG_STATUS = 1<<0,                              //0����   1������
    PLANE_FLYING_FLAG_STATUS = 1<<1,                            //0��ͣ��  1��������
    PLANE_GYRO_CALIBRATE_FLAG_STATUS = 1<<2,                    //1��У׼��
    PLANE_COMPASS_HORIZONTAL_CALIBRATE_FLAG_STATUS = 1<<3,      //1���ش�ˮƽУ׼ horizontal
    PLANE_COMPASS_VERTICAL_CALIBRATE_FLAG_STATUS = 1<<4,        //1���شŴ�ֱУ׼ vertical
    PLANE_FOLLOW_MODE_FLAG_STATUS = 1<<5,                       //����ģʽ 0��δ��   1������
    PLANE_CIRCLE_MODE_FLAG_STATUS = 1<<6,                       //����ģʽ 0��δ��   1������
    PLANE_POINT_MODE_FLAG_STATUS = 1<<7,                        //ָ�����ģʽ 0��δ��   1������
    PLANE_RETURN_HOME_MODE_FLAG_STATUS = 1<<8,                  //������ 1Ϊ���� 0Ϊδ����
    PLANE_LANDING_FLAG_STATUS = 1<<9,                           //���� 0��δ��   1������
    PLANE_LINK_REMOTE_FLAG_STATUS = 1<<10,                       //0ң����δ����    1ң���������ӣ�ң��������ʱapp��Ҫҡ�ˣ�
    PLANE_HEADLESS_MODE_FLAG_STATUS = 1<<11,                    //0����ģʽ    1��ͷģʽ
    PLANE_SCRAM_MODE_FLAG_STATUS = 1<<12,                       //0����ģʽ    1��ͣģʽ
    PLANE_SPEED_MODE_FLAG_STATUS = 1<<13,                       //0����ģʽ    1����ģʽ
    PLANE_LOW_POWER_MODE_FLAG_STATUS = 1<<14,                   //0����ģʽ    1�͵�ģʽ
    PLANE_GREEN_HANDS_MODE_FLAG_STATUS = 1<<15,                 //0����ģʽ    1����ģʽ
//    PLANE_FRONT_LEFT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<16,        //0�ɻ�ǰ��������    1�ɻ�ǰ��������
//    PLANE_FRONT_RIGHT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<17,       //0�ɻ�ǰ�ҵ������    1�ɻ�ǰ�ҵ������
//    PLANE_BACK_LEFT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<18,         //0�ɻ�����������    1�ɻ�����������
//    PLANE_BACK_RIGHT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<19,        //0�ɻ����ҵ������    1�ɻ����ҵ������
    PLANE_MPU_ERRO_FLAG_STATUS = 1<<20,                         //0 MPU����������      1 MPU����������
    PLANE_BARO_ERRO_FLAG_STATUS = 1<<21,                        //0 ��ѹ������         1 ��ѹ�ƴ���
    PLANE_GPS_ERRO_FLAG_STATUS = 1<<22,                         //0 GPS��������        1 GPS���Ӵ���
    PLANE_COMPASS_ERRO_FLAG_STATUS = 1<<23,                     //0 �شŸ�������       1 �شŸ��Ź���
    PLANE_WIND_ERRO_FLAG_STATUS = 1<<24,                        //0 ��С               1 ��󾯸�
    PLANE_GPS_RSSI_FLAG_STATUS = 1<<25,                         //0 GPS �źź�         1 GPS �ź�������
    PLANE_REMOTE_CHOICE_FLAG_STATUS = 1<<26,					//0 �м�ң����           1 2.4Gң����
    PLANE_RISING_FLAG_STATUS = 1<<27,							//һ����� 0��δ��    1������
    PLANE_IMU_GPS_MODE_FLAG_STATUS = 1<<28,                     //0:��̬ģʽ 1��GPSģʽ
    
}plane_status_flag_def;

extern uint32_t g_dw_plane_status;      //�ɻ�״̬


//���»�ͷ������APP
void updata_plane_heading_to_wifi(uint16_t wheading);
//���¾�����APP
void updata_plane_distance_to_wifi(int16_t sdistance);
//���º�����APP
void updata_plane_altitude_to_wifi(uint16_t saltitude);


void wifi_link_init(void);
void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd);
void wifi_data_packet_decode(uint8_t *pbuffer);
//����Ƶ��1000hz
void wifi_link_data_receive(void);
uint8_t get_camera_photo_status(void);
uint8_t get_camera_video_status(void);
//����Ƶ��1000hz
void wifi_link_data_send(void);
void store_fly_limit_setting_data(void);
bool read_fly_limit_setting_data(void);
void wifi_function_dispose(void);
int erase_iap_id(void);

void update_plane_status_now(plane_status_def plane_function,status_def status);

//��ȡ�ɻ������ܵĵ�ǰ״̬
status_def get_plane_status_now(plane_status_def plane_function);



//
// ��ȡ8bitУ���
uint8_t get_u8data_checksum(uint8_t *pbuf,uint8_t len);

#pragma pack (1) /*ָ����2�ֽڶ���*/
typedef struct FLASH_STRU
{   
    uint8_t     updata_flag[4];
    uint8_t     rfID[8];
    uint8_t     remote_version[4];
    uint8_t     FlyLimitSettingData[8];
    uint16_t    store_flag;
    uint8_t     check_sum;      //λ�ò��ܶ���������ڽṹ�����
}FLASH_DEF;
#pragma pack () /*ȡ��ָ�����룬�ָ�ȱʡ����*/
extern FLASH_DEF flash;

#define FLASH_DATA_BUFFER_LEN  sizeof(flash) 

#endif



