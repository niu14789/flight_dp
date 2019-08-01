#ifndef   WIFI_LINK_H
#define   WIFI_LINK_H
#include "stdint.h"
#include "stdbool.h"
#include "wifi_link.h"

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
    CMD3_PACKET_LEN    = 5,
    CMD4_PACKET_LEN    = 9,
    CMD5_PACKET_LEN    = 1,
    CMD6_PACKET_LEN    = 16,    
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
}plane_cmd0_packet_def;
#pragma pack () /*ȡ��ָ�����룬�ָ�ȱʡ����*/

extern plane_cmd0_packet_def plane_cmd0_packet;

//����¼��
typedef struct 
{
    uint8_t    photo_record;                //0x01��ʾ���գ�0x02��ʼ¼��0x03ֹͣ¼��(��Ҫʱ��)
}plane_cmd1_packet_def;

extern plane_cmd1_packet_def plane_cmd1_packet;

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
                                //bit4:  0 tbd            1 tbd
                                //bit5�� 0 tbd            1 tbd    
                                //bit6�� 0 ����ģʽ       1��ͷģʽ
                                //bit7�� 0 ����ģʽ       1��ͣģʽ     
            
    uint8_t     plane_status3;  //bit0��  0����ģʽ    1����ģʽ
                                //bit1:   0����ģʽ    1�͵�ģʽ
                                //bit2:   0����ģʽ    1����ģʽ
                                //bit3:   0�ɻ�ǰ��������    1�ɻ�ǰ��������
                                //bit4:   0�ɻ�ǰ�ҵ������    1�ɻ�ǰ�ҵ������	
                                //bit5:   0�ɻ����ҵ������    1�ɻ����ҵ������	
                                //bit6:   0�ɻ�����������    1�ɻ�����������	
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
}plane_cmd2_packet_def;
extern plane_cmd2_packet_def plane_cmd2_packet;

//plane_cmd2_packet.plane_status1����
void set_plane_lock_status(void);
void set_plane_unlock_status(void);
uint8_t get_plane_lock_status(void);
void set_plane_stop_fly_status(void);
void set_plane_flying_status(void);
uint8_t get_plane_flying_status(void);
    
void set_plane_no_gps_status(void);
void set_plane_got_gps_status(void);
void clear_plane_follow_mode(void);
void set_plane_follow_mode(void);
void clear_plane_circle_mode(void);
void set_plane_circle_mode(void);
void clear_plane_point_mode(void);
void set_plane_point_mode(void);
void clear_plane_return_noome_mode(void);
void set_plane_return_home_mode(void);
uint8_t get_plane_return_home_mode_status(void);
void clear_plane_landing_mode(void);
void set_plane_landing_mode(void);
uint8_t get_plane_landing_mode_status(void);

//plane_cmd2_packet.plane_status2����
void clear_plane_gyro_calibrate_status(void);
void set_plane_gyro_calibrate_status(void);
void clear_plane_compass_horizontal_calibrate_status(void);
void set_plane_compass_horizontal_calibrate_status(void);
void clear_plane_compass_vertical_calibrate_status(void);
void set_plane_compass_vertical_calibrate_status(void);
void set_plane_unlink_remoter_status(void);
void set_plane_link_remoter_status(void);
void clear_plane_nohead_mode(void);
void set_plane_nohead_mode(void);
uint8_t get_plane_nohead_mode_status(void);
void clear_plane_scram_mode(void);
void set_plane_scram_mode(void);

//plane_cmd2_packet.plane_status3����
void set_plane_low_speed_status(void);
void set_plane_high_speed_status(void);
void clear_plane_low_voltage_status(void);
void set_plane_low_voltage_status(void);
uint8_t get_plane_low_voltage_status(void);
void clear_plane_green_hands_mode(void);
void set_plane_green_hands_mode(void);
void clear_plane_front_left_motor_overload(void);
void set_plane_front_left_motor_overload(void);
void clear_plane_front_right_motor_overload(void);
void set_plane_front_right_motor_overload(void);
void clear_plane_back_right_motor_overload(void);
void set_plane_back_right_motor_overload(void);
void clear_plane_back_left_motor_overload(void);
void set_plane_back_left_motor_overload(void);

//plane_cmd2_packet.plane_status4����
void clear_plane_mpu_erro(void);
void set_plane_mpu_erro(void);
void clear_plane_baro_erro(void);
void set_plane_baro_erro(void);
void clear_plane_gps_erro(void);
void set_plane_gps_erro(void);
void clear_plane_compass_erro(void);
void set_plane_compass_erro(void);
void clear_plane_wind_erro(void);
void set_plane_wind_erro(void);
void clear_plane_gps_rssi_erro(void);
void set_plane_gps_rssi_erro(void);
//plane_cmd2_packet.plane_status5����
void update_plane_photo_function_to_wifi(void);
void update_plane_video_function_to_wifi(void);

//�ɻ����ص�ǰ��������
typedef struct 
{
    uint8_t     fly_altitude_limit;         //���Ʒ��и߶�(10---120m)
    uint8_t     fly_distance_limit;         //���Ʒ��о��� (10---250)*2Ĭ��35m1�ֽ�(��λ2m��С�仯2m��ʾ20---500��ʵ�ʷ�������(10-250) 
    uint8_t     return_altitude_limit;      //�����߶�(10---130)
    uint8_t     control_mode;               //����ģʽ0xff ����ģʽ0x00  
    uint8_t     stick_mode;                 //ҡ��ģʽ 0�������֣�1���ձ���
}plane_cmd3_packet_def;
extern plane_cmd3_packet_def plane_cmd3_packet;

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
}plane_cmd4_packet_def;
extern plane_cmd4_packet_def plane_cmd4_packet;

//�����������ݣ��ɻ��ķ��������룬���߶ȣ������߶ȣ�
//��ǰ�Ƿ�Ϊ����ģʽ��һֱ����ֱ���յ�0x01�յ��������ݣ�
typedef struct 
{
    uint8_t     setting_request;             //0x00 ����������0x01:�յ��������ݣ��ֻ��������
}plane_cmd5_packet_def;
extern plane_cmd5_packet_def plane_cmd5_packet;

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
}plane_cmd6_packet_def;
extern plane_cmd6_packet_def plane_cmd6_packet;

void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd);

//wifi->plane����Э�鶨��
#define WIFI_PACKET_HEAD        0xFFFD
#define WIFI_PACKET_HEAD_H      0xFF
#define WIFI_PACKET_HEAD_L      0xFD

enum e_command_wifi_packet_len
{
    WIFI_CMD0_PACKET_LEN    = 14,
    WIFI_CMD1_PACKET_LEN    = 11,
    WIFI_CMD2_PACKET_LEN    = 2,
    WIFI_CMD3_PACKET_LEN    = 5,
    WIFI_CMD4_PACKET_LEN    = 2,
    WIFI_CMD5_PACKET_LEN    = 1,
    WIFI_CMD6_PACKET_LEN    = 257, 
    WIFI_CMD7_PACKET_LEN    = 3,
    WIFI_CMD8_PACKET_LEN    = 2,
};

//�ֻ�gps ָ��������
typedef struct 
{
    int32_t     phone_latitude;             //�ֻ�γ�� int32/1e-7(0x xxxxxxxx��)
    int32_t     phone_longitude;            //�ֻ�����
    uint16_t    phone_gps_accuracy;         //�ֻ�GPS����
    uint16_t    phone_compass_direction;    //�ֻ�ָ���뷽�� 
    uint16_t    follow_me_flag;             //û�и���ʱ����0x0000,����ʱ����0xffff��200ms����һ�ν�����ƽ���һֱ����
}wifi_cmd0_packet_def;
extern wifi_cmd0_packet_def wifi_cmd0_packet;

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
}wifi_cmd1_packet_def;
extern wifi_cmd1_packet_def wifi_cmd1_packet;

uint8_t get_phone_control_speed_mode(void); //��С��ģʽ��0/1/2�����и�
uint8_t get_phone_control_roll_mode(void);  //����ģʽ������Ϊ1���ٰ�Ϊ0
uint8_t get_phone_control_return_home_mode(void);   //����ģʽ������Ϊ1���ٰ�Ϊ0
uint8_t get_phone_control_nohead_mode(void);    //��ͷģʽ������Ϊ1���ٰ�Ϊ0
uint8_t get_phone_control_scram_mode(void);     //��ͣģʽ������Ϊ1���ٰ�Ϊ0
uint8_t get_phone_control_start_landing(void);  //һ������/��½������Ϊ1���ٰ�Ϊ0
//uint8_t get_phone_control_landing(void);
uint8_t get_phone_control_compass_calibrate(void); //�ش�У׼������Ϊ1��1���Ϊ0
uint8_t get_phone_control_level_calibrate(void);//ˮƽУ׼������Ϊ1��1���Ϊ0
uint8_t get_phone_control_lock_info(void);//����������Ϊ1��1���Ϊ0
uint8_t get_phone_control_hot_spot_around(void);//�ȵ㻷�ƣ�����Ϊ1���ٰ�Ϊ0 
uint8_t get_phone_control_point_fly(void);//ָ����У�����Ϊ1���ٰ�Ϊ0
uint8_t get_phone_control_altitude_info(void);  //���߱�־������Ϊ1���ٰ�Ϊ0
uint8_t get_phone_control_one_key_calibrate_flag(void); //һ��У׼������Ϊ1���ɿ�Ϊ0
uint8_t get_phone_control_camera_up_left(void); //����ͷ�����������־������Ϊ1���ɿ�Ϊ0
uint8_t get_phone_control_camera_down_right(void); //����ͷ�½������ұ�־������Ϊ1���ɿ�Ϊ0
uint8_t get_phone_control_led(void); //�ƿ���λ����ʼλ0���ٰ�Ϊ1���ٰ�Ϊ2��0/1/2 ѭ��

//�� ��̨����
typedef struct 
{
    uint8_t     function;                   //0x00�����صƣ� ����0x01(��̨����)
    uint8_t     function_value;             //0x00:�ص� 0x01�����ƣ���ʢ�̣�  0x00��̨���ϣ�0x01��̨����  
}wifi_cmd2_packet_def;
extern wifi_cmd2_packet_def wifi_cmd2_packet;

//�ɻ����ã�����ģʽ����Ĭ�����ݣ����ɸı��С��
typedef struct 
{
    uint8_t     altitude_limit_setting;     //���Ʒ��и߶�(10---120m)Ĭ��30m
    uint8_t     distance_limit_setting;     //���Ʒ��о��� (10---250)*2Ĭ��30m1�ֽ�(��λ2m��С�仯2m��ʾ20---500��ʵ�ʷ�������(10-250) 
    uint8_t     return_altitude_setting;    //�����߶�(10---130)    Ĭ��30m  1�ֽ�//�����߶�С�������и߶�
    uint8_t     green_hands_setting;        //����ģʽ����ģʽ0xff ����ģʽ0x00
    uint8_t     stick_mode;                 //ҡ��ģʽ 0�������֣�1���ձ���
}wifi_cmd3_packet_def;
extern wifi_cmd3_packet_def wifi_cmd3_packet;

//app����������������
typedef struct 
{
    uint16_t     get_plane_setting_data;     //app���ӷɻ�ʱ������0xffff��ȡ��ǰ�ɻ��������� 
}wifi_cmd4_packet_def;
extern wifi_cmd4_packet_def wifi_cmd4_packet;

//У׼ ��������
typedef struct 
{
    uint8_t     control_info;               //Bit0: Ĭ��0�� 1�ų�У׼
                                            //Bit1: Ĭ��0��1ˮƽУ׼
                                            //Bit2: Ĭ��0��1����
}wifi_cmd5_packet_def;
extern wifi_cmd5_packet_def wifi_cmd5_packet;

uint8_t get_phone_control_compass_calibrate2(void); //Ĭ��0���ų�У׼Ϊ1
uint8_t get_phone_control_level_calibrate2(void); //Ĭ��0��ˮƽУ׼Ϊ1
uint8_t get_phone_control_lock_info2(void); //Ĭ��0������Ϊ1

//������У���㣩��
typedef struct 
{
    uint8_t     waypoint_number;            //������ 1---32
    uint32_t    longitude_latitude[32*2];   //����ľ�γ�� ע��γ����ǰ�������ں� ��Ҫָ���ͼ��γ��(wgs84����) (���) 4�ֽ�(int32/1e-7(0x xxxxxxxx��))
                                            //ע��һ��������Ϣ����γ�ȣ�8byte, �ܹ������32���㣩256byte, ����ֵ��û��Ϊ�գ����ȹ̶���
                                            //����ʱ�������500�׻�С��һ����Χʱ�ɻ���Ҫ������ʾ���ʺϷ��У���������֮��ɻ�������ʾ��ʼ���У�
                                            //����Ŀ��ص�ʱ�ɻ�������ʾָ�������ɣ����ڷ���ʱ�ٴη��������൱��ָֹͣ����зɻ���ʾ����δ��ɣ�ҡ�˲���Ҳ����ʹ�ɻ�ֹͣ��ǰָ�����
                                            //��������8�Σ����100ms��ǰ�Ĵ�У����ǰһλΪ0xff ���Ĵ�У����ǰһλΪ0x00��ʼһ��ָ�����
}wifi_cmd6_packet_def;
extern wifi_cmd6_packet_def wifi_cmd6_packet;


//�Ƶ���У��Ƶ�İ뾶2�ֽ�,�����ȷ����ť���������Ͱ˴μ��100ms ǰ�Ĵ�У����ǰһλΪ0xff ���Ĵ�У����ǰһλΪ0x00��ʼһ���Ƶ����
//�ٴε�����ư�ť����Ϊȡ������ͬ����������� ����8�λ��ư�ť״̬�ı� ����fffd05070005fff8  fffd0507000500f8
typedef struct 
{
    uint16_t    circle_radius;              //�Ʒɰ뾶
    uint8_t     check_code;                 //У����
                                        
}wifi_cmd7_packet_def;
extern wifi_cmd7_packet_def wifi_cmd7_packet;

#define SYNCHRONOUS_CMD     0XAA55
//ͬ������
typedef struct 
{
    uint16_t    synchronous_request;        //ͬ������                                       
}wifi_cmd8_packet_def;
extern wifi_cmd8_packet_def wifi_cmd8_packet;


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
#endif


