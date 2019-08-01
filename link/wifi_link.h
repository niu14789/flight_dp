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

//plnae->wifi数据协议定义
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



//电压、gps信息、距离速度、最大距离速度(500ms更新一次)
#pragma pack (1) /*指定按1字节对齐*/
typedef struct  
{
    uint16_t    plane_voltage;              //电压值*100
    uint16_t    remoter_voltage;            //电压值*100
    int32_t     gps_longitude;              //1e-7 大端
    int32_t     gps_latitude;               //1e-7 大端
    uint8_t     gps_NumSv;                  //卫星数  //GPS信号强度 0：无卫星，1：GPS信号弱 2：卫星正常 3：卫星非常强
    uint16_t    heading;                    //0-360 大端
    int16_t     distance;                   //距离    真实距离*10
    int16_t     altitude;                   //高度    真实高度*10
    uint16_t    horizontal_speed;           //水平速度 *10
    uint16_t    vertical_speed;             //垂直速度 *10  
    uint8_t     plane_battery_remaining_capacity;   //飞机电池剩余电量
    uint8_t     plane_battery_remaining_time;       //飞机电池剩余飞行时间
    int16_t     plane_pitch_angle;                  //飞机俯仰角
    int16_t     plane_roll_angle;                   //飞机翻滚角
    int16_t     plane_remain;                       //预留
}plane_cmd0_packet_def;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/

extern plane_cmd0_packet_def plane_cmd0_packet;

//拍照录像
typedef struct 
{
    uint8_t    photo_record;                //0x01表示拍照，0x02开始录像，0x03停止录像(需要时发)
}plane_cmd1_packet_def;

extern plane_cmd1_packet_def plane_cmd1_packet;

//提示码，输出飞机或遥控器的提示信息，如起飞、停止、遥控器信息丢失500ms刷新一次
typedef struct 
{
    uint8_t    plane_status1;   //bit0：0加锁  1解锁
                                //bit1：0未起飞  1起飞
                                //bit2：0未收到GPS   1收到GPS
                                //bit3：跟随
                                //bit4：环绕
                                //bit5：指点飞行
                                //bit6：返航中
                                //bit7：降落中
    
    uint8_t     plane_status2;  //bit0： 0 陀螺仪校准完成  1 陀螺仪校准中
                                //bit1： 0 非磁力计校准    1 磁力计水平校准中
                                //bit2： 0 非磁力计校准    1 磁力计垂直校准中
                                //bit3： 0 遥控器未连接    1 遥控器已连接（遥控器链接时app不要摇杆）
                                //bit4:  0 tbd            1 tbd
                                //bit5： 0 tbd            1 tbd    
                                //bit6： 0 正常模式       1无头模式
                                //bit7： 0 正常模式       1急停模式     
            
    uint8_t     plane_status3;  //bit0：  0低速模式    1高速模式
                                //bit1:   0正常模式    1低电模式
                                //bit2:   0正常模式    1新手模式
                                //bit3:   0飞机前左电机保护    1飞机前左电机过载
                                //bit4:   0飞机前右电机保护    1飞机前右电机过载	
                                //bit5:   0飞机后右电机保护    1飞机后右电机过载	
                                //bit6:   0飞机后左电机保护    1飞机后左电机过载	
                                //bit7:   0                    1 octant

    uint8_t     plane_status4;  //bit0:   0 MPU传感器正常     1MPU传感器错误
                                //bit1:   0 气压计正常        1气压计错误
                                //bit2:   0 GPS连接正常       1GPS连接错误
                                //bit3:   0 地磁干扰正常      1地磁干扰过大
                                //bit4:   0 风小             1 风大警告
                                //bit5：  0 GPS 信号好        1  GPS 信号弱警告
    
    uint8_t     plane_status5;  //bit0：  拍照，按下为1，再按为0（bit7 =1 有效）
                                //bit1:   录像，按下为1，再按为0（bit7 =1 有效）
                                //bit2:   
                                //bit3:   
                                //bit4:   
                                //bit5:   
                                //bit6:   
                                //bit7:   0：中继遥控版，1：2.4G遥控版   
}plane_cmd2_packet_def;
extern plane_cmd2_packet_def plane_cmd2_packet;

//plane_cmd2_packet.plane_status1操作
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

//plane_cmd2_packet.plane_status2操作
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

//plane_cmd2_packet.plane_status3操作
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

//plane_cmd2_packet.plane_status4操作
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
//plane_cmd2_packet.plane_status5操作
void update_plane_photo_function_to_wifi(void);
void update_plane_video_function_to_wifi(void);

//飞机返回当前设置数据
typedef struct 
{
    uint8_t     fly_altitude_limit;         //限制飞行高度(10---120m)
    uint8_t     fly_distance_limit;         //限制飞行距离 (10---250)*2默认35m1字节(单位2m最小变化2m显示20---500，实际发送数据(10-250) 
    uint8_t     return_altitude_limit;      //返航高度(10---130)
    uint8_t     control_mode;               //新手模式0xff 正常模式0x00  
    uint8_t     stick_mode;                 //摇杆模式 0：美国手，1：日本手
}plane_cmd3_packet_def;
extern plane_cmd3_packet_def plane_cmd3_packet;

//油门，电机转速，电机电流
typedef struct 
{
    uint8_t     throttle;                   //油门
    uint8_t     front_left_rotate_speed;    //前左转速（0-250）
    uint8_t     front_right_rotate_speed;   //前右转速（0-250）
    uint8_t     back_left_rotate_speed;     //后左转速（0-250） 
    uint8_t     back_lright_rotate_speed;   //后右转速（0-250）
    uint8_t     front_left_currents;        //前左电流（0-250）
    uint8_t     front_right_currents;       //前右电流（0-250）
    uint8_t     back_left_currents;         //后左电流（0-250） 
    uint8_t     back_lright_currents;       //后右电流（0-250）
}plane_cmd4_packet_def;
extern plane_cmd4_packet_def plane_cmd4_packet;

//请求设置数据，飞机的飞行最大距离，最大高度，返航高度，
//当前是否为新手模式（一直发送直到收到0x01收到设置数据）
typedef struct 
{
    uint8_t     setting_request;             //0x00 发设置请求，0x01:收到设置数据，手机设置完成
}plane_cmd5_packet_def;
extern plane_cmd5_packet_def plane_cmd5_packet;

//厂家及版本
typedef struct 
{
    uint8_t     manufacturer_id;            //制造商ID
    uint8_t     plane_version_h;            //飞机版本号 高位
    uint8_t     plane_version_m;            //飞机版本号 中位
    uint8_t     plane_version_l;            //飞机版本号 低位
    uint8_t     remoter_version_h;          //遥控器版本号 高位
    uint8_t     remoter_version_m;          //遥控器版本号 中位
    uint8_t     remoter_version_l;          //遥控器版本号 低位  
    uint8_t     gimbal_version_h;           //飞机版本号 高位
    uint8_t     gimbal_version_m;           //飞机版本号 中位
    uint8_t     gimbal_version_l;           //飞机版本号 低位
    uint8_t     camera_version_h;            //遥控器版本号 高位
    uint8_t     camera_version_m;            //遥控器版本号 中位
    uint8_t     camera_version_l;            //遥控器版本号 低位 
    uint8_t     image_version_h;            //遥控器版本号 高位
    uint8_t     image_version_m;            //遥控器版本号 中位
    uint8_t     image_version_l;            //遥控器版本号 低位 
}plane_cmd6_packet_def;
extern plane_cmd6_packet_def plane_cmd6_packet;

void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd);

//wifi->plane数据协议定义
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

//手机gps 指南针数据
typedef struct 
{
    int32_t     phone_latitude;             //手机纬度 int32/1e-7(0x xxxxxxxx度)
    int32_t     phone_longitude;            //手机经度
    uint16_t    phone_gps_accuracy;         //手机GPS精度
    uint16_t    phone_compass_direction;    //手机指南针方向 
    uint16_t    follow_me_flag;             //没有跟随时发送0x0000,跟随时发送0xffff，200ms发送一次进入控制界面一直发送
}wifi_cmd0_packet_def;
extern wifi_cmd0_packet_def wifi_cmd0_packet;

//手机控制数据包
typedef struct 
{
    uint8_t     channel_number;             //控制通道数
    uint8_t     throttle;                   //油门 0 为最小，250 最大 ,中间值为125
    uint8_t     aileron;                    //0 为最小，250 最大 ,中间值为125
    uint8_t     elevator;                   //0 为最小，250 最大 ,中间值为125
    uint8_t     rudder;                     //0 为最小，250 最大 ,中间值为125
    uint8_t     gimbal_pitch;               //0 为最小，250 最大 ,中间值为125
    uint8_t     camera_zoom;                //0 为最小，250 最大 ,中间值为125
    uint8_t     led_control;                //LED灯控制 bit6:7 0-3
    uint8_t     control_info;               //控制信息
    uint8_t     calibrate_info;             //校准信息
    uint8_t     camera_info;                //相机信息
}wifi_cmd1_packet_def;
extern wifi_cmd1_packet_def wifi_cmd1_packet;

uint8_t get_phone_control_speed_mode(void); //大小档模式，0/1/2，低中高
uint8_t get_phone_control_roll_mode(void);  //翻滚模式，按下为1，再按为0
uint8_t get_phone_control_return_home_mode(void);   //返航模式，按下为1，再按为0
uint8_t get_phone_control_nohead_mode(void);    //无头模式，按下为1，再按为0
uint8_t get_phone_control_scram_mode(void);     //急停模式，按下为1，再按为0
uint8_t get_phone_control_start_landing(void);  //一键启动/着陆，按下为1，再按为0
//uint8_t get_phone_control_landing(void);
uint8_t get_phone_control_compass_calibrate(void); //地磁校准，按下为1，1秒后为0
uint8_t get_phone_control_level_calibrate(void);//水平校准，按下为1，1秒后为0
uint8_t get_phone_control_lock_info(void);//解锁，按下为1，1秒后为0
uint8_t get_phone_control_hot_spot_around(void);//热点环绕，按下为1，再按为0 
uint8_t get_phone_control_point_fly(void);//指点飞行，按下为1，再按为0
uint8_t get_phone_control_altitude_info(void);  //定高标志，按下为1，再按为0
uint8_t get_phone_control_one_key_calibrate_flag(void); //一键校准，按下为1，松开为0
uint8_t get_phone_control_camera_up_left(void); //摄像头上升或向左标志，按下为1，松开为0
uint8_t get_phone_control_camera_down_right(void); //摄像头下降或向右标志，按下为1，松开为0
uint8_t get_phone_control_led(void); //灯控制位，初始位0，再按为1，再按为2，0/1/2 循环

//灯 云台控制
typedef struct 
{
    uint8_t     function;                   //0x00（开关灯） 功能0x01(云台控制)
    uint8_t     function_value;             //0x00:关灯 0x01：开灯（碧盛禾）  0x00云台向上，0x01云台向下  
}wifi_cmd2_packet_def;
extern wifi_cmd2_packet_def wifi_cmd2_packet;

//飞机设置（新手模式发送默认数据，不可改变大小）
typedef struct 
{
    uint8_t     altitude_limit_setting;     //限制飞行高度(10---120m)默认30m
    uint8_t     distance_limit_setting;     //限制飞行距离 (10---250)*2默认30m1字节(单位2m最小变化2m显示20---500，实际发送数据(10-250) 
    uint8_t     return_altitude_setting;    //返航高度(10---130)    默认30m  1字节//返航高度小于最大飞行高度
    uint8_t     green_hands_setting;        //新手模式新手模式0xff 正常模式0x00
    uint8_t     stick_mode;                 //摇杆模式 0：美国手，1：日本手
}wifi_cmd3_packet_def;
extern wifi_cmd3_packet_def wifi_cmd3_packet;

//app发送请求设置数据
typedef struct 
{
    uint16_t     get_plane_setting_data;     //app链接飞机时，发送0xffff获取当前飞机设置数据 
}wifi_cmd4_packet_def;
extern wifi_cmd4_packet_def wifi_cmd4_packet;

//校准 解锁控制
typedef struct 
{
    uint8_t     control_info;               //Bit0: 默认0， 1磁场校准
                                            //Bit1: 默认0，1水平校准
                                            //Bit2: 默认0，1解锁
}wifi_cmd5_packet_def;
extern wifi_cmd5_packet_def wifi_cmd5_packet;

uint8_t get_phone_control_compass_calibrate2(void); //默认0，磁场校准为1
uint8_t get_phone_control_level_calibrate2(void); //默认0，水平校准为1
uint8_t get_phone_control_lock_info2(void); //默认0，解锁为1

//航点飞行（多点）：
typedef struct 
{
    uint8_t     waypoint_number;            //航点数 1---32
    uint32_t    longitude_latitude[32*2];   //航点的经纬度 注意纬度在前。经度在后 需要指点地图经纬度(wgs84坐标) (大端) 4字节(int32/1e-7(0x xxxxxxxx度))
                                            //注：一个坐标信息（经纬度）8byte, 总共（最大32个点）256byte, 有则赋值，没则为空，长度固定，
                                            //飞行时距离大于500米或小于一定范围时飞机需要返回提示不适合飞行，发送坐标之后飞机返回提示开始飞行，
                                            //到达目标地点时飞机可以提示指点飞行完成，正在飞行时再次发送坐标相当于停止指点飞行飞机提示飞行未完成，摇杆操作也可以使飞机停止当前指点飞行
                                            //发送数据8次（间隔100ms）前四次校验码前一位为0xff 后四次校验码前一位为0x00开始一次指点飞行
}wifi_cmd6_packet_def;
extern wifi_cmd6_packet_def wifi_cmd6_packet;


//绕点飞行：绕点的半径2字节,点击（确定按钮）连续发送八次间隔100ms 前四次校验码前一位为0xff 后四次校验码前一位为0x00开始一次绕点飞行
//再次点击环绕按钮，则为取消环绕同样发送上面的 数据8次环绕按钮状态改变 例：fffd05070005fff8  fffd0507000500f8
typedef struct 
{
    uint16_t    circle_radius;              //绕飞半径
    uint8_t     check_code;                 //校验码
                                        
}wifi_cmd7_packet_def;
extern wifi_cmd7_packet_def wifi_cmd7_packet;

#define SYNCHRONOUS_CMD     0XAA55
//同步请求
typedef struct 
{
    uint16_t    synchronous_request;        //同步请求                                       
}wifi_cmd8_packet_def;
extern wifi_cmd8_packet_def wifi_cmd8_packet;


//更新机头方向至APP
void updata_plane_heading_to_wifi(uint16_t wheading);
//更新距离至APP
void updata_plane_distance_to_wifi(int16_t sdistance);
//更新海拔至APP
void updata_plane_altitude_to_wifi(uint16_t saltitude);


void wifi_link_init(void);
void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd);
void wifi_data_packet_decode(uint8_t *pbuffer);
//访问频率1000hz
void wifi_link_data_receive(void);
uint8_t get_camera_photo_status(void);
uint8_t get_camera_video_status(void);
//访问频率1000hz
void wifi_link_data_send(void);
void store_fly_limit_setting_data(void);
bool read_fly_limit_setting_data(void);
#endif


