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

//plnae->wifi数据协议定义
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
}plane_cmd0_packet_stru;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/

extern plane_cmd0_packet_stru plane_cmd0_packet;

//拍照录像
typedef struct 
{
    uint8_t    photo_record;                //0x01表示拍照，0x02开始录像，0x03停止录像(需要时发)
}plane_cmd1_packet_stru;

extern plane_cmd1_packet_stru plane_cmd1_packet;

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
                                //bit4:  起飞中 0：未开启 1：开启中
                                //bit5： 0 tbd            1 tbd    
                                //bit6： 0 正常模式       1无头模式
                                //bit7： 0 正常模式       1急停模式     
            
    uint8_t     plane_status3;  //bit0：  0低速模式    1高速模式
                                //bit1:   0正常模式    1低电模式
                                //bit2:   0正常模式    1新手模式
                                //bit3:   0 姿态模式 1：gps模式
                                //bit4:   0
                                //bit5:   0
                                //bit6:   0
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
}plane_cmd2_packet_stru;
extern plane_cmd2_packet_stru plane_cmd2_packet;

//plane_cmd2_packet.plane_status1操作
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

//plane_cmd2_packet.plane_status2操作
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

//plane_cmd2_packet.plane_status3操作
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

//plane_cmd2_packet.plane_status4操作
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
//plane_cmd2_packet.plane_status5操作
void update_plane_cmd2_photo_function_to_wifi(void);
void update_plane_cmd2_video_function_to_wifi(void);

void set_plane_cmd2_photo_ev_inc(void) ;
void clear_plane_cmd2_photo_ev_inc(void);
void set_plane_cmd2_photo_ev_dec(void) ;
void clear_plane_cmd2_photo_ev_dec(void);



//飞机返回当前设置数据
typedef struct 
{
    uint8_t     fly_altitude_limit;         //限制飞行高度(10---120m)
    uint8_t     fly_distance_limit;         //限制飞行距离 (10---250)*2默认35m1字节(单位2m最小变化2m显示20---500，实际发送数据(10-250) 
    uint8_t     return_altitude_limit;      //返航高度(10---130)
    uint8_t     control_mode;               //新手模式0xff 正常模式0x00  
    uint8_t     stick_mode;                 //摇杆模式 0：美国手，1：日本手
    uint16_t    circle_radius;              //绕飞半径
}plane_cmd3_packet_stru;
extern plane_cmd3_packet_stru plane_cmd3_packet;
 
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
}plane_cmd4_packet_stru;
extern plane_cmd4_packet_stru plane_cmd4_packet;

//请求设置数据，飞机的飞行最大距离，最大高度，返航高度，
//当前是否为新手模式（一直发送直到收到0x01收到设置数据）
typedef struct 
{
    uint8_t     setting_request;             //0x00 发设置请求，0x01:收到设置数据，手机设置完成
}plane_cmd5_packet_stru;
extern plane_cmd5_packet_stru plane_cmd5_packet;

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
}plane_cmd6_packet_stru;
extern plane_cmd6_packet_stru plane_cmd6_packet;

void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd);

//升级响应
typedef struct 
{
    uint8_t     from_device;        //来源设备
    uint8_t     to_device;          //目标设置
}plane_cmd9_packet_stru;
extern plane_cmd9_packet_stru plane_cmd9_packet;

//wifi->plane数据协议定义
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

//手机gps 指南针数据
typedef struct 
{
    int32_t     phone_latitude;             //手机纬度 int32/1e-7(0x xxxxxxxx度)
    int32_t     phone_longitude;            //手机经度
    uint16_t    phone_gps_accuracy;         //手机GPS精度
    uint16_t    phone_compass_direction;    //手机指南针方向 
    uint16_t    follow_altitude;            //跟随高度*10 精度0.1M
    uint16_t    follow_me_flag;             //没有跟随时发送0x0000,跟随时发送0xffff，200ms发送一次进入控制界面一直发送
}wifi_cmd0_packet_stru;
extern wifi_cmd0_packet_stru wifi_cmd0_packet;

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
}wifi_cmd1_packet_stru;
extern wifi_cmd1_packet_stru wifi_cmd1_packet;

//手机控制数据包
typedef struct 
{
    uint8_t     control_info;               //控制信息
    uint8_t     calibrate_info;             //校准信息
    uint8_t     camera_info;                //相机信息
}wifi_cmd1_packet_backup_stru;

uint16_t get_phone_control_follow_mode(void);     //0xffff 开启跟随，0x0000 未开启
uint8_t get_phone_control_speed_mode(void); //大小档模式，0/1/2，低中高
uint8_t get_phone_control_roll_mode(void);  //翻滚模式，1:有效
void clear_phone_control_roll_mode(void);
uint8_t get_phone_control_return_home_mode(void);   //返航模式，1:有效
void clear_phone_control_return_home_mode(void);
uint8_t get_phone_control_headless_mode(void);    //无头模式，1:有效
void clear_phone_control_headless_mode(void);
uint8_t get_phone_control_scram_mode(void);     //急停模式，1:有效
void clear_phone_control_scram_mode(void);
uint8_t get_phone_control_rising_landing(void);  //一键启动/着陆，1:有效
void clear_phone_control_rising_landing(void);
//uint8_t get_phone_control_landing(void);
uint8_t get_phone_control_compass_calibrate(void); //地磁校准，1:有效
void clear_phone_control_compass_calibrate(void);
uint8_t get_phone_control_level_calibrate(void);//水平校准，1:有效
void clear_phone_control_level_calibrate(void);
uint8_t get_phone_control_lock_info(void);//解锁，1:有效
void clear_phone_control_lock_info(void);
uint8_t get_phone_control_hot_spot_around(void);//热点环绕，1:有效
void clear_phone_control_circle(void);
uint8_t get_phone_control_point_fly(void);//指点飞行，1:有效
void clear_phone_control_point_fly(void);
uint8_t get_phone_control_altitude_info(void);  //定高标志，1:有效
void clear_phone_control_altitude_info(void);
uint8_t get_phone_control_one_key_calibrate_flag(void); //一键校准，1:有效
void clear_phone_control_one_key_calibrate_flag(void);
uint8_t get_phone_control_camera_up_left(void); //摄像头上升或向左标志，1:有效
uint8_t get_phone_control_camera_down_right(void); //摄像头下降或向右标志，1:有效
uint8_t get_phone_control_led(void); //灯控制位，初始位0，再按为1，再按为2，0/1/2 循环
//获取摇杆值
uint16_t get_throttle_value(void);  
uint16_t get_aileron_value(void);
uint16_t get_elevator_value(void);
uint16_t get_rudder_value(void);
uint16_t get_yuntai_value(void);

//灯 云台控制
typedef struct 
{
    uint8_t     function;                   //0x00（开关灯） 功能0x01(云台控制)
    uint8_t     function_value;             //0x00:关灯 0x01：开灯（碧盛禾）  0x00云台向上，0x01云台向下  
}wifi_cmd2_packet_stru;
extern wifi_cmd2_packet_stru wifi_cmd2_packet;

//飞机设置（新手模式发送默认数据，不可改变大小）
typedef struct 
{
    uint8_t     altitude_limit_setting;     //限制飞行高度(10---120m)默认30m
    uint8_t     distance_limit_setting;     //限制飞行距离 (10---250)*2默认30m1字节(单位2m最小变化2m显示20---500，实际发送数据(10-250) 
    uint8_t     return_altitude_setting;    //返航高度(10---130)    默认30m  1字节//返航高度小于最大飞行高度
    uint8_t     green_hands_setting;        //新手模式新手模式0xff 正常模式0x00
    uint8_t     stick_mode;                 //摇杆模式 0：美国手，1：日本手
    uint16_t    circle_radius;              //绕飞半径
}wifi_cmd3_packet_stru;
extern wifi_cmd3_packet_stru wifi_cmd3_packet;

//app发送请求设置数据
typedef struct 
{
    uint16_t     get_plane_setting_data;     //app链接飞机时，发送0xffff获取当前飞机设置数据 
}wifi_cmd4_packet_stru;
extern wifi_cmd4_packet_stru wifi_cmd4_packet;

//校准 解锁控制
typedef struct 
{
    uint8_t     control_info;               //Bit0: 默认0， 1磁场校准
                                            //Bit1: 默认0，1水平校准
                                            //Bit2: 默认0，1解锁
}wifi_cmd5_packet_stru;
extern wifi_cmd5_packet_stru wifi_cmd5_packet;

uint8_t get_phone_control_compass_calibrate2(void); //默认0，磁场校准为1
uint8_t get_phone_control_level_calibrate2(void); //默认0，水平校准为1
uint8_t get_phone_control_lock_info2(void); //默认0，解锁为1

//航点飞行（多点）：
#pragma pack (1) /*指定按1字节对齐*/
typedef struct 
{
    uint8_t     waypoint_number;            //航点数 1---32
    uint32_t    longitude_latitude[32*2];   //航点的经纬度 注意纬度在前。经度在后 需要指点地图经纬度(wgs84坐标) (大端) 4字节(int32/1e-7(0x xxxxxxxx度))
                                            //注：一个坐标信息（经纬度）8byte, 总共（最大32个点）256byte, 有则赋值，没则为空，长度固定，
                                            //飞行时距离大于500米或小于一定范围时飞机需要返回提示不适合飞行，发送坐标之后飞机返回提示开始飞行，
                                            //到达目标地点时飞机可以提示指点飞行完成，正在飞行时再次发送坐标相当于停止指点飞行飞机提示飞行未完成，摇杆操作也可以使飞机停止当前指点飞行
                                            //发送数据8次（间隔100ms）前四次校验码前一位为0xff 后四次校验码前一位为0x00开始一次指点飞行
}wifi_cmd6_packet_stru;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
extern wifi_cmd6_packet_stru wifi_cmd6_packet;


//绕点飞行：绕点的半径2字节,点击（确定按钮）连续发送八次间隔100ms 前四次校验码前一位为0xff 后四次校验码前一位为0x00开始一次绕点飞行
//再次点击环绕按钮，则为取消环绕同样发送上面的 数据8次环绕按钮状态改变 例：fffd05070005fff8  fffd0507000500f8
typedef struct 
{ 
    uint16_t        radius;                              
}wifi_cmd7_packet_stru;
extern wifi_cmd7_packet_stru wifi_cmd7_packet;

#define SYNCHRONOUS_CMD     0XAA55
//同步请求
typedef struct 
{
    uint16_t    synchronous_request;        //同步请求                                       
}wifi_cmd8_packet_stru;
extern wifi_cmd8_packet_stru wifi_cmd8_packet;

//同步请求
typedef struct 
{
    uint8_t     from_device;        //来源设备
    uint8_t     to_device;          //目标设置
}wifi_cmd9_packet_stru;
extern wifi_cmd9_packet_stru wifi_cmd9_packet;


typedef enum
{
    ROLLING_MODE = 1<<2,        //翻滚模式
    RETURN_HOME_MODE = 1<<3,    //返航模式
    HEADLESS_MODE = 1<<4,       //无头模式
    SCRAM_MODE = 1<<5,          //急停模式
    RISGING_LANDING_MODE = 1<<6,//一键起飞/着陆模式
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

//飞机控制功能存储寄存器
typedef struct 
{
    uint16_t w_lose_control_timer;
    uint8_t by_lose_control;   
    uint8_t by_control_info;    //Bit 0：1	大小档模式，0/1/2，低中高
                                //Bit2	翻滚模式，按下为1，再按为0
                                //Bit3	返航模式，按下为1，再按为0
                                //Bit4	无头模式，按下为1，再按为0
                                //Bit5	急停模式，按下为1，再按为0
                                //Bit6	一键启动/着陆，按下为1，再按为0
                                //Bit7	一键着陆，按下为1，再按为0
    uint8_t by_calibrate_info;  //Bit0	地磁校准，按下为1，再按为0
                                //Bit1	水平校准，按下为1，再按为0
                                //Bit2	解锁，按下为1，再按为0
                                //Bit3	热点环绕，按下为1，再按为0
                                //Bit4	指点飞行，按下为1，再按为0
                                //Bit5	一键校准，按下为1，再按为0
                                //Bit6	定高标志，按下为1，再按为0
                                //Bit7  
    uint8_t by_camero_info;     //Bit0	拍照，按下为1，再按为0（bit7 = 1 时（remoter）有效）
                                //Bit1	录像，按下为1，再按为0（bit7 = 1 时（remoter）有效）
                                //Bit2	拍照状态反馈，拍照为1，否则为0
                                //Bit3	录像状态反馈，录像开启为1，关闭为0
                                //Bit4	摄像头下降或向右标志，按下为1，松开为0
                                //Bit5	摄像头上升或向左标志，按下为1，松开为0
                                //Bit6	
                                //Bit7	0：phone，1：remoter
}plane_function_control_info_stru;

extern plane_function_control_info_stru m_plane_function_control_info;

typedef enum
{
    STATUS_OFF = 0,
    STATUS_ON = 1,
}status_def;

//飞控算法反馈状态
typedef enum
{
    PLANE_LOCK_STATUS  = 0,                                     //0：锁   1：解锁
    PLANE_FLYING_STATUS = 1,                                    //0：停飞  1：飞行中
    PLANE_GYRO_CALIBRATE_STATUS = 2,                            //1：水平校准中
    PLANE_COMPASS_HORIZONTAL_CALIBRATE_STATUS = 3,              //1：地磁水平校准 horizontal
    PLANE_COMPASS_VERTICAL_CALIBRATE_STATUS = 4,                //1：地磁垂直校准 vertical
    PLANE_FOLLOW_MODE_STATUS = 5,                               //跟随模式 0：未开   1：开启
    PLANE_CIRCLE_MODE_STATUS = 6,                               //环绕模式 0：未开   1：开启
    PLANE_POINT_MODE_STATUS = 7,                                //指点飞行模式 0：未开   1：开启
    PLANE_RETURN_HOME_MODE_STATUS = 8,                          //返航中 1为开启 0为未开启
    PLANE_LANDING_STATUS = 9,                                   //降落 0：未开   1：开启
    PLANE_LINK_REMOTE_STATUS = 10,                              //0遥控器未连接    1遥控器已连接（遥控器链接时app不要摇杆）
    PLANE_HEADLESS_MODE_STATUS = 11,                            //0正常模式    1无头模式
    PLANE_SCRAM_MODE_STATUS = 12,                               //0正常模式    1急停模式
    PLANE_SPEED_MODE_STATUS = 13,                               //0低速模式    1高速模式
    PLANE_LOW_POWER_MODE_STATUS = 14,                           //0正常模式    1低电模式
    PLANE_GREEN_HANDS_MODE_STATUS = 15,                         //0正常模式    1新手模式
//    PLANE_FRONT_LEFT_MOTOR_OVERLOAD_STATUS = 16,                //0飞机前左电机保护    1飞机前左电机过载
//    PLANE_FRONT_RIGHT_MOTOR_OVERLOAD_STATUS = 17,               //0飞机前右电机保护    1飞机前右电机过载
//    PLANE_BACK_LEFT_MOTOR_OVERLOAD_STATUS = 18,                 //0飞机后左电机保护    1飞机后左电机过载
//    PLANE_BACK_RIGHT_MOTOR_OVERLOAD_STATUS = 19,                //0飞机后右电机保护    1飞机后右电机过载
    PLANE_MPU_ERRO_STATUS = 20,                                 //0 MPU传感器正常      1 MPU传感器错误
    PLANE_BARO_ERRO_STATUS = 21,                                //0 气压计正常         1 气压计错误
    PLANE_GPS_ERRO_STATUS = 22,                                 //0 GPS连接正常        1 GPS连接错误
    PLANE_COMPASS_ERRO_STATUS = 23,                             //0 地磁干扰正常       1 地磁干扰过大
    PLANE_WIND_ERRO_STATUS = 24,                                //0 风小               1 风大警告
    PLANE_GPS_RSSI_STATUS = 25,                                 //0 GPS 信号好         1 GPS 信号弱警告   
    PLANE_REMOTE_CHOICE_STATUS = 26,							//0 中继遥控器           1 2.4G遥控器
    PLANE_RISING_STATUS = 27,									//一键起飞 0：未开    1：开启
    PLANE_IMU_GPS_MODE_STATUS = 28,                             //0:姿态模式 1：GPS模式

}plane_status_def;

//飞控算法反馈状态
typedef enum
{
    PLANE_LOCK_FLAG_STATUS = 1<<0,                              //0：锁   1：解锁
    PLANE_FLYING_FLAG_STATUS = 1<<1,                            //0：停飞  1：飞行中
    PLANE_GYRO_CALIBRATE_FLAG_STATUS = 1<<2,                    //1：校准中
    PLANE_COMPASS_HORIZONTAL_CALIBRATE_FLAG_STATUS = 1<<3,      //1：地磁水平校准 horizontal
    PLANE_COMPASS_VERTICAL_CALIBRATE_FLAG_STATUS = 1<<4,        //1：地磁垂直校准 vertical
    PLANE_FOLLOW_MODE_FLAG_STATUS = 1<<5,                       //跟随模式 0：未开   1：开启
    PLANE_CIRCLE_MODE_FLAG_STATUS = 1<<6,                       //环绕模式 0：未开   1：开启
    PLANE_POINT_MODE_FLAG_STATUS = 1<<7,                        //指点飞行模式 0：未开   1：开启
    PLANE_RETURN_HOME_MODE_FLAG_STATUS = 1<<8,                  //返航中 1为开启 0为未开启
    PLANE_LANDING_FLAG_STATUS = 1<<9,                           //降落 0：未开   1：开启
    PLANE_LINK_REMOTE_FLAG_STATUS = 1<<10,                       //0遥控器未连接    1遥控器已连接（遥控器链接时app不要摇杆）
    PLANE_HEADLESS_MODE_FLAG_STATUS = 1<<11,                    //0正常模式    1无头模式
    PLANE_SCRAM_MODE_FLAG_STATUS = 1<<12,                       //0正常模式    1急停模式
    PLANE_SPEED_MODE_FLAG_STATUS = 1<<13,                       //0低速模式    1高速模式
    PLANE_LOW_POWER_MODE_FLAG_STATUS = 1<<14,                   //0正常模式    1低电模式
    PLANE_GREEN_HANDS_MODE_FLAG_STATUS = 1<<15,                 //0正常模式    1新手模式
//    PLANE_FRONT_LEFT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<16,        //0飞机前左电机保护    1飞机前左电机过载
//    PLANE_FRONT_RIGHT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<17,       //0飞机前右电机保护    1飞机前右电机过载
//    PLANE_BACK_LEFT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<18,         //0飞机后左电机保护    1飞机后左电机过载
//    PLANE_BACK_RIGHT_MOTOR_OVERLOAD_FLAG_STATUS = 1<<19,        //0飞机后右电机保护    1飞机后右电机过载
    PLANE_MPU_ERRO_FLAG_STATUS = 1<<20,                         //0 MPU传感器正常      1 MPU传感器错误
    PLANE_BARO_ERRO_FLAG_STATUS = 1<<21,                        //0 气压计正常         1 气压计错误
    PLANE_GPS_ERRO_FLAG_STATUS = 1<<22,                         //0 GPS连接正常        1 GPS连接错误
    PLANE_COMPASS_ERRO_FLAG_STATUS = 1<<23,                     //0 地磁干扰正常       1 地磁干扰过大
    PLANE_WIND_ERRO_FLAG_STATUS = 1<<24,                        //0 风小               1 风大警告
    PLANE_GPS_RSSI_FLAG_STATUS = 1<<25,                         //0 GPS 信号好         1 GPS 信号弱警告
    PLANE_REMOTE_CHOICE_FLAG_STATUS = 1<<26,					//0 中继遥控器           1 2.4G遥控器
    PLANE_RISING_FLAG_STATUS = 1<<27,							//一键起飞 0：未开    1：开启
    PLANE_IMU_GPS_MODE_FLAG_STATUS = 1<<28,                     //0:姿态模式 1：GPS模式
    
}plane_status_flag_def;

extern uint32_t g_dw_plane_status;      //飞机状态


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
void wifi_function_dispose(void);
int erase_iap_id(void);

void update_plane_status_now(plane_status_def plane_function,status_def status);

//获取飞机各功能的当前状态
status_def get_plane_status_now(plane_status_def plane_function);



//
// 获取8bit校验和
uint8_t get_u8data_checksum(uint8_t *pbuf,uint8_t len);

#pragma pack (1) /*指定按2字节对齐*/
typedef struct FLASH_STRU
{   
    uint8_t     updata_flag[4];
    uint8_t     rfID[8];
    uint8_t     remote_version[4];
    uint8_t     FlyLimitSettingData[8];
    uint16_t    store_flag;
    uint8_t     check_sum;      //位置不能动，必须放在结构最后面
}FLASH_DEF;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
extern FLASH_DEF flash;

#define FLASH_DATA_BUFFER_LEN  sizeof(flash) 

#endif



