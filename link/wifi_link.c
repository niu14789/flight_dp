#include <stdio.h>
#include "gd32f30x.h"
//#include "gd32f307c_eval.h"
#include "wifi_link.h"
#include "string.h"
#include "rf_link.h"
#include "SerialPort.h"
#include "parameter.h"
#include "adc_driver.h"
#include "Variables.h"

plane_cmd0_packet_def plane_cmd0_packet;
plane_cmd1_packet_def plane_cmd1_packet;
plane_cmd2_packet_def plane_cmd2_packet;
plane_cmd3_packet_def plane_cmd3_packet;
plane_cmd4_packet_def plane_cmd4_packet;
plane_cmd5_packet_def plane_cmd5_packet;
plane_cmd6_packet_def plane_cmd6_packet;

wifi_cmd0_packet_def wifi_cmd0_packet;
wifi_cmd1_packet_def wifi_cmd1_packet;
wifi_cmd1_packet_def wifi_cmd1_packet_backup;
wifi_cmd2_packet_def wifi_cmd2_packet;
wifi_cmd3_packet_def wifi_cmd3_packet;
wifi_cmd4_packet_def wifi_cmd4_packet;
wifi_cmd5_packet_def wifi_cmd5_packet;
wifi_cmd5_packet_def wifi_cmd5_packet_backup;
wifi_cmd6_packet_def wifi_cmd6_packet;
wifi_cmd7_packet_def wifi_cmd7_packet;
wifi_cmd8_packet_def wifi_cmd8_packet;

SerialPortHandle *g_poHandle;
void wifi_link_init(void)
{  
    g_poHandle = UsartOpen(USART_PORT1);  
    if(false == read_fly_limit_setting_data())
    {
        wifi_cmd3_packet.altitude_limit_setting = 30;
        wifi_cmd3_packet.distance_limit_setting = 100;
        wifi_cmd3_packet.return_altitude_setting = 30;
        wifi_cmd3_packet.green_hands_setting= NORMAL_MODE;
        wifi_cmd3_packet.stick_mode = AMERICA_HAND;
    }
}

//访问频率1000hz
void wifi_link_data_receive(void)
{
    int iRet;    
    static U8 s_abyReceive[256];
    static U8 s_byRecvDataLength;
    static U8 s_byRecvDataStep = 0;
    
    iRet = UsartGetRecvBufferlen(g_poHandle);    
    switch(s_byRecvDataStep)
    {
        case 0:
            if (iRet)
            {
                iRet = UsartRecvNoBlocking(g_poHandle, s_abyReceive, 1);
                if (iRet > 0)
                {                    
                    if ( WIFI_PACKET_HEAD_H == s_abyReceive[0])
                    {
                        s_byRecvDataStep ++;
                    }
                }
            }
            break;
        case 1:
            if (iRet)
            {
                iRet = UsartRecvNoBlocking(g_poHandle, &s_abyReceive[1], 1);      
                if ( iRet > 0)
                {
                    if ( WIFI_PACKET_HEAD_L == s_abyReceive[1])
                    {
                        s_byRecvDataStep ++;
                    }
                    else
                    {
                        s_byRecvDataStep = 0;
                    }
                }
            }
            break;
        case 2:
            if (iRet)
            {   
                iRet = UsartRecvNoBlocking(g_poHandle, &s_abyReceive[2], 1);      
                if ( iRet > 0)
                {
                    s_byRecvDataLength = s_abyReceive[2];
                    if( 0 == s_byRecvDataLength)
                    {
                        s_byRecvDataLength = 0;
                    }
                    else
                    {
                        s_byRecvDataStep ++;
                    }
                }  
            }
            break;
        case 3:
            if (iRet >= s_byRecvDataLength)
            {
                iRet = UsartRecvNoBlocking(g_poHandle, &s_abyReceive[3], s_byRecvDataLength); 
                if (iRet > 0)
                {
                    wifi_data_packet_decode(s_abyReceive);
                    s_byRecvDataStep = 0;                    
                }
            }                
            break;
        default:
            break;
    } 
}

static bool f_cmd2_send_flag = false;
static bool f_cmd3_send_flag = false;
static bool f_cmd6_send_flag = false;
//访问频率1000hz
void wifi_link_data_send(void)
{
    static U8 s_abySend[256];
    U8 bySendFlag = 0;  
    U8 bySendDataLength = 0; 
    static U32 wSendTimer = 0; 
    
    if ( SYNCHRONOUS_CMD ==  wifi_cmd8_packet.synchronous_request)
    {
        wifi_cmd8_packet.synchronous_request = 0;
        f_cmd3_send_flag = true;
        f_cmd6_send_flag = true;
    }

    wSendTimer ++;
    
    //有数据为发完退出
    if (SEND_FINISHED != UsartGetSendStatus(g_poHandle))
        return;     
    
    // cmd0 数据500ms 发送一次
    if (0 == (wSendTimer % 500))
    {   
        plane_send_data_packet(s_abySend,CMD0); 
        bySendDataLength = CMD0_PACKET_LEN+5;
        bySendFlag = 1;
    }
      // cmd1 数据20ms 发送一次
//    else if (20 == (wSendTimer % 500))
//    {   
//        plane_send_data_packet(s_abySend,CMD1);
//        bySendDataLength = CMD1_PACKET_LEN+5;        
//        bySendFlag = 1;
//    } 
    // cmd2 数据500ms 发送一次
    else if (40 == (wSendTimer % 500) && (true == f_cmd2_send_flag))
    {   
        f_cmd2_send_flag = false;
        plane_send_data_packet(s_abySend,CMD2);   
        bySendDataLength = CMD2_PACKET_LEN+5;
        bySendFlag = 1;
    } 
    // cmd3 数据
    else if ((60 == (wSendTimer % 500)) && (true == f_cmd3_send_flag))
    {   
        f_cmd3_send_flag = false;
        plane_send_data_packet(s_abySend,CMD3);
        bySendDataLength = CMD3_PACKET_LEN+5;        
        bySendFlag = 1;
    }       
    // cmd6 数据
    else if ((120 == (wSendTimer % 500)) && (true == f_cmd6_send_flag))
    {   
        f_cmd6_send_flag = false;
        plane_send_data_packet(s_abySend,CMD6);
        bySendDataLength = CMD6_PACKET_LEN+5;        
        bySendFlag = 1;
    } 
    if (bySendFlag)
    {
        bySendFlag = 0;
        UsartSendNoBlocking(g_poHandle, s_abySend, bySendDataLength);
    }   
}

//plane_cmd2_packet.plane_status1操作
void set_plane_lock_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT0; }
void set_plane_unlock_status(void) { plane_cmd2_packet.plane_status1 |= BIT0; }
uint8_t get_plane_lock_status(void) { return (plane_cmd2_packet.plane_status1 & BIT0);}

void set_plane_stop_fly_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT1; }
void set_plane_flying_status(void) { plane_cmd2_packet.plane_status1 |= BIT1; }
uint8_t get_plane_flying_status(void) { return (plane_cmd2_packet.plane_status1 & BIT1); }

void set_plane_no_gps_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT2; }
void set_plane_got_gps_status(void) { plane_cmd2_packet.plane_status1 |= BIT2;}
uint8_t get_plane_gps_status(void) { return (plane_cmd2_packet.plane_status1 & BIT2); }

void clear_plane_follow_mode(void) { plane_cmd2_packet.plane_status1 &= ~BIT3; }
void set_plane_follow_mode(void) { plane_cmd2_packet.plane_status1 |= BIT3; }
uint8_t get_plane_follow_mode_status(void) { return (plane_cmd2_packet.plane_status1 & BIT3); }

void clear_plane_circle_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT4; }
void set_plane_circle_mode(void) {plane_cmd2_packet.plane_status1 |= BIT4; }
uint8_t get_plane_circle_mode_status(void) { return (plane_cmd2_packet.plane_status1 & BIT4); }

void clear_plane_point_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT5; }
void set_plane_point_mode(void) {plane_cmd2_packet.plane_status1 |= BIT5; }
uint8_t get_plane_point_mode_status(void) { return (plane_cmd2_packet.plane_status1 & BIT5); }

void clear_plane_return_noome_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT6; }
void set_plane_return_home_mode(void) {plane_cmd2_packet.plane_status1 |= BIT6; }
uint8_t get_plane_return_home_mode_status(void) { return (plane_cmd2_packet.plane_status1 & BIT6); }

void clear_plane_landing_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT7; }
void set_plane_landing_mode(void) {plane_cmd2_packet.plane_status1 |= BIT7; }
uint8_t get_plane_landing_mode_status(void) { return (plane_cmd2_packet.plane_status1 & BIT7); }

//plane_cmd2_packet.plane_status2操作
void clear_plane_gyro_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT0; }
void set_plane_gyro_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT0; }

void clear_plane_compass_horizontal_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT1; }
void set_plane_compass_horizontal_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT1; }
void clear_plane_compass_vertical_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT2; }
void set_plane_compass_vertical_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT2; }
void set_plane_unlink_remoter_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT3; }
void set_plane_link_remoter_status(void) { plane_cmd2_packet.plane_status2 |= BIT3; }
void clear_plane_nohead_mode(void) { plane_cmd2_packet.plane_status2 &= ~BIT6; }
void set_plane_nohead_mode(void) { plane_cmd2_packet.plane_status2 |= BIT6; }
uint8_t get_plane_nohead_mode_status(void) { return (plane_cmd2_packet.plane_status2 & BIT6); }

void clear_plane_scram_mode(void) { plane_cmd2_packet.plane_status2 &= ~BIT7; }
void set_plane_scram_mode(void) { plane_cmd2_packet.plane_status2 |= BIT7; }
//plane_cmd2_packet.plane_status3操作
void set_plane_low_speed_status(void) { plane_cmd2_packet.plane_status3 &= ~BIT0; }
void set_plane_high_speed_status(void) { plane_cmd2_packet.plane_status3 |= BIT0; }
void clear_plane_low_voltage_status(void) { plane_cmd2_packet.plane_status3 &= ~BIT1; }
void set_plane_low_voltage_status(void) { plane_cmd2_packet.plane_status3 |= BIT1; }
uint8_t get_plane_low_voltage_status(void) { return (plane_cmd2_packet.plane_status3 & BIT1); }

void clear_plane_green_hands_mode(void) { plane_cmd2_packet.plane_status3 &= ~BIT2; }
void set_plane_green_hands_mode(void) { plane_cmd2_packet.plane_status3 |= BIT2; }
void clear_plane_front_left_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT3; }
void set_plane_front_left_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT3; }
void clear_plane_front_right_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT4; }
void set_plane_front_right_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT4; }
void clear_plane_back_right_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT5; }
void set_plane_back_right_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT5; }
void clear_plane_back_left_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT6; }
void set_plane_back_left_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT6; }

//plane_cmd2_packet.plane_status4操作
void clear_plane_mpu_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT0; }
void set_plane_mpu_erro(void) { plane_cmd2_packet.plane_status4 |= BIT0; }
void clear_plane_baro_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT1; }
void set_plane_baro_erro(void) { plane_cmd2_packet.plane_status4 |= BIT1; }
void clear_plane_gps_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT2; }
void set_plane_gps_erro(void) { plane_cmd2_packet.plane_status4 |= BIT2; }
void clear_plane_compass_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT3; }
void set_plane_compass_erro(void) { plane_cmd2_packet.plane_status4 |= BIT3; }
void clear_plane_wind_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT4; }
void set_plane_wind_erro(void) { plane_cmd2_packet.plane_status4 |= BIT4; }
void clear_plane_gps_rssi_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT5; }
void set_plane_gps_rssi_erro(void) { plane_cmd2_packet.plane_status4 |= BIT5; }

void update_plane_photo_function_to_wifi(void) { plane_cmd2_packet.plane_status5 ^= BIT0; }
void update_plane_video_function_to_wifi(void) { plane_cmd2_packet.plane_status5 ^= BIT1; }

//wifi_cmd1_packet.control_info 操作
uint8_t get_phone_control_speed_mode(void) { return (wifi_cmd1_packet.control_info & 0x03); }
uint8_t get_phone_control_roll_mode(void) { return (wifi_cmd1_packet.control_info & BIT2); }
uint8_t get_phone_control_return_home_mode(void) { return (wifi_cmd1_packet.control_info & BIT3); }
uint8_t get_phone_control_nohead_mode(void) { return (wifi_cmd1_packet.control_info & BIT4); }
uint8_t get_phone_control_scram_mode(void) { return (wifi_cmd1_packet.control_info & BIT5); }
uint8_t get_phone_control_start_landing(void) { return (wifi_cmd1_packet.control_info & BIT6); }
//uint8_t get_phone_control_landing(void) { return (wifi_cmd1_packet.control_info & BIT7); }
//wifi_cmd1_packet.led_control 操作
uint8_t get_phone_control_led(void) { return (wifi_cmd1_packet.led_control & (BIT7|BIT6)); }
//wifi_cmd1_packet.calibrate_info 操作
uint8_t get_phone_control_compass_calibrate(void) { return (wifi_cmd1_packet.calibrate_info & BIT0); } //地磁校准，按下为1，1秒后为0
uint8_t get_phone_control_level_calibrate(void) { return (wifi_cmd1_packet.calibrate_info & BIT1); } //水平校准，按下为1，1秒后为0
uint8_t get_phone_control_lock_info(void) { return (wifi_cmd1_packet.calibrate_info & BIT2); } //解锁，按下为1，1秒后为0
uint8_t get_phone_control_hot_spot_around(void) { return (wifi_cmd1_packet.calibrate_info & BIT2); } //热点环绕，按下为1，再按为0 
uint8_t get_phone_control_point_fly(void) { return (wifi_cmd1_packet.calibrate_info & BIT2); } //指点飞行，按下为1，再按为0
uint8_t get_phone_control_one_key_calibrate_flag(void) { return (wifi_cmd1_packet.calibrate_info & BIT5); }
uint8_t get_phone_control_altitude_info(void) { return (wifi_cmd1_packet.calibrate_info & BIT6); }
//wifi_cmd1_packet.camera_info 操作
uint8_t get_camera_photo_status(void)
{
    return (wifi_cmd1_packet.camera_info & BIT2);
}
uint8_t get_camera_video_status(void)
{
    return (wifi_cmd1_packet.camera_info & BIT3);
}
uint8_t get_phone_control_camera_down_right(void) { return (wifi_cmd1_packet.camera_info & BIT4); }
uint8_t get_phone_control_camera_up_left(void) { return (wifi_cmd1_packet.camera_info & BIT5); }
//wifi_cmd5_packet.control_info 操作
uint8_t get_phone_control_compass_calibrate2(void) {return (wifi_cmd5_packet.control_info & BIT0); }
uint8_t get_phone_control_level_calibrate2(void) {return (wifi_cmd5_packet.control_info & BIT1); }
uint8_t get_phone_control_lock_info2(void) {return (wifi_cmd5_packet.control_info & BIT2); }


//wifi_cmd1_packet_backup.control_info 操作
uint8_t get_phone_control_speed_mode_backup(void) { return (wifi_cmd1_packet_backup.control_info & 0x03); }
uint8_t get_phone_control_roll_mode_backup(void) { return (wifi_cmd1_packet_backup.control_info & BIT2); }
uint8_t get_phone_control_return_home_mode_backup(void) { return (wifi_cmd1_packet_backup.control_info & BIT3); }
uint8_t get_phone_control_nohead_mode_backup(void) { return (wifi_cmd1_packet_backup.control_info & BIT4); }
uint8_t get_phone_control_scram_mode_backup(void) { return (wifi_cmd1_packet_backup.control_info & BIT5); }
uint8_t get_phone_control_start_landing_backup(void) { return (wifi_cmd1_packet_backup.control_info & BIT6); }
//uint8_t get_phone_control_landing_backup(void) { return (wifi_cmd1_packet_backup.control_info & BIT7); }
//wifi_cmd1_packet_backup.led_control 操作
uint8_t get_phone_control_led_backup(void) { return (wifi_cmd1_packet_backup.led_control & (BIT7|BIT6)); }
//wifi_cmd1_packet_backup.calibrate_info 操作
uint8_t get_phone_control_compass_calibrate_backup(void) { return (wifi_cmd1_packet_backup.calibrate_info & BIT0); } //地磁校准，按下为1，1秒后为0
uint8_t get_phone_control_level_calibrate_backup(void) { return (wifi_cmd1_packet_backup.calibrate_info & BIT1); } //水平校准，按下为1，1秒后为0
uint8_t get_phone_control_lock_info_backup(void) { return (wifi_cmd1_packet_backup.calibrate_info & BIT2); } //解锁，按下为1，1秒后为0
uint8_t get_phone_control_hot_spot_around_backup(void) { return (wifi_cmd1_packet_backup.calibrate_info & BIT2); } //热点环绕，按下为1，再按为0 
uint8_t get_phone_control_point_fly_backup(void) { return (wifi_cmd1_packet_backup.calibrate_info & BIT2); } //指点飞行，按下为1，再按为0
uint8_t get_phone_control_one_key_calibrate_flag_backup(void) { return (wifi_cmd1_packet_backup.calibrate_info & BIT5); }
uint8_t get_phone_control_altitude_info_backup(void) { return (wifi_cmd1_packet_backup.calibrate_info & BIT6); }
//wifi_cmd1_packe_backupt.camera_info 操作
uint8_t get_camera_photo_status_backup(void)
{
    return (wifi_cmd1_packet_backup.camera_info & BIT2);
}
uint8_t get_camera_video_status_backup(void)
{
    return (wifi_cmd1_packet_backup.camera_info & BIT3);
}
uint8_t get_phone_control_camera_down_right_backup(void) { return (wifi_cmd1_packet_backup.camera_info & BIT4); }
uint8_t get_phone_control_camera_up_left_backup(void) { return (wifi_cmd1_packet_backup.camera_info & BIT5); }

//wifi_cmd5_packet.control_info 操作
uint8_t get_phone_control_compass_calibrate2_backup(void) {return (wifi_cmd5_packet_backup.control_info & BIT0); }
uint8_t get_phone_control_level_calibrate2_backup(void) {return (wifi_cmd5_packet_backup.control_info & BIT1); }
uint8_t get_phone_control_lock_info2_backup(void) {return (wifi_cmd5_packet_backup.control_info & BIT2); }

//更新机头方向至APP
void updata_plane_heading_to_wifi(uint16_t wheading)
{
    plane_cmd0_packet.heading = wheading;
}
//更新距离至APP
void updata_plane_distance_to_wifi(int16_t sdistance)
{
    plane_cmd0_packet.distance = sdistance;
}
//更新海拔至APP
void updata_plane_altitude_to_wifi(uint16_t saltitude)
{
    plane_cmd0_packet.altitude = saltitude;
}

//飞机发送数据打包
extern SENSOR_RAW_DATA oSensorData;
void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd)
{   
    PARAMETER_SAVE *poParam;    
    uint8_t *ptrd;
    
    poParam = LoadParameter();    
    ptrd = pbuffer;
    
    //最大命令值为6
    if(cmd > 6)
        return;
    
    ptrd[0] = PLANE_PACKET_HEAD_H;
    ptrd[1] = PLANE_PACKET_HEAD_L;

    switch(cmd)
    {
        case    CMD0:
            ptrd[2] = CMD0_PACKET_LEN + 2;   
            ptrd[3] = CMD0;        
            plane_cmd0_packet.plane_voltage = (uint16_t)(get_plane_battery_voltage_value() * 100);
            plane_cmd0_packet.remoter_voltage = (uint16_t)(remote_packet.battery_voltage * 10);    //remote_packet.battery_voltage：为实际电压*10 
            plane_cmd0_packet.gps_longitude = oSensorData.oGpsInfo.oPvt.iLon;
            plane_cmd0_packet.gps_latitude =  oSensorData.oGpsInfo.oPvt.iLat;
            plane_cmd0_packet.gps_NumSv = oSensorData.oGpsInfo.oPvt.byNumSv;
//            plane_cmd0_packet.heading = 0;
//            plane_cmd0_packet.distance = 4520;
//            plane_cmd0_packet.altitude = 540;
            plane_cmd0_packet.horizontal_speed = oSensorData.oGpsInfo.oPvt.iGSpeed;
            plane_cmd0_packet.vertical_speed = oSensorData.oGpsInfo.oPvt.iVelD;
            plane_cmd0_packet.plane_battery_remaining_capacity = 85;
            plane_cmd0_packet.plane_battery_remaining_time = 20;
            plane_cmd0_packet.plane_pitch_angle = -21;
            plane_cmd0_packet.plane_roll_angle = 16;
            plane_cmd0_packet.plane_remain = 0;                    
            memcpy(&ptrd[4],&plane_cmd0_packet,CMD0_PACKET_LEN);           
            break;
        case    CMD1:
            ptrd[2] = CMD1_PACKET_LEN + 2;   
            ptrd[3] = CMD1;            
            memcpy(&ptrd[4],&plane_cmd1_packet,CMD1_PACKET_LEN);
            break;
        case    CMD2:
            ptrd[2] = CMD2_PACKET_LEN + 2;   
            ptrd[3] = CMD2;            
            memcpy(&ptrd[4],&plane_cmd2_packet,CMD2_PACKET_LEN);
            break;
        case    CMD3:
            ptrd[2] = CMD3_PACKET_LEN + 2;   
            ptrd[3] = CMD3; 
            memcpy(&plane_cmd3_packet,&wifi_cmd3_packet,CMD3_PACKET_LEN);
            memcpy(&ptrd[4],&plane_cmd3_packet,CMD3_PACKET_LEN);
            break;
        case    CMD4:
            ptrd[2] = CMD4_PACKET_LEN + 2;   
            ptrd[3] = CMD4;            
            memcpy(&ptrd[4],&plane_cmd4_packet,CMD4_PACKET_LEN);
            break;
        case    CMD5:
            ptrd[2] = CMD5_PACKET_LEN + 2;   
            ptrd[3] = CMD5;            
            memcpy(&ptrd[4],&plane_cmd5_packet,CMD5_PACKET_LEN);
            break;
        case    CMD6:
            ptrd[2] = CMD6_PACKET_LEN + 2;   
            ptrd[3] = CMD6;   
            plane_cmd6_packet.manufacturer_id = MANUFACTURER;
            plane_cmd6_packet.plane_version_h = PLANE_VERSION_HIGH;
            plane_cmd6_packet.plane_version_m = PLANE_VERSION_MEDIAN;
            plane_cmd6_packet.plane_version_l = PLANE_VERSION_LOW;
            plane_cmd6_packet.remoter_version_h = poParam->abyRemoterVersion[0];
            plane_cmd6_packet.remoter_version_m = poParam->abyRemoterVersion[1];
            plane_cmd6_packet.remoter_version_l = poParam->abyRemoterVersion[2];
            plane_cmd6_packet.gimbal_version_h = 2;
            plane_cmd6_packet.gimbal_version_m = 1;
            plane_cmd6_packet.gimbal_version_l = 2;
            plane_cmd6_packet.camera_version_h = 13;
            plane_cmd6_packet.camera_version_m = 24;
            plane_cmd6_packet.camera_version_l = 36;
            plane_cmd6_packet.image_version_h = 6;
            plane_cmd6_packet.image_version_m = 2;
            plane_cmd6_packet.image_version_l = 16;
            memcpy(&ptrd[4],&plane_cmd6_packet,CMD6_PACKET_LEN);
            break;
        default:
            break;
    }      
    ptrd[2+ptrd[2]] = get_u8data_checksum(&ptrd[2],ptrd[2]);    
}

//接收数据解析
void wifi_data_packet_decode(uint8_t *pbuffer)
{    
    uint8_t *ptrd;
    ptrd = pbuffer;
    
    //最大命令值为7
    if(ptrd[3] > CMD8)
        return;
//    //判断包头是否正确
//    if ((ptrd[0] != ((WIFI_PACKET_HEAD >> 8) & 0x00ff)) || (ptrd[1] != (WIFI_PACKET_HEAD & 0x00ff)))
//        return;
    //判断校验是否正确
    if (ptrd[ptrd[2] + 2] != get_u8data_checksum(&ptrd[2],ptrd[2]))
        return;
    //判断功能命令
    switch(ptrd[3])
    {
        case    CMD0:
            memcpy(&wifi_cmd0_packet,&ptrd[4],WIFI_CMD0_PACKET_LEN);
            break;
        case    CMD1:
            memcpy(&wifi_cmd1_packet,&ptrd[4],WIFI_CMD1_PACKET_LEN);
            if (get_phone_control_start_landing() != get_phone_control_start_landing_backup())
            {
                if (!get_plane_flying_status())
                {
                    set_plane_unlock_status();
                    set_plane_flying_status();
                }
                else
                {
                    set_plane_lock_status();
                    set_plane_stop_fly_status();
                }
            }            
            memcpy(&wifi_cmd1_packet_backup,&wifi_cmd1_packet,sizeof(wifi_cmd1_packet));
            break;
        case    CMD2:
            memcpy(&wifi_cmd2_packet,&ptrd[4],WIFI_CMD2_PACKET_LEN);            
            break;
        case    CMD3:
            memcpy(&wifi_cmd3_packet,&ptrd[4],WIFI_CMD3_PACKET_LEN);
            store_fly_limit_setting_data();
            f_cmd3_send_flag = true;
            break;
        case    CMD4:
            memcpy(&wifi_cmd4_packet,&ptrd[4],WIFI_CMD4_PACKET_LEN);
            break;
        case    CMD5:
            memcpy(&wifi_cmd5_packet,&ptrd[4],WIFI_CMD5_PACKET_LEN);
            break;
        case    CMD6:
            memcpy(&wifi_cmd6_packet,&ptrd[4],WIFI_CMD6_PACKET_LEN);
            set_plane_point_mode();
            f_cmd2_send_flag = true;
            break;
        case    CMD7:
            memcpy(&wifi_cmd7_packet,&ptrd[4],WIFI_CMD7_PACKET_LEN);
            break;
        case    CMD8:
            memcpy(&wifi_cmd8_packet,&ptrd[4],WIFI_CMD8_PACKET_LEN);
            break;
        default:
            break;        
    }    
}

//保存飞行限制设置数据
void store_fly_limit_setting_data(void)
{
    PARAMETER_SAVE *poParam;    
    
    poParam = LoadParameter();
    poParam->abyFlyLimitSettingData[0] = wifi_cmd3_packet.altitude_limit_setting;
    poParam->abyFlyLimitSettingData[1] = wifi_cmd3_packet.distance_limit_setting;
    poParam->abyFlyLimitSettingData[2] = wifi_cmd3_packet.return_altitude_setting;
    poParam->abyFlyLimitSettingData[3] = wifi_cmd3_packet.green_hands_setting;
    poParam->abyFlyLimitSettingData[3] = wifi_cmd3_packet.stick_mode;
    SaveParameter();
}

//读取飞行限制设置数据
bool read_fly_limit_setting_data(void)
{
    uint8_t i;    
    PARAMETER_SAVE *poParam;
    
    poParam = LoadParameter();
    
    for (i = 0;i<5;i++)
    {
        if(poParam->abyFlyLimitSettingData[i] != 0xff)
        {            
            break;
        }
    }
    if (i >= 5)
    {
        return false;
    } 
    
    wifi_cmd3_packet.altitude_limit_setting = poParam->abyFlyLimitSettingData[0];
    wifi_cmd3_packet.distance_limit_setting = poParam->abyFlyLimitSettingData[1];
    wifi_cmd3_packet.return_altitude_setting = poParam->abyFlyLimitSettingData[2];
    wifi_cmd3_packet.green_hands_setting = poParam->abyFlyLimitSettingData[3];
    wifi_cmd3_packet.stick_mode = poParam->abyFlyLimitSettingData[4];
    return true;
}





