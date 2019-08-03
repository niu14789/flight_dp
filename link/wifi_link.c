#include <stdio.h>
#include "gd32f30x.h"
//#include "gd32f307c_eval.h"
#include "wifi_link.h"
#include "string.h"
//#include "rf_link.h"

plane_cmd0_packet_def plane_cmd0_packet;
plane_cmd1_packet_def plane_cmd1_packet;
plane_cmd2_packet_def plane_cmd2_packet;
plane_cmd3_packet_def plane_cmd3_packet;
plane_cmd4_packet_def plane_cmd4_packet;
plane_cmd5_packet_def plane_cmd5_packet;
plane_cmd6_packet_def plane_cmd6_packet;

wifi_cmd0_packet_def wifi_cmd0_packet;
wifi_cmd1_packet_def wifi_cmd1_packet;
wifi_cmd2_packet_def wifi_cmd2_packet;
wifi_cmd3_packet_def wifi_cmd3_packet;
wifi_cmd4_packet_def wifi_cmd4_packet;
wifi_cmd5_packet_def wifi_cmd5_packet;
wifi_cmd6_packet_def wifi_cmd6_packet;
wifi_cmd7_packet_def wifi_cmd7_packet;

// 获取8bit校验和
uint8_t get_u8data_checksum(uint8_t *pbuf,uint8_t len)
{
    uint8_t u8_i,checksum;
    checksum = 0;
    for (u8_i = 0;u8_i < len;u8_i ++)
    {
        checksum ^= pbuf[u8_i];        
    }
    return checksum;
}

//plane_cmd2_packet.plane_status1操作
__INLINE void set_plane_lock_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT0; }
__INLINE void set_plane_unlock_status(void) { plane_cmd2_packet.plane_status1 |= BIT0; }
__INLINE void set_plane_stop_fly_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT1; }
__INLINE void set_plane_flying_status(void) { plane_cmd2_packet.plane_status1 |= BIT1; }
__INLINE void set_plane_no_gps_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT2; }
__INLINE void set_plane_got_gps_status(void) { plane_cmd2_packet.plane_status1 |= BIT2;}
__INLINE void clear_plane_follow_mode(void) { plane_cmd2_packet.plane_status1 &= ~BIT3; }
__INLINE void set_plane_follow_mode(void) { plane_cmd2_packet.plane_status1 |= BIT3; }
__INLINE void clear_plane_circle_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT4; }
__INLINE void set_plane_circle_mode(void) {plane_cmd2_packet.plane_status1 |= BIT4; }
__INLINE void clear_plane_point_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT5; }
__INLINE void set_plane_point_mode(void) {plane_cmd2_packet.plane_status1 |= BIT5; }
__INLINE void clear_plane_return_noome_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT6; }
__INLINE void set_plane_return_home_mode(void) {plane_cmd2_packet.plane_status1 |= BIT6; }
__INLINE void clear_plane_landing_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT7; }
__INLINE void set_plane_landing_mode(void) {plane_cmd2_packet.plane_status1 |= BIT7; }
//plane_cmd2_packet.plane_status2操作
__INLINE void clear_plane_gyro_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT0; }
__INLINE void set_plane_gyro_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT0; }
__INLINE void clear_plane_compass_horizontal_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT1; }
__INLINE void set_plane_compass_horizontal_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT1; }
__INLINE void clear_plane_compass_vertical_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT2; }
__INLINE void set_plane_compass_vertical_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT2; }
__INLINE void set_plane_unlink_remoter_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT3; }
__INLINE void set_plane_link_remoter_status(void) { plane_cmd2_packet.plane_status2 |= BIT3; }
__INLINE void clear_plane_nohead_mode(void) { plane_cmd2_packet.plane_status2 &= ~BIT6; }
__INLINE void set_plane_nohead_mode(void) { plane_cmd2_packet.plane_status2 |= BIT6; }
__INLINE void clear_plane_scram_mode(void) { plane_cmd2_packet.plane_status2 &= ~BIT7; }
__INLINE void set_plane_scram_mode(void) { plane_cmd2_packet.plane_status2 |= BIT7; }
//plane_cmd2_packet.plane_status3操作
__INLINE void set_plane_low_speed_status(void) { plane_cmd2_packet.plane_status3 &= ~BIT0; }
__INLINE void set_plane_high_speed_status(void) { plane_cmd2_packet.plane_status3 |= BIT0; }
__INLINE void clear_plane_low_voltage_status(void) { plane_cmd2_packet.plane_status3 &= ~BIT1; }
__INLINE void set_plane_low_voltage_status(void) { plane_cmd2_packet.plane_status3 |= BIT1; }
__INLINE void clear_plane_green_hands_mode(void) { plane_cmd2_packet.plane_status3 &= ~BIT2; }
__INLINE void set_plane_green_hands_mode(void) { plane_cmd2_packet.plane_status3 |= BIT2; }
__INLINE void clear_plane_front_left_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT3; }
__INLINE void set_plane_front_left_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT3; }
__INLINE void clear_plane_front_right_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT4; }
__INLINE void set_plane_front_right_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT4; }
__INLINE void clear_plane_back_right_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT5; }
__INLINE void set_plane_back_right_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT5; }
__INLINE void clear_plane_back_left_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT6; }
__INLINE void set_plane_back_left_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT6; }

//plane_cmd2_packet.plane_status4操作
__INLINE void clear_plane_mpu_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT0; }
__INLINE void set_plane_mpu_erro(void) { plane_cmd2_packet.plane_status4 |= BIT0; }
__INLINE void clear_plane_baro_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT1; }
__INLINE void set_plane_baro_erro(void) { plane_cmd2_packet.plane_status4 |= BIT1; }
__INLINE void clear_plane_gps_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT2; }
__INLINE void set_plane_gps_erro(void) { plane_cmd2_packet.plane_status4 |= BIT2; }
__INLINE void clear_plane_compass_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT3; }
__INLINE void set_plane_compass_erro(void) { plane_cmd2_packet.plane_status4 |= BIT3; }
__INLINE void clear_plane_wind_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT4; }
__INLINE void set_plane_wind_erro(void) { plane_cmd2_packet.plane_status4 |= BIT4; }
__INLINE void clear_plane_gps_rssi_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT5; }
__INLINE void set_plane_gps_rssi_erro(void) { plane_cmd2_packet.plane_status4 |= BIT5; }

//wifi_cmd1_packet.control_info 操作
__INLINE uint8_t get_phone_control_speed_mode(void) { return (wifi_cmd1_packet.control_info & 0x03); }
__INLINE uint8_t get_phone_control_roll_mode(void) { return (wifi_cmd1_packet.control_info & BIT2); }
__INLINE uint8_t get_phone_control_return_home_mode(void) { return (wifi_cmd1_packet.control_info & BIT3); }
__INLINE uint8_t get_phone_control_nohead_mode(void) { return (wifi_cmd1_packet.control_info & BIT4); }
__INLINE uint8_t get_phone_control_scram_mode(void) { return (wifi_cmd1_packet.control_info & BIT5); }
__INLINE uint8_t get_phone_control_start(void) { return (wifi_cmd1_packet.control_info & BIT6); }
__INLINE uint8_t get_phone_control_landing(void) { return (wifi_cmd1_packet.control_info & BIT7); }
//wifi_cmd1_packet.aileron_fine_tuning 操作
__INLINE uint8_t get_phone_control_altitude_info(void) { return (wifi_cmd1_packet.aileron_fine_tuning & BIT7); }
__INLINE uint8_t get_phone_control_one_key_calibrate_flag(void) { return (wifi_cmd1_packet.aileron_fine_tuning & BIT6); }
__INLINE uint8_t get_phone_control_camera_up_left(void) { return (wifi_cmd1_packet.elevator_fine_tuning & BIT7); }
__INLINE uint8_t get_phone_control_camera_down_right(void) { return (wifi_cmd1_packet.elevator_fine_tuning & BIT6); }
//wifi_cmd1_packet.rudder_fine_tuning 操作
__INLINE uint8_t get_phone_control_led(void) { return (wifi_cmd1_packet.rudder_fine_tuning & (BIT7|BIT6)); }
//wifi_cmd5_packet.control_info 操作
__INLINE uint8_t get_phone_control_compass_calibrate(void) {return (wifi_cmd5_packet.control_info & BIT0); }
__INLINE uint8_t get_phone_control_level_calibrate(void) {return (wifi_cmd5_packet.control_info & BIT1); }
__INLINE uint8_t get_phone_control_lock_info(void) {return (wifi_cmd5_packet.control_info & BIT2); }

//飞机发送数据打包
void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd)
{   
    uint8_t *ptrd;
    ptrd = pbuffer;
    
    //最大命令值为6
    if(cmd > 6)
        return;
    
    ptrd[0] = (PLANE_PACKET_HEAD >> 8) & 0x00ff;
    ptrd[1] = PLANE_PACKET_HEAD & 0x00ff;    
    
    switch(cmd)
    {
        case    CMD0:
            ptrd[2] = CMD0_PACKET_LEN + 2;   
            ptrd[3] = CMD0;            
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
            memcpy(&ptrd[4],&plane_cmd6_packet,CMD6_PACKET_LEN);
            break;
        default:
            break;
    }      
    ptrd[4+CMD0_PACKET_LEN] = get_u8data_checksum(&ptrd[2],ptrd[2]+1);    
}

//接收数据解析
void wifi_data_packet_decode(uint8_t *pbuffer)
{    
    uint8_t *ptrd;
    ptrd = pbuffer;
    
    //最大命令值为7
    if(ptrd[3] > 7)
        return;
    //判断包头是否正确
    if ((ptrd[0] != ((WIFI_PACKET_HEAD >> 8) & 0x00ff)) || (ptrd[1] != (WIFI_PACKET_HEAD & 0x00ff)))
        return;
    //判断校验是否正确
    if (ptrd[ptrd[2] + 1] != get_u8data_checksum(&ptrd[2],ptrd[2]+1))
        return;
    //判断功能命令
    switch(ptrd[3])
    {
        case    CMD0:
            memcpy(&wifi_cmd0_packet,&ptrd[4],WIFI_CMD0_PACKET_LEN);
            break;
        case    CMD1:
            memcpy(&wifi_cmd1_packet,&ptrd[4],WIFI_CMD1_PACKET_LEN);
            break;
        case    CMD2:
            memcpy(&wifi_cmd2_packet,&ptrd[4],WIFI_CMD2_PACKET_LEN);
            break;
        case    CMD3:
            memcpy(&wifi_cmd3_packet,&ptrd[4],WIFI_CMD3_PACKET_LEN);
            break;
        case    CMD4:
            memcpy(&wifi_cmd4_packet,&ptrd[4],WIFI_CMD4_PACKET_LEN);
            break;
        case    CMD5:
            memcpy(&wifi_cmd5_packet,&ptrd[4],WIFI_CMD5_PACKET_LEN);
            break;
        case    CMD6:
            memcpy(&wifi_cmd6_packet,&ptrd[4],WIFI_CMD6_PACKET_LEN);
            break;
        case    CMD7:
            memcpy(&wifi_cmd7_packet,&ptrd[4],WIFI_CMD7_PACKET_LEN);
            break;
        default:
            break;        
    }    
}




