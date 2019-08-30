/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : notify.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	* BEEP TIM3 CHANNEL1 PWM Gerente
	* common is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gd32f30x.h"
#include "fs.h"
#include "f_shell.h"
#include "fs_config.h"
#include "string.h"
#include "wifi.h"
#include "f_ops.h"
#include "f_drv.h"
#include "state.h"
#include "wifi_link.h"
#include "common.h"

uint32_t g_dw_plane_status;      //飞机状态

plane_cmd0_packet_stru plane_cmd0_packet;
plane_cmd1_packet_stru plane_cmd1_packet;
plane_cmd2_packet_stru plane_cmd2_packet;
plane_cmd3_packet_stru plane_cmd3_packet;
plane_cmd4_packet_stru plane_cmd4_packet;
plane_cmd5_packet_stru plane_cmd5_packet;
plane_cmd6_packet_stru plane_cmd6_packet;
plane_cmd9_packet_stru plane_cmd9_packet;

wifi_cmd0_packet_stru wifi_cmd0_packet;
wifi_cmd1_packet_stru wifi_cmd1_packet;
wifi_cmd1_packet_backup_stru wifi_cmd1_packet_backup;
wifi_cmd2_packet_stru wifi_cmd2_packet;
wifi_cmd3_packet_stru wifi_cmd3_packet;
wifi_cmd4_packet_stru wifi_cmd4_packet;
wifi_cmd5_packet_stru wifi_cmd5_packet;
wifi_cmd5_packet_stru wifi_cmd5_packet_backup;
wifi_cmd6_packet_stru wifi_cmd6_packet;
wifi_cmd7_packet_stru wifi_cmd7_packet;
wifi_cmd8_packet_stru wifi_cmd8_packet;
wifi_cmd9_packet_stru wifi_cmd9_packet;

plane_function_control_info_stru m_plane_function_control_info;

void store_iap_start_flag(void);
void set_plane_wifi_lose_control_flag(void);
void plane_function_status_update_to_wifi(void);

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

void wifi_link_init(void)
{       
    if(false == read_fly_limit_setting_data())
    {
        wifi_cmd3_packet.altitude_limit_setting = 30;
        wifi_cmd3_packet.distance_limit_setting = 100;
        wifi_cmd3_packet.return_altitude_setting = 30;
        wifi_cmd3_packet.green_hands_setting= NORMAL_MODE;
        wifi_cmd3_packet.stick_mode = AMERICA_HAND;
        wifi_cmd3_packet.circle_radius = 10;
        store_fly_limit_setting_data();
    }
    set_plane_wifi_lose_control_flag();
    memset(&m_plane_function_control_info,0,sizeof(m_plane_function_control_info));
    m_plane_function_control_info.w_lose_control_timer = 5000;    
}

/* some define */
static unsigned char wifi_receive_buffer[512];
static unsigned char wifi_parse_buffer[256];
/*-------------*/
void wifi_link_data_receive(void)
{
	/* some defines */
	static unsigned char steps = 0;
	static unsigned char read_data_len = 0;
	static unsigned char payload_cnt = 0;    
    
    //判断失控
    if (m_plane_function_control_info.w_lose_control_timer < 5000/4)
    {
        m_plane_function_control_info.w_lose_control_timer++;
    }
    if (m_plane_function_control_info.w_lose_control_timer >= 2000/4)
    {
        set_plane_wifi_lose_control_flag();
    }    
    
	/* read data from usart3 */
	int len = wifi_receive_bytes(wifi_receive_buffer,sizeof(wifi_receive_buffer));
	/* got some data */
	if( len > 0 )
	{
		for( int i = 0 ; i < len ; i ++ )
		{
			/* parse */
			switch(steps)
			{
				case 0:
					/* head h */
				  if( wifi_receive_buffer[i] == 0xFF /*WIFI_PACKET_HEAD_H */ )
					{
						/* copy data */
						wifi_parse_buffer[0] = wifi_receive_buffer[i];
						/* switch */
						steps = 1;
					}
					break;
				case 1:
					/* head h */
				  if( wifi_receive_buffer[i] == 0xFD /*WIFI_PACKET_HEAD_L */ )
					{
						/* copy data */
						wifi_parse_buffer[1] = wifi_receive_buffer[i];
						/* switch */						
						steps = 2;
					}
                    else
					{
						steps = 0;
					}
					break;	
				case 2:
					/* len = payload + checksum */
				  read_data_len =  wifi_receive_buffer[i];
					/* copy data */
					wifi_parse_buffer[2] = wifi_receive_buffer[i];
				  /* payload cnt clear */
				  payload_cnt = 0;
					/* switch */
					steps = 3;
				  /* end of this cmd */
					break;					
				case 3:
					/* payload and checksum */
				  if( payload_cnt <= read_data_len )
					{
						/* copy data */
						wifi_parse_buffer[3+payload_cnt] = wifi_receive_buffer[i];
						/* over */
						payload_cnt ++;
						/* parse data */
						if( payload_cnt >= read_data_len )
						{
#if 1
                            wifi_data_packet_decode(wifi_parse_buffer);     
                            plane_function_status_update_to_wifi();
#endif							
							steps = 0;
						}
					}
					else
					{
						steps = 0;
					}
				  /* end of this cmd */
					break;	
				default:
  				steps = 0;
					break;
			}
			/* end of this */
		}
	}
}

//static bool f_cmd2_send_flag = false;
static bool f_cmd3_send_flag = false;
static bool f_cmd6_send_flag = false;
//static bool f_cmd9_send_flag = false;
/* void wifi_link_data_send(void) */
void wifi_link_data_send(void)
{
	static uint8_t s_abySend[256];
    uint8_t bySendFlag = 0;  
    uint8_t bySendDataLength = 0; 
    static uint32_t wSendTimer = 0; 
    
    if ( SYNCHRONOUS_CMD ==  wifi_cmd8_packet.synchronous_request)
    {
        wifi_cmd8_packet.synchronous_request = 0;
        f_cmd3_send_flag = true;
        f_cmd6_send_flag = true;
    }

    wSendTimer ++;
    
    //有数据为发完退出
//    if (SEND_FINISHED != UsartGetSendStatus(g_poHandle))
//        return;     
    
    // cmd0 数据500ms 发送一次
    if (0 == (wSendTimer % (500/4)))
    {   
        plane_send_data_packet(s_abySend,CMD0); 
        bySendDataLength = CMD0_PACKET_LEN+5;
        bySendFlag = 1;
    }
      // cmd1 数据20ms 发送一次
//    else if (20 == (wSendTimer % (500/4)))
//    {   
//        plane_send_data_packet(s_abySend,CMD1);
//        bySendDataLength = CMD1_PACKET_LEN+5;        
//        bySendFlag = 1;
//    } 
    // cmd2 数据500ms 发送一次
    else if (40 == (wSendTimer % (500/4)))// && (true == f_cmd2_send_flag))
    {   
//        f_cmd2_send_flag = false;
        plane_send_data_packet(s_abySend,CMD2);   
        bySendDataLength = CMD2_PACKET_LEN+5;
        bySendFlag = 1;
    } 
    // cmd3 数据
    else if ((60 == (wSendTimer % (500/4))) && (true == f_cmd3_send_flag))
    {   
        f_cmd3_send_flag = false;
        plane_send_data_packet(s_abySend,CMD3);
        bySendDataLength = CMD3_PACKET_LEN+5;        
        bySendFlag = 1;
    }       
    // cmd6 数据
    else if ((120 == (wSendTimer % (500/4))) && (true == f_cmd6_send_flag))
    {   
        f_cmd6_send_flag = false;
        plane_send_data_packet(s_abySend,CMD6);
        bySendDataLength = CMD6_PACKET_LEN+5;        
        bySendFlag = 1;
    } 
    // cmd9 数据
//    else if ((200 == (wSendTimer % (500/4))) && (true == f_cmd9_send_flag))
//    {   
//        f_cmd9_send_flag = false;
//        plane_send_data_packet(s_abySend,CMD9);
//        bySendDataLength = CMD9_PACKET_LEN+5; 
//        if (plane_cmd9_packet.to_device == 0x10)
//        {
//            //如果升级飞控，发送升级响应完成后软件复位进入BootLoader程序
//            UsartSendNoBlocking(g_poHandle, s_abySend, bySendDataLength);
//            delay_ms(10);
//            store_iap_start_flag();
//            __set_FAULTMASK(0);
//            NVIC_SystemReset();
//        }        
//        else
//        {
//            //其他模块升级，发送响应，还需进入升级传输模式，后续增加
//            /*
//                飞控进入升级传输功能处理，待增加
//            */
//            bySendFlag = 1;
//        } 
//    }
    if (bySendFlag)
    {
        bySendFlag = 0;
        wifi_send_bytes(s_abySend, bySendDataLength);
    }   
}


//plane_cmd2_packet.plane_status1操作
void set_plane_cmd2_lock_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT0; }
void clear_plane_cmd2_lock_status(void) { plane_cmd2_packet.plane_status1 |= BIT0; }
void set_plane_cmd2_flying_status(void) { plane_cmd2_packet.plane_status1 |= BIT1; }
void clear_plane_cmd2_flying_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT1; }
void set_plane_cmd2_got_gps_status(void) { plane_cmd2_packet.plane_status1 |= BIT2; }
void clear_plane_cmd2_got_gps_status(void) { plane_cmd2_packet.plane_status1 &= ~BIT2;}
void set_plane_cmd2_follow_mode(void) { plane_cmd2_packet.plane_status1 |= BIT3; }
void clear_plane_cmd2_follow_mode(void) { plane_cmd2_packet.plane_status1 &= ~BIT3; }

void clear_plane_cmd2_circle_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT4; }
void set_plane_cmd2_circle_mode(void) {plane_cmd2_packet.plane_status1 |= BIT4; }
void clear_plane_cmd2_point_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT5; }
void set_plane_cmd2_point_mode(void) {plane_cmd2_packet.plane_status1 |= BIT5; }
void clear_plane_cmd2_return_noome_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT6; }
void set_plane_cmd2_return_home_mode(void) {plane_cmd2_packet.plane_status1 |= BIT6; }
void clear_plane_cmd2_landing_mode(void) {plane_cmd2_packet.plane_status1 &= ~BIT7; }
void set_plane_cmd2_landing_mode(void) {plane_cmd2_packet.plane_status1 |= BIT7; }
//plane_cmd2_packet.plane_status2操作
void clear_plane_cmd2_gyro_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT0; }
void set_plane_cmd2_gyro_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT0; }
void clear_plane_cmd2_compass_horizontal_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT1; }
void set_plane_cmd2_compass_horizontal_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT1; }
void clear_plane_cmd2_compass_vertical_calibrate_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT2; }
void set_plane_cmd2_compass_vertical_calibrate_status(void) { plane_cmd2_packet.plane_status2 |= BIT2; }
void clear_plane_cmd2_link_remoter_status(void) { plane_cmd2_packet.plane_status2 &= ~BIT3; }
void set_plane_cmd2_link_remoter_status(void) { plane_cmd2_packet.plane_status2 |= BIT3; }
void clear_plane_cmd2_rising_mode(void) {plane_cmd2_packet.plane_status2 &= ~BIT4; }
void set_plane_cmd2_rising_mode(void) { plane_cmd2_packet.plane_status2 |= BIT4; }
void clear_plane_cmd2_headless_mode(void) { plane_cmd2_packet.plane_status2 &= ~BIT6; }
void set_plane_cmd2_headless_mode(void) { plane_cmd2_packet.plane_status2 |= BIT6; }
void clear_plane_cmd2_scram_mode(void) { plane_cmd2_packet.plane_status2 &= ~BIT7; }
void set_plane_cmd2_scram_mode(void) { plane_cmd2_packet.plane_status2 |= BIT7; }
//plane_cmd2_packet.plane_status3操作
void set_plane_cmd2_low_speed_status(void) { plane_cmd2_packet.plane_status3 &= ~BIT0; }
void set_plane_cmd2_high_speed_status(void) { plane_cmd2_packet.plane_status3 |= BIT0; }
void clear_plane_cmd2_low_voltage_status(void) { plane_cmd2_packet.plane_status3 &= ~BIT1; }
void set_plane_cmd2_low_voltage_status(void) { plane_cmd2_packet.plane_status3 |= BIT1; }
void clear_plane_cmd2_green_hands_mode(void) { plane_cmd2_packet.plane_status3 &= ~BIT2; }
void set_plane_cmd2_green_hands_mode(void) { plane_cmd2_packet.plane_status3 |= BIT2; }
void clear_plane_cmd2_front_left_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT3; }
void set_plane_cmd2_front_left_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT3; }
void clear_plane_cmd2_front_right_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT4; }
void set_plane_cmd2_front_right_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT4; }
void clear_plane_cmd2_back_right_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT5; }
void set_plane_cmd2_back_right_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT5; }
void clear_plane_cmd2_back_left_motor_overload(void) { plane_cmd2_packet.plane_status3 &= ~BIT6; }
void set_plane_cmd2_back_left_motor_overload(void) { plane_cmd2_packet.plane_status3 |= BIT6; }

//plane_cmd2_packet.plane_status4操作
void clear_plane_cmd2_mpu_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT0; }
void set_plane_cmd2_mpu_erro(void) { plane_cmd2_packet.plane_status4 |= BIT0; }
void clear_plane_cmd2_baro_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT1; }
void set_plane_cmd2_baro_erro(void) { plane_cmd2_packet.plane_status4 |= BIT1; }
void clear_plane_cmd2_gps_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT2; }
void set_plane_cmd2_gps_erro(void) { plane_cmd2_packet.plane_status4 |= BIT2; }
void clear_plane_cmd2_compass_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT3; }
void set_plane_cmd2_compass_erro(void) { plane_cmd2_packet.plane_status4 |= BIT3; }
void clear_plane_cmd2_wind_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT4; }
void set_plane_cmd2_wind_erro(void) { plane_cmd2_packet.plane_status4 |= BIT4; }
void clear_plane_cmd2_gps_rssi_erro(void) { plane_cmd2_packet.plane_status4 &= ~BIT5; }
void set_plane_cmd2_gps_rssi_erro(void) { plane_cmd2_packet.plane_status4 |= BIT5; }

void update_plane_cmd2_photo_function_to_wifi(void) { plane_cmd2_packet.plane_status5 ^= BIT0; }
void update_plane_cmd2_video_function_to_wifi(void) { plane_cmd2_packet.plane_status5 ^= BIT1; }

void set_plane_cmd2_photo_ev_inc(void) { plane_cmd2_packet.plane_status5 |= BIT4; }
void clear_plane_cmd2_photo_ev_inc(void) { plane_cmd2_packet.plane_status5 &= ~BIT4; }
void set_plane_cmd2_photo_ev_dec(void) { plane_cmd2_packet.plane_status5 |= BIT5; }
void clear_plane_cmd2_photo_ev_dec(void) { plane_cmd2_packet.plane_status5 &= ~BIT5; }



void set_plane_cmd2_remote_choice_24G(void)	{ plane_cmd2_packet.plane_status5 |= BIT7; }
void set_plane_cmd2_remote_choice_repeater(void) { plane_cmd2_packet.plane_status5 &= ~BIT7; }


//wifi_cmd0_packet
uint16_t get_phone_control_follow_mode(void) { return wifi_cmd0_packet.follow_me_flag;}
//wifi_cmd1_packet.control_info 操作
uint8_t get_phone_control_speed_mode(void) { return (m_plane_function_control_info.by_control_info & 0x03); }
uint8_t get_phone_control_roll_mode(void) { return (m_plane_function_control_info.by_control_info & BIT2); }
uint8_t get_phone_control_return_home_mode(void) { return (m_plane_function_control_info.by_control_info & BIT3); }
uint8_t get_phone_control_headless_mode(void) { return (m_plane_function_control_info.by_control_info & BIT4); }
uint8_t get_phone_control_scram_mode(void) { return (m_plane_function_control_info.by_control_info & BIT5); }
uint8_t get_phone_control_rising_landing(void) { return (m_plane_function_control_info.by_control_info & BIT6); }
void clear_phone_control_rising_landing(void) { m_plane_function_control_info.by_control_info &= ~BIT6; }

//uint8_t get_phone_control_landing(void) { return (m_plane_function_control_info.by_control_info & BIT7); }
//wifi_cmd1_packet.led_control 操作
uint8_t get_phone_control_led(void) { return (wifi_cmd1_packet.led_control & (BIT7|BIT6)); }
//wifi_cmd1_packet.calibrate_info 操作
uint8_t get_phone_control_compass_calibrate(void) { return (m_plane_function_control_info.by_calibrate_info & BIT0); } //地磁校准，按下为1，再按为0 
void clear_phone_control_compass_calibrate(void) { m_plane_function_control_info.by_calibrate_info &= ~BIT0; }
uint8_t get_phone_control_level_calibrate(void) { return (m_plane_function_control_info.by_calibrate_info & BIT1); } //水平校准，按下为1，再按为0 
uint8_t get_phone_control_lock_info(void) { return (m_plane_function_control_info.by_calibrate_info & BIT2); } //解锁，按下为1，再按为0 
uint8_t get_phone_control_circle(void) { return (m_plane_function_control_info.by_calibrate_info & BIT3); } //热点环绕，按下为1，再按为0 
void clear_phone_control_circle(void) { m_plane_function_control_info.by_calibrate_info &= ~BIT3; }
uint8_t get_phone_control_point_fly(void) { return (m_plane_function_control_info.by_calibrate_info & BIT4); } //指点飞行，按下为1，再按为0
void clear_phone_control_point_fly(void) { m_plane_function_control_info.by_calibrate_info &= ~BIT4; }
uint8_t get_phone_control_one_key_calibrate_flag(void) { return (m_plane_function_control_info.by_calibrate_info & BIT5); }
uint8_t get_phone_control_altitude_info(void) { return (m_plane_function_control_info.by_calibrate_info & BIT6); }
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

/* define a template struct to get data */
extern ICM206_INS_DEF  icm206_d;
extern GPS_User_t      gps_user;
extern ST480_MAG_DEF   st480_user; /* user mag data */
extern BARO_METER_DATA baro_user;  /* user baro data*/ 
extern power_user_s    bat_user;   /* user battery voltage */
extern rcs_user_s      sbus_user;  /* user sbus data */

//飞机发送数据打包
//extern SENSOR_RAW_DATA oSensorData;
void plane_send_data_packet(uint8_t *pbuffer,uint8_t cmd)
{  
    uint8_t *ptrd;    

    ptrd = pbuffer;
    
    //最大命令值为6
    if(cmd > 9)
        return;
    
    ptrd[0] = PLANE_PACKET_HEAD_H;
    ptrd[1] = PLANE_PACKET_HEAD_L;

    switch(cmd)
    {
        case    CMD0:
            ptrd[2] = CMD0_PACKET_LEN + 2;   
            ptrd[3] = CMD0;        
            plane_cmd0_packet.plane_voltage = (uint16_t)(bat_user.bat_voltage * 100);
            plane_cmd0_packet.remoter_voltage = 375;//(uint16_t)(remote_packet.battery_voltage * 10);    //remote_packet.battery_voltage：为实际电压*10 
            plane_cmd0_packet.gps_longitude = gps_user.lon;
            plane_cmd0_packet.gps_latitude =  gps_user.lat;
            plane_cmd0_packet.gps_NumSv = gps_user.numSV;
            plane_cmd0_packet.heading = 0;
            plane_cmd0_packet.distance = 4520;
            plane_cmd0_packet.altitude = 540;
            plane_cmd0_packet.horizontal_speed = gps_user.velE;
            plane_cmd0_packet.vertical_speed = gps_user.velU;
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
            //飞机状态更新
        
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
//            plane_cmd6_packet.remoter_version_h = poParam->abyRemoterVersion[0];
//            plane_cmd6_packet.remoter_version_m = poParam->abyRemoterVersion[1];
//            plane_cmd6_packet.remoter_version_l = poParam->abyRemoterVersion[2];
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
        case    CMD9:
            ptrd[2] = CMD9_PACKET_LEN + 2;   
            ptrd[3] = CMD9;   
            memcpy(&plane_cmd9_packet,&wifi_cmd9_packet,CMD9_PACKET_LEN);
            memcpy(&ptrd[4],&plane_cmd9_packet,CMD9_PACKET_LEN);
            break;
        default:
        
            break;
    }      
    ptrd[2+ptrd[2]] = get_u8data_checksum(&ptrd[2],ptrd[2]);    
}

#define PLANE_WIFI_LOSE_CONTROL_STATUS 1

uint8_t get_plane_wifi_link_status(void)
{
    return m_plane_function_control_info.by_lose_control;
}
void set_plane_wifi_lose_control_flag(void)
{
    m_plane_function_control_info.by_lose_control = 1;
}
void clear_plane_wifi_lose_control_flag(void)
{
    m_plane_function_control_info.by_lose_control = 0;
    m_plane_function_control_info.w_lose_control_timer = 0;
}

//接收数据解析
void wifi_data_packet_decode(uint8_t *pbuffer)
{    
    uint8_t *ptrd;
    ptrd = pbuffer;
    
    //最大命令值为9
    if(ptrd[3] > CMD9)
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
        
//            if (wifi_cmd1_packet.channel_number != 0x04)
//            {
//                break;
//            }
                
            //若飞机不是出于失控状态，则处理控制信号，否则只是更新备份信息 
            if (PLANE_WIFI_LOSE_CONTROL_STATUS != get_plane_wifi_link_status())
            {   
                m_plane_function_control_info.by_calibrate_info ^= wifi_cmd1_packet.calibrate_info ^ wifi_cmd1_packet_backup.calibrate_info;            
                m_plane_function_control_info.by_control_info ^= wifi_cmd1_packet.control_info ^ wifi_cmd1_packet_backup.control_info;
//                m_plane_function_control_info.by_camero_info |= wifi_cmd1_packet.camera_info ^ wifi_cmd1_packet_backup.camera_info;
                m_plane_function_control_info.by_camero_info = wifi_cmd1_packet_backup.camera_info;
            }
            clear_plane_wifi_lose_control_flag();
            //更新飞机控制备份信息            
            wifi_cmd1_packet_backup.calibrate_info = wifi_cmd1_packet.calibrate_info;
            wifi_cmd1_packet_backup.control_info = wifi_cmd1_packet.control_info; 
            wifi_cmd1_packet_backup.camera_info = wifi_cmd1_packet.camera_info; 
            
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
            break;
        case    CMD7:
            memcpy(&wifi_cmd7_packet,&ptrd[4],WIFI_CMD7_PACKET_LEN);
            break;
        case    CMD8:
            memcpy(&wifi_cmd8_packet,&ptrd[4],WIFI_CMD8_PACKET_LEN);
            break;
        case    CMD9:
            memcpy(&wifi_cmd9_packet,&ptrd[4],WIFI_CMD9_PACKET_LEN);
            //目标设备为飞控或者云台则相应
//            if ((wifi_cmd9_packet.to_device == 0x10) || (wifi_cmd9_packet.to_device == 0x14))
//            {
//                f_cmd9_send_flag = true;
//            }
            if (wifi_cmd9_packet.to_device == 0x10)
            {
                //如果升级飞控，发送升级响应完成后软件复位进入BootLoader程序  
//                store_iap_start_flag();
							erase_iap_id();
//                delay_ms(10);                
                __set_FAULTMASK(0);
                NVIC_SystemReset();
            }        
            else
            {
                //其他模块升级，发送响应，还需进入升级传输模式，后续增加
                /*
                    飞控进入升级传输功能处理，待增加
                */               
            }              
            break;
        default:
            break;        
    }    
}

#pragma pack (1) /*指定按2字节对齐*/
typedef struct FLASH_STRU
{   
    uint8_t     updata_flag[4];
    uint8_t     FlyLimitSettingData[8];
    uint16_t    store_flag;
    uint8_t     check_sum;      //位置不能动，必须放在结构最后面
}FLASH_DEF;
#pragma pack () /*取消指定对齐，恢复缺省对齐*/
FLASH_DEF flash;
#define FLASH_DATA_BUFFER_LEN  sizeof(flash)

#define IAP_START_FLAG      0X11223344
//保存飞行限制设置数据
int erase_iap_id(void)
{
	/* erase flash addr */
  user_flash_ioctrl(4,0x08007800,0);
	/* judge */
	if( (*((unsigned int *)0x08007800)) == 0xffffffff )
	{
		return FS_OK;
	}
	/* error */
	return FS_ERR;
}

//保存飞行限制设置数据
void store_fly_limit_setting_data(void)
{
    flash.FlyLimitSettingData[0] = wifi_cmd3_packet.altitude_limit_setting;
    flash.FlyLimitSettingData[1] = wifi_cmd3_packet.distance_limit_setting;
    flash.FlyLimitSettingData[2] = wifi_cmd3_packet.return_altitude_setting;
    flash.FlyLimitSettingData[3] = wifi_cmd3_packet.green_hands_setting;
    flash.FlyLimitSettingData[4] = wifi_cmd3_packet.stick_mode;
    flash.FlyLimitSettingData[5] = (wifi_cmd3_packet.circle_radius>>8) & 0x00ff;
    flash.FlyLimitSettingData[6] = wifi_cmd3_packet.circle_radius & 0x00ff;
    
    flash.check_sum = get_u8data_checksum((uint8_t *)&flash,FLASH_DATA_BUFFER_LEN-1);
    user_save_param(FLASH_BLOCK0, &flash, FLASH_DATA_BUFFER_LEN);
    user_save_param(FLASH_BLOCK1, &flash, FLASH_DATA_BUFFER_LEN);
}

//读取飞行限制设置数据
bool read_fly_limit_setting_data(void)
{  
    uint8_t checksum;
    
    user_load_param(FLASH_BLOCK0,&flash,FLASH_DATA_BUFFER_LEN);
    checksum = get_u8data_checksum((uint8_t *)&flash,FLASH_DATA_BUFFER_LEN-1);   
    
    if ((checksum == flash.check_sum) && (flash.store_flag == 0x55aa))
    {
        wifi_cmd3_packet.altitude_limit_setting = flash.FlyLimitSettingData[0];
        wifi_cmd3_packet.distance_limit_setting = flash.FlyLimitSettingData[1];
        wifi_cmd3_packet.return_altitude_setting = flash.FlyLimitSettingData[2];
        wifi_cmd3_packet.green_hands_setting = flash.FlyLimitSettingData[3];
        wifi_cmd3_packet.stick_mode = flash.FlyLimitSettingData[4];
        wifi_cmd3_packet.circle_radius = flash.FlyLimitSettingData[5]<<8;
        wifi_cmd3_packet.circle_radius |= flash.FlyLimitSettingData[6];
        return true;
    }
    else
    {
        user_load_param(FLASH_BLOCK1,&flash,FLASH_DATA_BUFFER_LEN);
        checksum = get_u8data_checksum((uint8_t *)&flash,FLASH_DATA_BUFFER_LEN-1);   
        if ((checksum == flash.check_sum) && (flash.store_flag == 0x55aa))
        {
            wifi_cmd3_packet.altitude_limit_setting = flash.FlyLimitSettingData[0];
            wifi_cmd3_packet.distance_limit_setting = flash.FlyLimitSettingData[1];
            wifi_cmd3_packet.return_altitude_setting = flash.FlyLimitSettingData[2];
            wifi_cmd3_packet.green_hands_setting = flash.FlyLimitSettingData[3];
            wifi_cmd3_packet.stick_mode = flash.FlyLimitSettingData[4];
            wifi_cmd3_packet.circle_radius = flash.FlyLimitSettingData[5]<<8;
            wifi_cmd3_packet.circle_radius |= flash.FlyLimitSettingData[6];            
            
            user_save_param(FLASH_BLOCK0, &flash, FLASH_DATA_BUFFER_LEN);
            return true;
        }
        else
        {
            memset(&flash,0,FLASH_DATA_BUFFER_LEN);
            flash.store_flag = 0x55aa;
            flash.check_sum = get_u8data_checksum((uint8_t *)&flash,FLASH_DATA_BUFFER_LEN-1);
            user_save_param(FLASH_BLOCK0, &flash, FLASH_DATA_BUFFER_LEN);
            user_save_param(FLASH_BLOCK1, &flash, FLASH_DATA_BUFFER_LEN);
            return false;
        }
        
    }
}

//更新飞机各功能的当前状态
void update_plane_status_now(plane_status_def plane_function,status_def status)
{
    switch (plane_function)
    {
        case    PLANE_LOCK_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_LOCK_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= ~PLANE_LOCK_FLAG_STATUS;
            }
            break;
        case    PLANE_FLYING_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_FLYING_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= ~PLANE_FLYING_FLAG_STATUS;
            }
            break;
        case    PLANE_GYRO_CALIBRATE_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_GYRO_CALIBRATE_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= ~PLANE_GYRO_CALIBRATE_FLAG_STATUS;
            }
            break;
        case    PLANE_COMPASS_HORIZONTAL_CALIBRATE_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_COMPASS_HORIZONTAL_CALIBRATE_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= PLANE_COMPASS_HORIZONTAL_CALIBRATE_FLAG_STATUS;
            }
            break;
        case    PLANE_COMPASS_VERTICAL_CALIBRATE_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_COMPASS_VERTICAL_CALIBRATE_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= ~PLANE_COMPASS_VERTICAL_CALIBRATE_FLAG_STATUS;
            }
            break;
		case    PLANE_FOLLOW_MODE_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_FOLLOW_MODE_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= ~PLANE_FOLLOW_MODE_FLAG_STATUS;
            }
			break;
        case    PLANE_CIRCLE_MODE_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_CIRCLE_MODE_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= ~PLANE_CIRCLE_MODE_FLAG_STATUS;
            }
            break;
        case    PLANE_POINT_MODE_STATUS:
            if (STATUS_ON == status)
            {
                g_dw_plane_status |= PLANE_POINT_MODE_FLAG_STATUS;
            }
            else
            {
                g_dw_plane_status &= ~PLANE_POINT_MODE_FLAG_STATUS;
            }
			break;        
		case	PLANE_RETURN_HOME_MODE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_RETURN_HOME_MODE_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_RETURN_HOME_MODE_FLAG_STATUS;
			}
			break;
		case	PLANE_LANDING_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_LANDING_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_LANDING_FLAG_STATUS;
			}
			break;
		case	PLANE_LINK_REMOTE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_LINK_REMOTE_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_LINK_REMOTE_FLAG_STATUS;
			}
			break;		
		case	PLANE_HEADLESS_MODE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_HEADLESS_MODE_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_HEADLESS_MODE_FLAG_STATUS;
			}
			break;
		case	PLANE_SCRAM_MODE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_SCRAM_MODE_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_SCRAM_MODE_FLAG_STATUS;
			}
			break;			
		case	PLANE_SPEED_MODE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_SPEED_MODE_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_SPEED_MODE_FLAG_STATUS;
			}
			break;
		case	PLANE_LOW_POWER_MODE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_LOW_POWER_MODE_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_LOW_POWER_MODE_FLAG_STATUS;
			}
			break;
		case	PLANE_GREEN_HANDS_MODE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_GREEN_HANDS_MODE_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_GREEN_HANDS_MODE_FLAG_STATUS;
			}
			break;
		case	PLANE_MPU_ERRO_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_MPU_ERRO_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_MPU_ERRO_FLAG_STATUS;
			}
			break;	
		case	PLANE_BARO_ERRO_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_BARO_ERRO_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_BARO_ERRO_FLAG_STATUS;
			}
			break;
		case	PLANE_GPS_ERRO_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_GPS_ERRO_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_GPS_ERRO_FLAG_STATUS;
			}
			break;	
		case	PLANE_COMPASS_ERRO_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_COMPASS_ERRO_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_COMPASS_ERRO_FLAG_STATUS;
			}
			break;
		case	PLANE_WIND_ERRO_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_WIND_ERRO_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_WIND_ERRO_FLAG_STATUS;
			}
			break;
		case	PLANE_GPS_RSSI_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_GPS_RSSI_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_GPS_RSSI_FLAG_STATUS;
			}
			break;
		case 	PLANE_REMOTE_CHOICE_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_REMOTE_CHOICE_FLAG_STATUS;	//2.4G遥控器
			}
			else
			{
				g_dw_plane_status &= ~PLANE_REMOTE_CHOICE_FLAG_STATUS;	//中继遥控器
			}
			
            break;
		case	PLANE_RISING_STATUS:
			if (STATUS_ON == status)
			{
				g_dw_plane_status |= PLANE_RISING_FLAG_STATUS;
			}
			else
			{
				g_dw_plane_status &= ~PLANE_RISING_FLAG_STATUS;
			}
			break;
        default:
            break;
    }   
}

//获取飞机各功能的当前状态
status_def get_plane_status_now(plane_status_def plane_function)
{
    status_def status_bit = STATUS_OFF;
    switch (plane_function)
    {
        case    PLANE_LOCK_STATUS:
            if (g_dw_plane_status & PLANE_LOCK_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;
        case    PLANE_FLYING_STATUS:
            if (g_dw_plane_status & PLANE_FLYING_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;
        case    PLANE_GYRO_CALIBRATE_STATUS:
            if (g_dw_plane_status & PLANE_GYRO_CALIBRATE_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;            
        case    PLANE_COMPASS_HORIZONTAL_CALIBRATE_STATUS:
            if (g_dw_plane_status & PLANE_COMPASS_HORIZONTAL_CALIBRATE_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;
        case    PLANE_COMPASS_VERTICAL_CALIBRATE_STATUS:
            if (g_dw_plane_status & PLANE_COMPASS_VERTICAL_CALIBRATE_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;
        case    PLANE_FOLLOW_MODE_STATUS:
            if (g_dw_plane_status & PLANE_FOLLOW_MODE_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;   
		case    PLANE_CIRCLE_MODE_STATUS:
            if (g_dw_plane_status & PLANE_COMPASS_VERTICAL_CALIBRATE_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;
        case    PLANE_POINT_MODE_STATUS:
            if (g_dw_plane_status & PLANE_POINT_MODE_FLAG_STATUS)
                status_bit = STATUS_ON;
            else
                status_bit = STATUS_OFF;
            break;
		case	PLANE_RETURN_HOME_MODE_STATUS:
			if (g_dw_plane_status & PLANE_RETURN_HOME_MODE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_LANDING_STATUS:
			if (g_dw_plane_status & PLANE_LANDING_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_LINK_REMOTE_STATUS:
			if (g_dw_plane_status & PLANE_LINK_REMOTE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_HEADLESS_MODE_STATUS:
			if (g_dw_plane_status & PLANE_HEADLESS_MODE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_SCRAM_MODE_STATUS:
			if (g_dw_plane_status & PLANE_SCRAM_MODE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_SPEED_MODE_STATUS:
			if (g_dw_plane_status & PLANE_SPEED_MODE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_LOW_POWER_MODE_STATUS:
			if (g_dw_plane_status & PLANE_LOW_POWER_MODE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_GREEN_HANDS_MODE_STATUS:
			if (g_dw_plane_status & PLANE_GREEN_HANDS_MODE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_MPU_ERRO_STATUS:
			if (g_dw_plane_status & PLANE_MPU_ERRO_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_BARO_ERRO_STATUS:
			if (g_dw_plane_status & PLANE_BARO_ERRO_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_GPS_ERRO_STATUS:
			if (g_dw_plane_status & PLANE_GPS_ERRO_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_COMPASS_ERRO_STATUS:
			if (g_dw_plane_status & PLANE_COMPASS_ERRO_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_WIND_ERRO_STATUS:
			if (g_dw_plane_status & PLANE_WIND_ERRO_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_GPS_RSSI_STATUS:
			if (g_dw_plane_status & PLANE_GPS_RSSI_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_REMOTE_CHOICE_STATUS:
			if (g_dw_plane_status & PLANE_REMOTE_CHOICE_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
		case	PLANE_RISING_STATUS:
			if (g_dw_plane_status & PLANE_RISING_FLAG_STATUS)
				status_bit = STATUS_ON;
			else
				status_bit = STATUS_OFF;
			break;
        default:            
            break;        
    }   
    return status_bit;
}

void plane_function_status_update_to_wifi(void)
{
    static uint16_t w_testCouter = 0;
    //app 解锁测试
    if (get_phone_control_lock_info())
    {
        update_plane_status_now(PLANE_LOCK_STATUS,STATUS_ON);
    }
	else
	{
		update_plane_status_now(PLANE_LOCK_STATUS,STATUS_OFF);
	}
    
    
    //地磁校准测试
    if (get_phone_control_compass_calibrate())
    {        
        w_testCouter++;
        if (w_testCouter < 10000)
        {
            set_plane_cmd2_compass_horizontal_calibrate_status();
        }
        else if (w_testCouter < 20000)
        {
            clear_plane_cmd2_compass_horizontal_calibrate_status();
            set_plane_cmd2_compass_vertical_calibrate_status();
        }
        else
        {
            clear_phone_control_compass_calibrate();
            clear_plane_cmd2_compass_vertical_calibrate_status();
            w_testCouter = 0;
        }            
    }
    //指点飞行测试
    if (get_phone_control_point_fly())
    {
        if (wifi_cmd6_packet.waypoint_number)
        {            
            set_plane_cmd2_point_mode();  
            update_plane_status_now(PLANE_POINT_MODE_STATUS,STATUS_ON);
        }
    }
    else
    {
        if (STATUS_ON == get_plane_status_now(PLANE_POINT_MODE_STATUS))
        {
            update_plane_status_now(PLANE_POINT_MODE_STATUS,STATUS_OFF);
            clear_phone_control_point_fly();
            clear_plane_cmd2_point_mode();
            wifi_cmd6_packet.waypoint_number = 0;
        }       
    }
    //热点环绕测试
//    if (get_phone_control_circle())
//    {
//        w_testCouter++;
//        if (w_testCouter < 20000)
//        {
//            set_plane_cmd2_circle_mode();
//        }
//        else
//        {            
//            clear_phone_control_circle();
//            clear_plane_cmd2_circle_mode();
//            w_testCouter = 0;
//        }
//    }
//    else
//    {
//        clear_phone_control_circle();
//        clear_plane_cmd2_circle_mode();
//        w_testCouter = 0;
//    }
    
    //跟随测试
    if (get_phone_control_follow_mode())
    {
        set_plane_cmd2_follow_mode();
        
    }
    else
    {
//        clear_phone_control_follow_mode();
        clear_plane_cmd2_follow_mode();
    }

	//一键起飞测试
	if (get_phone_control_rising_landing())
	{
		clear_phone_control_rising_landing();
		if (STATUS_ON == get_plane_status_now(PLANE_FLYING_STATUS))
		{
			update_plane_status_now(PLANE_LANDING_STATUS,STATUS_ON);
		}
		else
		{
			update_plane_status_now(PLANE_RISING_STATUS,STATUS_ON);
		}
	}
	//APP返航测试
	if (get_phone_control_return_home_mode())
	{
		update_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS, STATUS_ON);
	}
	else
	{
		update_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS, STATUS_OFF);
	}

	//一键起飞
	if (STATUS_ON == get_plane_status_now(PLANE_RISING_STATUS))
	{
		w_testCouter++;
        if (w_testCouter < 10000)
        {
            set_plane_cmd2_rising_mode();
        }
        else
        {            
            clear_plane_cmd2_rising_mode();
			update_plane_status_now(PLANE_RISING_STATUS, STATUS_OFF);
            update_plane_status_now(PLANE_FLYING_STATUS, STATUS_ON);
            w_testCouter = 0;
        }
	}
	//一键降落
	if (STATUS_ON == get_plane_status_now(PLANE_LANDING_STATUS))
	{
		w_testCouter++;
        if (w_testCouter < 10000)
        {
            set_plane_cmd2_landing_mode();
        }
        else
        {            
            clear_plane_cmd2_landing_mode();
			update_plane_status_now(PLANE_LANDING_STATUS, STATUS_OFF);
            update_plane_status_now(PLANE_FLYING_STATUS, STATUS_OFF);
            w_testCouter = 0;
        }
	}


	
	if (STATUS_ON == get_plane_status_now(PLANE_LOCK_STATUS))
    {
        clear_plane_cmd2_lock_status();
    }
    else
    {
        set_plane_cmd2_lock_status();
    }
	if (STATUS_ON == get_plane_status_now(PLANE_FLYING_STATUS))
	{
		set_plane_cmd2_flying_status();
	}
	else
	{
		clear_plane_cmd2_flying_status();
	}
	if (STATUS_ON == get_plane_status_now(PLANE_GYRO_CALIBRATE_STATUS))
	{
		set_plane_cmd2_gyro_calibrate_status();
	}
	else
	{
		clear_plane_cmd2_gyro_calibrate_status();
	}
	 if (STATUS_ON == get_plane_status_now(PLANE_COMPASS_HORIZONTAL_CALIBRATE_STATUS))
    {
        set_plane_cmd2_compass_horizontal_calibrate_status();
    }
    else
    {
        clear_plane_cmd2_compass_horizontal_calibrate_status();
    }
    if (STATUS_ON == get_plane_status_now(PLANE_COMPASS_VERTICAL_CALIBRATE_STATUS))
    {
        set_plane_cmd2_compass_vertical_calibrate_status();
    }
    else
    {
        clear_plane_cmd2_compass_vertical_calibrate_status();
    }
	if (STATUS_ON == get_plane_status_now(PLANE_FOLLOW_MODE_STATUS))
    {
        set_plane_cmd2_follow_mode();
    }
    else
    {
        clear_plane_cmd2_follow_mode();
    }
	 if (STATUS_ON == get_plane_status_now(PLANE_CIRCLE_MODE_STATUS))
    {
        set_plane_cmd2_circle_mode();
    }
    else
    {
        clear_plane_cmd2_circle_mode();
    }
    if (STATUS_ON == get_plane_status_now(PLANE_POINT_MODE_STATUS))
    {
        set_plane_cmd2_point_mode();
    }
    else
    {
        clear_plane_cmd2_point_mode();
    }
	if (STATUS_ON == get_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS))
	{
		set_plane_cmd2_return_home_mode();
	}
	else
	{
		clear_plane_cmd2_return_noome_mode(); 
	}
    if (STATUS_ON == get_plane_status_now(PLANE_LANDING_STATUS))
    {
		set_plane_cmd2_landing_mode();
	}
	else
	{
		clear_plane_cmd2_landing_mode();
	}	
    if (STATUS_ON == get_plane_status_now(PLANE_LINK_REMOTE_STATUS))
    {
		set_plane_cmd2_link_remoter_status();
    }
	else
	{
		clear_plane_cmd2_link_remoter_status();
	}
	if (STATUS_ON == get_plane_status_now(PLANE_HEADLESS_MODE_STATUS))
    {
		set_plane_cmd2_headless_mode();
    }
	else
	{
		clear_plane_cmd2_headless_mode();
	}
	if (STATUS_ON == get_plane_status_now(PLANE_SCRAM_MODE_STATUS))
    {
		set_plane_cmd2_scram_mode();
    }
	else
	{
		clear_plane_cmd2_scram_mode();
	}
	if (STATUS_ON == get_plane_status_now(PLANE_SPEED_MODE_STATUS))
    {
		set_plane_cmd2_high_speed_status();
    }
	else
	{
		set_plane_cmd2_low_speed_status();
	} 
	if (STATUS_ON == get_plane_status_now(PLANE_LOW_POWER_MODE_STATUS))
    {
		set_plane_cmd2_low_voltage_status();
    }
	else
	{
		clear_plane_cmd2_low_voltage_status();
	} 
	if (STATUS_ON == get_plane_status_now(PLANE_GREEN_HANDS_MODE_STATUS))
    {
		set_plane_cmd2_green_hands_mode();
    }
	else
	{
		clear_plane_cmd2_green_hands_mode();
	} 
	if (STATUS_ON == get_plane_status_now(PLANE_MPU_ERRO_STATUS))
    {
		set_plane_cmd2_mpu_erro();
    }
	else
	{
		clear_plane_cmd2_mpu_erro();
	} 
	if (STATUS_ON == get_plane_status_now(PLANE_BARO_ERRO_STATUS))
    {
		set_plane_cmd2_baro_erro();
    }
	else
	{
		clear_plane_cmd2_baro_erro();
	} 
	if (STATUS_ON == get_plane_status_now(PLANE_GPS_ERRO_STATUS))
    {
		set_plane_cmd2_gps_erro();
    }
	else
	{
		clear_plane_cmd2_gps_erro();
	} 
	if (STATUS_ON == get_plane_status_now(PLANE_COMPASS_ERRO_STATUS))
    {
		set_plane_cmd2_compass_erro();
    }
	else
	{
		clear_plane_cmd2_compass_erro();
	}
	if (STATUS_ON == get_plane_status_now(PLANE_WIND_ERRO_STATUS))
    {
		set_plane_cmd2_wind_erro();
    }
	else
	{
		clear_plane_cmd2_wind_erro();
	}
	if (STATUS_ON == get_plane_status_now(PLANE_GPS_RSSI_STATUS))
    {
		set_plane_cmd2_gps_rssi_erro();
    }
	else
	{
		clear_plane_cmd2_gps_rssi_erro();
	} 
	if (STATUS_ON == get_plane_status_now(PLANE_REMOTE_CHOICE_STATUS))
	{
		set_plane_cmd2_remote_choice_24G();
	}
	else
	{
		set_plane_cmd2_remote_choice_repeater();
    }
	if (STATUS_ON == get_plane_status_now(PLANE_RISING_STATUS))
    {
		set_plane_cmd2_rising_mode();
    }
	else
	{
		clear_plane_cmd2_rising_mode();
	} 

    
}

//void wifi_function_dispose(void)
//{
//    wifi_link_data_receive();
//    plane_function_status_update_to_wifi();
//    wifi_link_data_send();    
//}





















