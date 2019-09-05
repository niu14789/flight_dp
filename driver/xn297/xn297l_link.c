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
	* LED is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "xn297.h"
#include "state.h"
#include "gd32f30x.h"
#include "string.h"
#include "wifi_link.h"

//test use
uint16_t test_counter = 0;
uint8_t receive_couter = 0;
uint8_t receive_couter_backup = 0; 


#define PLANE_FUNC  1

remote_packet_stru remote_packet;
uint8_t by_remote_function_status_backup;
__IO uint8_t by_remote_function_status_now;

uint8_t get_remote_mode_function_control(void) { return (by_remote_function_status_now & MODE_FUNCTION_BIT); }
void clear_remote_mode_function_control(void) { by_remote_function_status_now &= ~MODE_FUNCTION_BIT; }
uint8_t get_remote_return_home_function_control(void) { return (by_remote_function_status_now & RETURN_HOME_FUNCTION_BIT); }
void clear_remote_return_home_function_control(void) { by_remote_function_status_now &= ~RETURN_HOME_FUNCTION_BIT; }
uint8_t get_remote_photo_function_control(void) { return (by_remote_function_status_now & PHOTO_FUNCTION_BIT); }
void clear_remote_photo_function_control(void) { by_remote_function_status_now &= ~PHOTO_FUNCTION_BIT; }
uint8_t get_remote_record_function_control(void) { return (by_remote_function_status_now & VIDEO_FUNCTION_BIT); }
void clear_remote_record_function_control(void) { by_remote_function_status_now &= ~VIDEO_FUNCTION_BIT; }
uint8_t get_remote_rise_land_function_control(void) { return (by_remote_function_status_now & RISE_LAND_FUNCTION_BIT); }
void clear_remote_rise_land_function_control(void) { by_remote_function_status_now &= ~RISE_LAND_FUNCTION_BIT; }

//频点图


//频点图
const uint8_t frequency_channal_tab[FREQ_CH_NUMBER] = {4,12,18,27,36,41,47,54,62,72};

//调用频率为1000HZ
void rf_link_timer(void)
{
    s_rf_link.link_timer_counter++;
    if(s_rf_link.link_timer_counter == RF_LINK_TX_TIME)
    {
        s_rf_link.link_step = RF_STEP_TX;        
    }
	if(s_rf_link.link_timer_counter == RF_LINK_RX_TIME)
    {
        s_rf_link.link_step = RF_STEP_RX;        
    }
    if(s_rf_link.link_timer_counter == RF_HOPPING_TIME)
    {        
        s_rf_link.link_step = RF_STEP_HOPPING;  
    }
    if(s_rf_link.link_timer_counter >= RF_LINK_CYCLE)
    {                  
        s_rf_link.link_timer_counter = 0;      
    }
}

//连接初始化
void rf_link_init(void)
{
    s_rf_link.link_timer_counter = 0;
    s_rf_link.hopping_counter = 0;
    s_rf_link.link_status = DISCONNECCTING;
    s_rf_link.link_step = RF_STEP_HOPPING;
    s_rf_link.binding_status = 1;
    by_remote_function_status_backup = 0;
    
#if (RF_WORK_MODE == TX_MODE)
    RF_TxMode();
#elif (RF_WORK_MODE == RX_MODE)
    rf_set_rx();
#endif  
   
}

//更新飞机拍照状态
void update_plane_photo_function_to_remote(void)
{
    //拍照有效，状态翻转
    s_rf_link.plane_function_flag ^= BIT6;
}
//更新飞机录像状态
void update_plane_video_function_to_remote(void)
{
    //录像有效，状态翻转
    s_rf_link.plane_function_flag ^= BIT7;
}
#if PLANE_FUNC
//飞机功能状态更新
void plane_function_flag_update(void)
{
    if(1 == get_plane_status_now(PLANE_LOCK_STATUS))
    {
        s_rf_link.plane_function_flag |= BIT0;
    }
    else
    {
        s_rf_link.plane_function_flag &= ~BIT0;
    }
    
    if (get_plane_status_now(PLANE_HEADLESS_MODE_STATUS))
    {
        s_rf_link.plane_function_flag |= BIT2;
    }
    else
    {
        s_rf_link.plane_function_flag &= ~BIT2;
    }
    
    if (get_plane_status_now(PLANE_LANDING_STATUS))
    {
        s_rf_link.plane_function_flag |= BIT4;
    }
    else
    {
        s_rf_link.plane_function_flag &= ~BIT4;
    }
    
    if (get_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS))
    {
        s_rf_link.plane_function_flag |= BIT5;
    }
    else
    {
        s_rf_link.plane_function_flag &= ~BIT5;
    }
    
    if (get_plane_status_now(PLANE_LOW_POWER_MODE_STATUS))
    {
        s_rf_link.plane_function_flag |= BIT8;
    }
    else
    {
        s_rf_link.plane_function_flag &= ~BIT8;
    }
    
    if (get_plane_status_now(PLANE_FLYING_STATUS))
    {        
        s_rf_link.plane_function_flag |= BIT12;
    }
    else
    {
        s_rf_link.plane_function_flag &= ~BIT12;
    }
}
#endif
extern power_user_s    bat_user;   /* user battery voltage */
void rf_normal_data_packet(void)
{
    uint16_t u16_temp;
#if PLANE_FUNC	
    u16_temp = (uint16_t)(bat_user.bat_voltage * 100);
    plane_function_flag_update();
#else
	  u16_temp = 16000;//16V
#endif	
    s_rf_link.tx_buffer[0] = 0xA2;
    s_rf_link.tx_buffer[1] = 17;
    s_rf_link.tx_buffer[2] = 0xF0;
    s_rf_link.tx_buffer[3] = (uint8_t)((s_rf_link.plane_function_flag >> 8) & 0x00ff) ;
    s_rf_link.tx_buffer[4] = (uint8_t)(s_rf_link.plane_function_flag & 0x00ff);
    s_rf_link.tx_buffer[5] = 0;
    s_rf_link.tx_buffer[6] = receive_couter_backup;
    s_rf_link.tx_buffer[7] = 0;
    s_rf_link.tx_buffer[8] = 0;
    s_rf_link.tx_buffer[9] = 0;
    s_rf_link.tx_buffer[10] = 0;
    s_rf_link.tx_buffer[11] = 0;
    s_rf_link.tx_buffer[12] = 0;
    s_rf_link.tx_buffer[13] = 0;
    s_rf_link.tx_buffer[14] = 0;
    s_rf_link.tx_buffer[15] = 0;
    s_rf_link.tx_buffer[16] = 0;
    s_rf_link.tx_buffer[17] = (uint8_t)((u16_temp >> 8) & 0x00ff);
    s_rf_link.tx_buffer[18] = (uint8_t)(u16_temp & 0x00ff);
    s_rf_link.tx_buffer[19] = rf_checksum(&s_rf_link.tx_buffer[1],18);        
}

//传输数据打包
void rf_link_packet(void)
{
	rf_normal_data_packet();   
}

#if PLANE_FUNC
//2.4G遥控器能控制处理
void remote_function_control(void)
{
    if (s_rf_link.link_status == CONNECCTING )
    {
        //处于录像状态，拍照无效,只检测录像控制
        if (get_remote_record_function_control())
        {
			clear_remote_record_function_control();
            //有录像控制命令，关闭录像
            update_plane_cmd2_video_function_to_wifi();
            //更新飞机录像状态
            update_plane_video_function_to_remote(); 
        }
        //查询录像状态
 //       if (!get_camera_video_status())
 //       {     			
            //不在录像状态，拍照控制
            if (get_remote_photo_function_control())
            {
				clear_remote_photo_function_control();
                //有拍照控制命令，控制拍照
                update_plane_cmd2_photo_function_to_wifi();
                //更新飞机拍照状态
                update_plane_photo_function_to_remote();
            }
 //       }
        //返航功能控制
        if (get_remote_return_home_function_control())
        {       
            clear_remote_return_home_function_control();
            //更新飞机返航状态 
            if (STATUS_OFF == get_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS))             
                update_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS,STATUS_ON); 
            else
                update_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS,STATUS_OFF);
        }
        
        //起飞降落
        if (get_remote_rise_land_function_control())
        {
			clear_remote_rise_land_function_control();
            //飞机出于起飞状态，置飞机降落状态
            if (STATUS_ON == get_plane_status_now(PLANE_FLYING_STATUS))
            {
                update_plane_status_now(PLANE_LANDING_STATUS,STATUS_OFF);
            }
			else
			{				
				update_plane_status_now(PLANE_RISING_STATUS,STATUS_ON);
			}
        }
		//模式功能
//		if (get_remote_mode_function_control())
//		{
//			//更新飞机模式状态
//            update_plane_status_now(PLANE_SPEED_MODE_STATUS,STATUS_ON);
//        }
//        else
//        {
//            update_plane_status_now(PLANE_SPEED_MODE_STATUS,STATUS_OFF);		
//		}
		//camera ev+
		if (remote_packet.camera_zoom > 0xd50)
		{
			set_plane_cmd2_photo_ev_inc();
		}
		else
		{
			clear_plane_cmd2_photo_ev_inc();
		}
		if (remote_packet.camera_zoom < 0xb20)
		{
			set_plane_cmd2_photo_ev_dec();
		}
		else
		{
			clear_plane_cmd2_photo_ev_dec();
		}			
    }
}
#endif
//遥控数据更新
void rf_remote_data_updata(void)
{
    remote_packet.key_status = s_rf_link.rx_buffer[KEY_STATUS_INDEX];
    remote_packet.function_status = s_rf_link.rx_buffer[FUNCTION_STATUS_INDEX];
    remote_packet.throttle = (uint16_t)(s_rf_link.rx_buffer[THROTTLE_INDEX]<<8 | s_rf_link.rx_buffer[THROTTLE_INDEX+1]);
    remote_packet.aileron = (uint16_t)(s_rf_link.rx_buffer[AILERON_INDEX]<<8 | s_rf_link.rx_buffer[AILERON_INDEX+1]);
    remote_packet.elevator = (uint16_t)(s_rf_link.rx_buffer[ELEVATOR_INDEX]<<8 | s_rf_link.rx_buffer[ELEVATOR_INDEX+1]);
    remote_packet.rudder = (uint16_t)(s_rf_link.rx_buffer[RUDDER_INDEX]<<8 | s_rf_link.rx_buffer[RUDDER_INDEX+1]);
    remote_packet.yuntai = (uint16_t)(s_rf_link.rx_buffer[YUNTAI_INDEX]<<8 | s_rf_link.rx_buffer[YUNTAI_INDEX+1]);
    remote_packet.camera_zoom = (uint16_t)(s_rf_link.rx_buffer[CAMERA_INDEX]<<8 | s_rf_link.rx_buffer[CAMERA_INDEX+1]);
    remote_packet.battery_voltage = (uint16_t)(s_rf_link.rx_buffer[BATTERY_INDEX]<<8 | s_rf_link.rx_buffer[BATTERY_INDEX+1]);
#if PLANE_FUNC    
    remote_function_control();
#endif	
}

//跳频切换频道
void rf_link_frequency_hopping(void)
{
    s_rf_link.hopping_counter++;
    if (s_rf_link.hopping_counter >=10)
    {
        s_rf_link.hopping_counter = 0;
    }
    rf_set_channel(frequency_channal_tab[s_rf_link.hopping_counter]);
}

//失联状态判断，重连编制置位
void rf_relink_detect(void)
{
	if (s_rf_link.loss_link_counter < 6)
	{    
		s_rf_link.loss_link_counter++;
		s_rf_link.link_status = CONNECCTING;
#if PLANE_FUNC
		update_plane_status_now(PLANE_LINK_REMOTE_STATUS, STATUS_ON);
		update_plane_status_now(PLANE_REMOTE_CHOICE_STATUS, STATUS_ON);	
#endif			
	}
	else
	{
		s_rf_link.link_status = DISCONNECCTING;
#if PLANE_FUNC			
		update_plane_status_now(PLANE_LINK_REMOTE_STATUS, STATUS_OFF);
#endif		
	}    
}

#if 0
//连接通讯功能
uint8_t rev_status;

void rf_link_test(void)
{      
    rev_status = ucRF_GetStatus(); 
    if (rev_status & RX_DR_FLAG)		                                           
    {  
        RF_ReadBuf(R_RX_PAYLOAD,s_rf_link.rx_buffer, PAYLOAD_WIDTH); 
        rf_link_packet();
        RF_Tx_TransmintData(s_rf_link.tx_buffer,PAYLOAD_WIDTH);        
        DelayMs(4);
        RF_ClearFIFO();
        RF_ClearStatus ();   
    }   
}
#endif

void rf_link_function(void)
{
    test_counter++;
    if(test_counter >= 2000)
    {   
        test_counter = 0;
        receive_couter_backup = receive_couter;
        receive_couter = 0;
    }
    
    if (s_rf_link.link_step == RF_STEP_HOPPING)
    {
        //重连状态检测
        rf_relink_detect();
        if (s_rf_link.link_status == DISCONNECCTING)
        {
            //失联则将正常频点设置至频点图第一个频点
            s_rf_link.hopping_counter = 0;
            //设置    重连是RF通讯通道
            rf_set_channel(RF_RELINK_CH);
        }
        else
        {
            //正常通讯跳频
            rf_link_frequency_hopping();
//            RF_SetChannel(RF_NORMAL_CH);            
        } 
        s_rf_link.link_step = RF_STEP_RX;
        
    }       
    //失联时一直处于接收状态，以便快速连接上
    else if ((s_rf_link.link_step == RF_STEP_RX) || (s_rf_link.link_status == DISCONNECCTING))  
    {        
        unsigned char rev_status = rf_get_statue(); 
			
        if (rev_status & RX_DR_FLAG)
        {
            receive_couter++;
            //接收成功，读取数据
            rf_read_multiple(R_RX_PAYLOAD,s_rf_link.rx_buffer, PAYLOAD_WIDTH);                                   
            s_rf_link.loss_link_counter = 0;  
            //更新RF数据
            rf_remote_data_updata();
            if(s_rf_link.link_status == CONNECCTING)
            {
                //遥控器功能处理 
                //一键起飞降落、返航键功能、拍照功能、录像功能获取，翻转逻辑，按下为1，再按为0
                by_remote_function_status_now ^= (by_remote_function_status_backup & 0x1e) ^ (remote_packet.function_status & 0x1e);
                //急停功能，启动为1，否则为0, 模式功能，正常为0，其他为1
                by_remote_function_status_now &= ~0x60;
                by_remote_function_status_now |= remote_packet.function_status & 0x60;
            }
            by_remote_function_status_backup = remote_packet.function_status;
            s_rf_link.link_status = CONNECCTING;
            s_rf_link.link_step = RF_STEP_NULL;            
            //接收成功，时钟同步
            s_rf_link.link_timer_counter = 3;             
            rf_link_packet();
            rf_tx_transmintdata(s_rf_link.tx_buffer,PAYLOAD_WIDTH); 
        }
    }
    //失联时，未接收成功就不进人发送状态
    else if ((s_rf_link.link_step == RF_STEP_TX) && (s_rf_link.link_status == CONNECCTING)) 
    {  
        rf_clear_status();
        rf_clear_fifo ();   
        s_rf_link.link_step = RF_STEP_NULL;
    }  
}
#if 0
//RF 新地址根据ADC值和timer0数值共同产生
uint16_t rf_new_address_generate(void)
{
    uint16_t u16_temp;
    u16_temp = ADC_RDATA(ADC1);
    u16_temp ^= (uint16_t)(SysTick->VAL);
    return u16_temp;
}

//RF 对码
void rf_binding(void)
{    
    uint8_t u8_binding_step;
    uint8_t u8_binding_success_flag;    
    uint8_t u8_temp,exit_binding_counter;
    uint16_t u16_temp;
    
    uint32_t binding_timer = 0;
    u8_binding_step = 0;
    u8_binding_success_flag = false;    
    
    //判断是否需要对码，不需则返回
    if (s_rf_link.binding_status == STATUS_OFF)
        return;
    //对码前产生新ID
    u16_temp = rf_new_address_generate(); 
    //对码时使用默认ID及对码特点频点
    rf_write_address((uint8_t *)TX_ADDRESS_DEF);
    RF_SetChannel(RF_BINDONG_CH);
    //对码时降低发射功率，控制对码距离
    //rf_set_power(RF0dBm);    
   
    //对码前产生新ID
    s_rf_link.rf_new_address[2] = (uint8_t)(u16_temp>>8 & 0x00ff);
    s_rf_link.rf_new_address[3] = (uint8_t)(u16_temp & 0x00ff); 
    //接收端开始设置为接收模式
//    RF_ClearStatus (); 
    RF_RxMode();
    u8_temp = 0;
    set_led_binding_flicker();
    while(u8_binding_success_flag == false)
    {       
        if((GetTickCountMs() % 10) == 0)
		{   
            binding_timer++;
            exit_binding_counter++;
            led_flicker();

            //30秒未对码成功退出对码状态
            if (binding_timer >= 3000)
            {                
                break;                
            }
            //5秒未接收成功退出对码状态
            if ((0 == u8_binding_step) && (binding_timer >= 500))
            { 
                break;
            } 
            DelayMs(1);
        }
        switch (u8_binding_step)
        {                
            case    0:   
                //判断是否接收成功                   
                 u8_temp = ucRF_GetStatus(); 
                if (u8_temp & RX_DR_FLAG)		                                           
                {  
                    RF_ReadBuf(R_RX_PAYLOAD,s_rf_link.rx_buffer, PAYLOAD_WIDTH);            
//                    if (get_u8data_checksum(s_rf_link.rx_buffer ,PAYLOAD_WIDTH - 1) == s_rf_link.rx_buffer[PAYLOAD_WIDTH - 1])
//                    {
                        //接收正确进入对码状态，设置对码灯闪
                        set_led_binding_flicker();
                        u8_temp = 0;
                        s_rf_link.tx_buffer[0] = RF_LINK_BUFFER_HEAD;
                        s_rf_link.tx_buffer[1] = s_rf_link.rf_new_address[0];
                        s_rf_link.tx_buffer[2] = s_rf_link.rf_new_address[1];
                        s_rf_link.tx_buffer[3] = s_rf_link.rf_new_address[2];
                        s_rf_link.tx_buffer[4] = s_rf_link.rf_new_address[3];
                        s_rf_link.tx_buffer[5] = s_rf_link.rf_new_address[4];
                        s_rf_link.tx_buffer[6] = u8_binding_step | 0x10;
                        s_rf_link.tx_buffer[PAYLOAD_WIDTH-1] = get_u8data_checksum(s_rf_link.tx_buffer,PAYLOAD_WIDTH-1); 
                        RF_Tx_TransmintData(s_rf_link.tx_buffer,PAYLOAD_WIDTH);
                        if(s_rf_link.rx_buffer[6] == 2)
                        {
                            u8_binding_step++; 
                        }                            
                        DelayMs(4);
//                    } 
                    RF_ClearFIFO();
                    RF_ClearStatus ();                        
                }
                break;
            case    1: 
            case    2:
                //判断是否接收成功                   
                 u8_temp = ucRF_GetStatus(); 
                if (u8_temp & RX_DR_FLAG)		                                           
                { 
                    exit_binding_counter = 0;
                    if (read_rf_receive_data(s_rf_link.rx_buffer,PAYLOAD_WIDTH) == true)
                    {
                        //校验正确跳转至下一步骤
                        u8_temp = 0;                                                
                        //保存新的ID
                        s_rf_link.rf_new_address[0] = s_rf_link.rx_buffer[1];
                        s_rf_link.rf_new_address[1] = s_rf_link.rx_buffer[2];
                        s_rf_link.rf_new_address[2] = s_rf_link.rx_buffer[3];
                        s_rf_link.rf_new_address[3] = s_rf_link.rx_buffer[4];
                        s_rf_link.rf_new_address[4] = s_rf_link.rx_buffer[5];
                        
                        s_rf_link.remoter_version[0] = s_rf_link.rx_buffer[10];
                        s_rf_link.remoter_version[1] = s_rf_link.rx_buffer[11];
                        s_rf_link.remoter_version[2] = s_rf_link.rx_buffer[12];
                        //数据打包                                
                        s_rf_link.tx_buffer[0] = RF_LINK_BUFFER_HEAD;
                        s_rf_link.tx_buffer[1] = s_rf_link.rf_new_address[0];
                        s_rf_link.tx_buffer[2] = s_rf_link.rf_new_address[1];
                        s_rf_link.tx_buffer[3] = s_rf_link.rf_new_address[2];
                        s_rf_link.tx_buffer[4] = s_rf_link.rf_new_address[3];
                        s_rf_link.tx_buffer[5] = s_rf_link.rf_new_address[4];
                        s_rf_link.tx_buffer[6] = u8_binding_step | 0x10; 
                        s_rf_link.tx_buffer[PAYLOAD_WIDTH-1] = get_u8data_checksum(s_rf_link.tx_buffer,PAYLOAD_WIDTH-1); 
                        RF_Tx_TransmintData(s_rf_link.tx_buffer,PAYLOAD_WIDTH);
                        if (u8_binding_step < 2)
                        {
                            u8_binding_step++; 
                        }
                        DelayMs(4);
                    }                      
                    RF_ClearFIFO();
                    RF_ClearStatus ();                         
                }      
                else
                {
                    if (u8_binding_step == 2)
                    {                           
                        //等待一段时间，确保发射端接收上步骤数据成功后再成功退出对码状态                        
                        if (exit_binding_counter >100)
                        {
                            s_rf_link.binding_status = STATUS_OFF;
                            u8_binding_success_flag = true;                             
                        }                                
                    }
                }
                break; 
            default:
                break;            
        }            
               
    } 
    RF_Init();
    rf_link_init();
    if(u8_binding_success_flag == true)
    {
        rf_id_store();
        rf_write_address(s_rf_link.rf_new_address);
    }
    else
    {
        if (true == rf_id_read())
        {
            rf_write_address(s_rf_link.rf_new_address);
        }
        else
        {
            rf_write_address((uint8_t *)TX_ADDRESS_DEF);
        }
    }
    RF_RxMode();
}

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
//读取RF 接收数据并校验
bool read_rf_receive_data(uint8_t *pbuf,uint8_t len)
{
    RF_ReadBuf(R_RX_PAYLOAD,pbuf, len); 
    if (get_u8data_checksum(pbuf ,len - 1) == pbuf[len - 1])
    {
        return true;
    }
    else
    {
        return false;
    }
}
#endif


































