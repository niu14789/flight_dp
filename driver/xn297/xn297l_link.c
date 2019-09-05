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

//Ƶ��ͼ


//Ƶ��ͼ
const uint8_t frequency_channal_tab[FREQ_CH_NUMBER] = {4,12,18,27,36,41,47,54,62,72};

//����Ƶ��Ϊ1000HZ
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

//���ӳ�ʼ��
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

//���·ɻ�����״̬
void update_plane_photo_function_to_remote(void)
{
    //������Ч��״̬��ת
    s_rf_link.plane_function_flag ^= BIT6;
}
//���·ɻ�¼��״̬
void update_plane_video_function_to_remote(void)
{
    //¼����Ч��״̬��ת
    s_rf_link.plane_function_flag ^= BIT7;
}
#if PLANE_FUNC
//�ɻ�����״̬����
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

//�������ݴ��
void rf_link_packet(void)
{
	rf_normal_data_packet();   
}

#if PLANE_FUNC
//2.4Gң�����ܿ��ƴ���
void remote_function_control(void)
{
    if (s_rf_link.link_status == CONNECCTING )
    {
        //����¼��״̬��������Ч,ֻ���¼�����
        if (get_remote_record_function_control())
        {
			clear_remote_record_function_control();
            //��¼���������ر�¼��
            update_plane_cmd2_video_function_to_wifi();
            //���·ɻ�¼��״̬
            update_plane_video_function_to_remote(); 
        }
        //��ѯ¼��״̬
 //       if (!get_camera_video_status())
 //       {     			
            //����¼��״̬�����տ���
            if (get_remote_photo_function_control())
            {
				clear_remote_photo_function_control();
                //�����տ��������������
                update_plane_cmd2_photo_function_to_wifi();
                //���·ɻ�����״̬
                update_plane_photo_function_to_remote();
            }
 //       }
        //�������ܿ���
        if (get_remote_return_home_function_control())
        {       
            clear_remote_return_home_function_control();
            //���·ɻ�����״̬ 
            if (STATUS_OFF == get_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS))             
                update_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS,STATUS_ON); 
            else
                update_plane_status_now(PLANE_RETURN_HOME_MODE_STATUS,STATUS_OFF);
        }
        
        //��ɽ���
        if (get_remote_rise_land_function_control())
        {
			clear_remote_rise_land_function_control();
            //�ɻ��������״̬���÷ɻ�����״̬
            if (STATUS_ON == get_plane_status_now(PLANE_FLYING_STATUS))
            {
                update_plane_status_now(PLANE_LANDING_STATUS,STATUS_OFF);
            }
			else
			{				
				update_plane_status_now(PLANE_RISING_STATUS,STATUS_ON);
			}
        }
		//ģʽ����
//		if (get_remote_mode_function_control())
//		{
//			//���·ɻ�ģʽ״̬
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
//ң�����ݸ���
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

//��Ƶ�л�Ƶ��
void rf_link_frequency_hopping(void)
{
    s_rf_link.hopping_counter++;
    if (s_rf_link.hopping_counter >=10)
    {
        s_rf_link.hopping_counter = 0;
    }
    rf_set_channel(frequency_channal_tab[s_rf_link.hopping_counter]);
}

//ʧ��״̬�жϣ�����������λ
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
//����ͨѶ����
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
        //����״̬���
        rf_relink_detect();
        if (s_rf_link.link_status == DISCONNECCTING)
        {
            //ʧ��������Ƶ��������Ƶ��ͼ��һ��Ƶ��
            s_rf_link.hopping_counter = 0;
            //����    ������RFͨѶͨ��
            rf_set_channel(RF_RELINK_CH);
        }
        else
        {
            //����ͨѶ��Ƶ
            rf_link_frequency_hopping();
//            RF_SetChannel(RF_NORMAL_CH);            
        } 
        s_rf_link.link_step = RF_STEP_RX;
        
    }       
    //ʧ��ʱһֱ���ڽ���״̬���Ա����������
    else if ((s_rf_link.link_step == RF_STEP_RX) || (s_rf_link.link_status == DISCONNECCTING))  
    {        
        unsigned char rev_status = rf_get_statue(); 
			
        if (rev_status & RX_DR_FLAG)
        {
            receive_couter++;
            //���ճɹ�����ȡ����
            rf_read_multiple(R_RX_PAYLOAD,s_rf_link.rx_buffer, PAYLOAD_WIDTH);                                   
            s_rf_link.loss_link_counter = 0;  
            //����RF����
            rf_remote_data_updata();
            if(s_rf_link.link_status == CONNECCTING)
            {
                //ң�������ܴ��� 
                //һ����ɽ��䡢���������ܡ����չ��ܡ�¼���ܻ�ȡ����ת�߼�������Ϊ1���ٰ�Ϊ0
                by_remote_function_status_now ^= (by_remote_function_status_backup & 0x1e) ^ (remote_packet.function_status & 0x1e);
                //��ͣ���ܣ�����Ϊ1������Ϊ0, ģʽ���ܣ�����Ϊ0������Ϊ1
                by_remote_function_status_now &= ~0x60;
                by_remote_function_status_now |= remote_packet.function_status & 0x60;
            }
            by_remote_function_status_backup = remote_packet.function_status;
            s_rf_link.link_status = CONNECCTING;
            s_rf_link.link_step = RF_STEP_NULL;            
            //���ճɹ���ʱ��ͬ��
            s_rf_link.link_timer_counter = 3;             
            rf_link_packet();
            rf_tx_transmintdata(s_rf_link.tx_buffer,PAYLOAD_WIDTH); 
        }
    }
    //ʧ��ʱ��δ���ճɹ��Ͳ����˷���״̬
    else if ((s_rf_link.link_step == RF_STEP_TX) && (s_rf_link.link_status == CONNECCTING)) 
    {  
        rf_clear_status();
        rf_clear_fifo ();   
        s_rf_link.link_step = RF_STEP_NULL;
    }  
}
#if 0
//RF �µ�ַ����ADCֵ��timer0��ֵ��ͬ����
uint16_t rf_new_address_generate(void)
{
    uint16_t u16_temp;
    u16_temp = ADC_RDATA(ADC1);
    u16_temp ^= (uint16_t)(SysTick->VAL);
    return u16_temp;
}

//RF ����
void rf_binding(void)
{    
    uint8_t u8_binding_step;
    uint8_t u8_binding_success_flag;    
    uint8_t u8_temp,exit_binding_counter;
    uint16_t u16_temp;
    
    uint32_t binding_timer = 0;
    u8_binding_step = 0;
    u8_binding_success_flag = false;    
    
    //�ж��Ƿ���Ҫ���룬�����򷵻�
    if (s_rf_link.binding_status == STATUS_OFF)
        return;
    //����ǰ������ID
    u16_temp = rf_new_address_generate(); 
    //����ʱʹ��Ĭ��ID�������ص�Ƶ��
    rf_write_address((uint8_t *)TX_ADDRESS_DEF);
    RF_SetChannel(RF_BINDONG_CH);
    //����ʱ���ͷ��书�ʣ����ƶ������
    //rf_set_power(RF0dBm);    
   
    //����ǰ������ID
    s_rf_link.rf_new_address[2] = (uint8_t)(u16_temp>>8 & 0x00ff);
    s_rf_link.rf_new_address[3] = (uint8_t)(u16_temp & 0x00ff); 
    //���ն˿�ʼ����Ϊ����ģʽ
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

            //30��δ����ɹ��˳�����״̬
            if (binding_timer >= 3000)
            {                
                break;                
            }
            //5��δ���ճɹ��˳�����״̬
            if ((0 == u8_binding_step) && (binding_timer >= 500))
            { 
                break;
            } 
            DelayMs(1);
        }
        switch (u8_binding_step)
        {                
            case    0:   
                //�ж��Ƿ���ճɹ�                   
                 u8_temp = ucRF_GetStatus(); 
                if (u8_temp & RX_DR_FLAG)		                                           
                {  
                    RF_ReadBuf(R_RX_PAYLOAD,s_rf_link.rx_buffer, PAYLOAD_WIDTH);            
//                    if (get_u8data_checksum(s_rf_link.rx_buffer ,PAYLOAD_WIDTH - 1) == s_rf_link.rx_buffer[PAYLOAD_WIDTH - 1])
//                    {
                        //������ȷ�������״̬�����ö������
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
                //�ж��Ƿ���ճɹ�                   
                 u8_temp = ucRF_GetStatus(); 
                if (u8_temp & RX_DR_FLAG)		                                           
                { 
                    exit_binding_counter = 0;
                    if (read_rf_receive_data(s_rf_link.rx_buffer,PAYLOAD_WIDTH) == true)
                    {
                        //У����ȷ��ת����һ����
                        u8_temp = 0;                                                
                        //�����µ�ID
                        s_rf_link.rf_new_address[0] = s_rf_link.rx_buffer[1];
                        s_rf_link.rf_new_address[1] = s_rf_link.rx_buffer[2];
                        s_rf_link.rf_new_address[2] = s_rf_link.rx_buffer[3];
                        s_rf_link.rf_new_address[3] = s_rf_link.rx_buffer[4];
                        s_rf_link.rf_new_address[4] = s_rf_link.rx_buffer[5];
                        
                        s_rf_link.remoter_version[0] = s_rf_link.rx_buffer[10];
                        s_rf_link.remoter_version[1] = s_rf_link.rx_buffer[11];
                        s_rf_link.remoter_version[2] = s_rf_link.rx_buffer[12];
                        //���ݴ��                                
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
                        //�ȴ�һ��ʱ�䣬ȷ������˽����ϲ������ݳɹ����ٳɹ��˳�����״̬                        
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

// ��ȡ8bitУ���
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
//��ȡRF �������ݲ�У��
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


































