#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>
#include "led.h"

led_function_s m_led_func;

typedef struct
{
    uint32_t gpio_periph;
    uint32_t pin;
} led_t;

static led_t leds[LED_NUM] =
{
   {GPIOA, GPIO_PIN_8},
   {GPIOC, GPIO_PIN_9},
   {GPIOC, GPIO_PIN_1},
   {GPIOC, GPIO_PIN_0}
};

/* ����ĳ��LED��״̬ */
static void led_set(led_e led, bool value)
{
    //    if (led >= LED_NUM)
    //        return;    

    if (value)
        gpio_bit_set(leds[led].gpio_periph, leds[led].pin);
    else
        gpio_bit_reset(leds[led].gpio_periph, leds[led].pin);
}

/*�ر�����LED*/
static void led_clear_all(void)
{
    uint8_t i;
    for (i = 0; i < LED_NUM; i++)
    {
        led_set((led_e)i, 0);
    }
}

/*������LED*/
static void led_set_all(void)
{
    uint8_t i;
    for (i = 0; i < LED_NUM; i++)
    {
        led_set((led_e)i, 1);
    }
}

void led_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    //LED_L_H
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    //LED_L_T
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    //LED_R_H
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    //LED_R_T
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    led_clear_all();
    set_led_all_off();
}
//��������LEDΪ����
void set_led_all_on(void) { m_led_func.flicker_function = LED_ALL_ON; }
//��������LEDΪ����
void set_led_all_off(void) { m_led_func.flicker_function = LED_ALL_OFF; }
//����LED��Ϊʧ����˸
void set_led_lose_control_flicker(void) { m_led_func.flicker_function = LED_LOSE_CONTROL_FLICKER; }
//����LED��Ϊ�ش�У׼��һ����˸
void set_led_compass_calibrate_step1_flicker(void) { m_led_func.flicker_function = LED_COMPASS_CALIBRATE_STEP1_FLICKER; }
//����LED��Ϊ�ش�У׼�ڶ�����˸
void set_led_compass_calibrate_step2_flicker(void) { m_led_func.flicker_function = LED_COMPASS_CALIBRATE_STEP2_FLICKER; }
//����LED��ΪˮƽУ׼��˸
void set_led_level_calibrate_flicker(void) { m_led_func.flicker_function = LED_LEVEL_CALIBRATE_FLICKER; }
//����LED��Ϊ������˸
void set_led_binding_flicker(void) { m_led_func.flicker_function = LED_BINDING_FLICKER; }
//����LED��Ϊ�͵���˸
void set_led_low_voltage_flicker(void) { m_led_func.flicker_function = LED_LOW_VOLTAGE_FLICKER; }
//����LED��Ϊ��̬ģʽ��˸
void set_led_imu_mode_flicker(void) { m_led_func.flicker_function = LED_IMU_MODE_FLICKER; }
//����LED��Ϊ������˸
void set_led_all_fast_flicker(void) { m_led_func.flicker_function = LED_ALL_FAST_FLICKER; }
//����LED��Ϊ������˸
void set_led_all_slow_flicker(void) { m_led_func.flicker_function = LED_ALL_SLOW_FLICKER; }
//����LED��Ϊ��ͣ��˸
void set_led_scram_flicker(void) { m_led_func.flicker_function = LED_SCRAM_FLICKER; }

void led_all_on(void)
{
    led_set_all();
}

void led_all_off(void)
{
    led_clear_all();
}

//led  ʧ����˸���� �ĸ��ŵƳ���/����ѭ����ֱ�����䵽���� ����Ƶ��100hz
void led_lose_control_flicker(void)
{
    if ((m_led_func.timer % 200) == 0)
    {
        led_all_on();
    }
    else if ((m_led_func.timer % 200) == 10)
    {
        led_all_off();
    }
}

//led ��˸���� ����Ƶ��100hz 
void led_all_flicker(uint8_t by_flicker_freq)
{
    if ((m_led_func.timer % (50 / by_flicker_freq)) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_all_on();
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_all_off();
            m_led_func.flicker_on_off_flag = true;
        }
    }
}

//led ����˸���� ����Ƶ��100hz ��˸Ƶ��2Hz
static void led_all_flicker_fast(void)
{
    led_all_flicker(2);
}

//led ����˸���� ����Ƶ��100hz ��˸Ƶ��1Hz
static void led_all_flicker_slow(void)
{
    led_all_flicker(1);
}

//led �ĵƽ������˸���� ����Ƶ��100hz 
static void led_all_flicker_alternately(uint8_t by_flicker_freq)
{
    static uint8_t alternately_step = 0;
    if ((m_led_func.timer % (50 / by_flicker_freq)) == 0)
    {
        alternately_step++;
        if (alternately_step >= LED_NUM)
        {
            alternately_step = 0;
        }
        switch (alternately_step)
        {
        case    3:
            led_all_off();
            led_set(LED_L_H, 1);
            break;
        case    2:
            led_all_off();
            led_set(LED_L_T, 1);
            break;
        case    1:
            led_all_off();
            led_set(LED_R_T, 1);
            break;
        case    0:
            led_all_off();
            led_set(LED_R_H, 1);
            break;
        default:
            break;
        }
    }
}

//�ش�У׼��һ�� ���� ����ͷ�̵ƺͻ�β�������
static void led_compass_calibrate_step1_flicker(void)
{
    led_set(LED_R_H, 0);
    led_set(LED_R_T, 0);
    if ((m_led_func.timer % 50) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_set(LED_L_H, 1);
            led_set(LED_L_T, 1);
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_set(LED_L_H, 0);
            led_set(LED_L_T, 0);
            m_led_func.flicker_on_off_flag = true;
        }
    }
}
//�ش�У׼�ڶ��� ���� �Ҳ��ͷ�̵ƺͻ�β�������
static void led_compass_calibrate_step2_flicker(void)
{
    led_set(LED_L_H, 0);
    led_set(LED_L_T, 0);
    if ((m_led_func.timer % 50) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_set(LED_R_H, 1);
            led_set(LED_R_T, 1);
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_set(LED_R_H, 0);
            led_set(LED_R_T, 0);
            m_led_func.flicker_on_off_flag = true;
        }
    }
}
//��̬ģʽ���� ��̬ģʽ����ͷ�̵Ƴ�������β�����������
static void led_imu_mode_flicker(void)
{
    led_set(LED_L_H, 1);
    led_set(LED_R_H, 1);
    if ((m_led_func.timer % 50) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_set(LED_L_T, 1);
            led_set(LED_R_T, 1);
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_set(LED_L_T, 0);
            led_set(LED_R_T, 0);
            m_led_func.flicker_on_off_flag = true;
        }
    }

}

//led �ĵƽ������˸���� ����Ƶ��100hz ��˸Ƶ��2Hz
//static void led_all_flicker_alternately_fast(void)
//{        
//    led_all_flicker_alternately(2);
//}

//led �ĵƽ����������� ����Ƶ��100hz ��˸Ƶ��1Hz
static void led_all_flicker_alternately_slow(void)
{
    led_all_flicker_alternately(1);
}

//led ��˸���� ����Ƶ��100hz
void led_flicker(void)
{
    m_led_func.timer++;
    switch (m_led_func.flicker_function)
    {
    case    LED_ALL_ON:
        led_all_on();
        break;
    case    LED_ALL_OFF:
        led_all_off();
        break;
    case    LED_LOSE_CONTROL_FLICKER:
        led_lose_control_flicker();
        break;
    case    LED_COMPASS_CALIBRATE_STEP1_FLICKER:
        led_compass_calibrate_step1_flicker();
        break;
    case    LED_COMPASS_CALIBRATE_STEP2_FLICKER:
        led_compass_calibrate_step2_flicker();
        break;
    case    LED_ALL_SLOW_FLICKER:
    case    LED_LEVEL_CALIBRATE_FLICKER:
        led_all_flicker_slow();
        break;
    case    LED_ALL_FAST_FLICKER:
    case    LED_BINDING_FLICKER:
        led_all_flicker_fast();
        break;
    case    LED_LOW_VOLTAGE_FLICKER:
        led_all_flicker_slow();
        break;
    case    LED_IMU_MODE_FLICKER:
        led_imu_mode_flicker();
        break;
    case    LED_SCRAM_FLICKER:
        led_all_flicker_alternately_slow();
        break;
    default:
        break;
    }
}





