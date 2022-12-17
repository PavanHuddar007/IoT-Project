#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define SAMPLE_CNT 32
static const adc1_channel_t adc_channel = ADC_CHANNEL_4;
#define LEDC_GPIO 27
static ledc_channel_config_t ledc_channel;
#define Motor_F 18
#define Motor_B 19

void motor_drive()
{
    gpio_pad_select_gpio(Motor_B);
    gpio_pad_select_gpio(Motor_F);
    gpio_set_direction(Motor_B, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(Motor_F, GPIO_MODE_OUTPUT);

    // vTaskDelay(500/portTICK_PERIOD_MS);
    gpio_set_level(Motor_F, 1);
    // vTaskDelay(50/portTICK_PERIOD_MS);
    gpio_set_level(Motor_B, 0);
}

static void init_hw(void)
{
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = LEDC_GPIO;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);
}