/* PWM Output using ADC input from potentiometer 
* PWM Output: GPIO8
* ADC Input: GPIO7 (ADC1_CH6)
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_cali.h"

#define ADC_CHANNEL ADC_CHANNEL_6     // GPIO7 if ADC1, GPIO17 if ADC2
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_UNIT ADC_UNIT_1

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0

#define LED_PIN GPIO_NUM_8


TaskHandle_t ADCTaskHandle = NULL;
TaskHandle_t LEDTaskHandle = NULL;

QueueHandle_t adc_queue;

void ADCTask(void *arg) {
    int potentiometer_read, potentiometer_output, old_output;

    adc_oneshot_unit_handle_t handle = NULL;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    }; 

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CHANNEL, &config));

    /*          CALIBRATION             */

    adc_cali_handle_t adc1_cali_handle = NULL;

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));

    /*          READ RAW ADC INPUT AND CONVERT TO  mV               */
    old_output = 0;
    while (1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(handle, ADC_CHANNEL, &potentiometer_read));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, potentiometer_read, &potentiometer_output));
        if (potentiometer_output != old_output) {
                old_output = potentiometer_output;
                xQueueGenericSend(adc_queue, &potentiometer_read, portMAX_DELAY, queueSEND_TO_BACK);
                vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
}

void LEDTask(void *arg) {
    int pwm_duty = 0;
    int read_duty;

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_12_BIT,
        .freq_hz          = 12000,  // Frequency in Hertz. Set frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LED_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // start the led?

    while(1) {
        xQueueReceive(adc_queue, &pwm_duty, portMAX_DELAY);

        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pwm_duty);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        read_duty = ledc_get_duty(ledc_channel.speed_mode, ledc_channel.channel);

        printf("Duty: %d\n", read_duty);
    }
}



void app_main(void)
{
    adc_queue = xQueueCreate(10, sizeof(int));

    xTaskCreatePinnedToCore(ADCTask, "ADCTask", 4096, NULL, 5, &ADCTaskHandle, 0);
    xTaskCreatePinnedToCore(LEDTask, "LEDTask", 4096, NULL, 5, &LEDTaskHandle, 0);
}