/* PWM Output using ADC input from potentiometer 
* PWM Output: GPIO8
* ADC Input: GPIO7 (ADC1_CH6)
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
static const char* TAG = "TEST";

#include "driver/ledc.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL LEDC_CHANNEL_0

#define LED_PIN GPIO_NUM_38

#define DOWN_PIN GPIO_NUM_1
#define UP_PIN GPIO_NUM_2


TaskHandle_t LEDTaskHandle = NULL;

#define STEP (65536)/10 // rounding errors => 6553 is inc, so 65530 is max, will be irrelevant during 8-bit mapping.
#define THRUSTIDXMAX 10
#define THRUSTIDXMIN 0

#define THRUSTMAP_MAX 41000
#define MOTORS_PWM_BITS 8

static uint16_t motorsThrustMap(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);


// @@ This function is for mapping the Thrust command (16 bit) to a Duty Cycle (8 bit, only 128-256)
static uint16_t motorsConv16ToBits(uint16_t bits)
{
    bits = motorsThrustMap(bits); // @@

    // @@ This function has been modified to map 0-65535 (16 bit) to 128-256 (8 bit) for MOTORS_PWM_BITS=8 (will work for any MOTORS_PWM_BITS <= 17)
    // return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1)); // original crazyflie line
    return (bits >> (16 - MOTORS_PWM_BITS + 1)) + (1 << (MOTORS_PWM_BITS-1)); // @@ explanation: bitshift down 9 bits (maps to 0-127), then add 128 (maps to 127-255) 
}

// @@ Custom Function - Linearly remap the thrust to limit power
// @@ ie.     T ---->[THRUST Map]---T'---->[motorsConv16ToBits]----DutyCycle--->Motors
// @@ where T = Thrust Input (0-65535); T' = Mapped Thrust (0-THRUSTMAP_MAX); DutyCycle = 128-256 (but upper range of DutyCycle is limited by THRUSTMAP_MAX)
// @@ (although this function is technically a part of motorsConv16ToBits so only motorsConv16ToBits needs to be called)
static uint16_t motorsThrustMap(uint16_t bits)
{   
    uint32_t mappedThrust = ((uint32_t)bits * THRUSTMAP_MAX) / 65535u;

    return (uint16_t)mappedThrust;
}



void LEDTask(void *arg) {
    uint8_t thrustidx = 0;
    uint16_t thrust = 0;
    uint16_t pwm_duty = 0;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << UP_PIN) | (1ULL << DOWN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = MOTORS_PWM_BITS,
        .freq_hz          = 500,  // Frequency in Hertz. Set frequency at 5 kHz
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

    while(1) {
        // If Up low & thrust < max
        if ((gpio_get_level(UP_PIN) == 0) && (thrustidx < THRUSTIDXMAX)) {
            // Add to thrust idx
            thrustidx++;

            // Calculate Thrust
            uint32_t temp = thrustidx * STEP - 1;
            thrust = temp;

            // compute DC from using mapping functions
            pwm_duty = motorsConv16ToBits(thrust);

            // tell user
            ESP_LOGI(TAG, "Thrust: %d (%dpc)\t Duty Cycle (raw): %d\t Duty Cycle (ms): %dms", thrust,10*thrustidx, pwm_duty, (uint16_t) (1000000 * pwm_duty / (256 * ledc_timer.freq_hz)));

        } 
        else if ((gpio_get_level(DOWN_PIN) == 0) && (thrustidx > THRUSTIDXMIN)) {
            // Add to thrust idx
            thrustidx--;

            // Calculate Thrust
            thrust = thrustidx*STEP;

            // compute DC from using mapping functions
            pwm_duty = motorsConv16ToBits(thrust);

            // tell user
            ESP_LOGI(TAG, "Thrust: %d (%dpc)\t Duty Cycle (raw): %d\t Duty Cycle (ms): %dms", thrust,10*thrustidx, pwm_duty, (uint16_t) (1000000*pwm_duty / (256 * ledc_timer.freq_hz)));
        };

        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pwm_duty);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);

        //ESP_LOGI("LEDTask", "UP=%d, DOWN=%d", gpio_get_level(UP_PIN), gpio_get_level(DOWN_PIN));


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



void app_main(void)
{

    xTaskCreatePinnedToCore(LEDTask, "LEDTask", 4096, NULL, 5, &LEDTaskHandle, 0);
}