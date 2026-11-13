#include "drv8833.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "drv8833";

#define SPEED_MODE LEDC_LOW_SPEED_MODE
#define DUTY_BITS  8
#define DUTY_MAX   ((1u << DUTY_BITS) - 1u) /* 255 = команда 100 %         */
#define DUTY_HIGH  (1u << DUTY_BITS)        /* 256 = вихід постійно в 1    */

/* Кожен мотор використовує два канали LEDC, по одному на вхід моста. */
typedef struct {
    ledc_channel_t in1;
    ledc_channel_t in2;
} motor_channels_t;

static const motor_channels_t s_motors[2] = {
    [DRV8833_MOTOR_A] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1},
    [DRV8833_MOTOR_B] = {LEDC_CHANNEL_2, LEDC_CHANNEL_3},
};

/* Задати заповнення одного каналу й одразу його застосувати. */
static esp_err_t set_duty(ledc_channel_t ch, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(SPEED_MODE, ch, duty), TAG, "set duty");
    return ledc_update_duty(SPEED_MODE, ch);
}

/* Прив'язати канал LEDC до GPIO, старт із заповненням 0 (вихід у 0). */
static esp_err_t channel_init(ledc_channel_t ch, int gpio)
{
    ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = SPEED_MODE,
        .channel = ch,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    return ledc_channel_config(&cfg);
}

esp_err_t drv8833_init(void)
{
    /* Тримаємо міст у сні, поки налаштовуються ШІМ-виводи. */
    gpio_config_t sleep_pin = {
        .pin_bit_mask = 1ULL << DRV8833_PIN_NSLEEP,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&sleep_pin), TAG, "nsleep");
    gpio_set_level(DRV8833_PIN_NSLEEP, 0);

    ledc_timer_config_t timer = {
        .speed_mode = SPEED_MODE,
        .duty_resolution = DUTY_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = DRV8833_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "timer");

    ESP_RETURN_ON_ERROR(channel_init(s_motors[DRV8833_MOTOR_A].in1, DRV8833_PIN_AIN1), TAG, "ain1");
    ESP_RETURN_ON_ERROR(channel_init(s_motors[DRV8833_MOTOR_A].in2, DRV8833_PIN_AIN2), TAG, "ain2");
    ESP_RETURN_ON_ERROR(channel_init(s_motors[DRV8833_MOTOR_B].in1, DRV8833_PIN_BIN1), TAG, "bin1");
    ESP_RETURN_ON_ERROR(channel_init(s_motors[DRV8833_MOTOR_B].in2, DRV8833_PIN_BIN2), TAG, "bin2");

    /* Усі входи в 0 -> будимо міст. */
    gpio_set_level(DRV8833_PIN_NSLEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "ready, PWM %d Hz", DRV8833_PWM_HZ);
    return ESP_OK;
}

esp_err_t drv8833_set_speed(drv8833_motor_t motor, int speed_pct)
{
    if (speed_pct > 100) {
        speed_pct = 100;
    } else if (speed_pct < -100) {
        speed_pct = -100;
    }

    const motor_channels_t *m = &s_motors[motor];
    const uint32_t duty = (uint32_t)abs(speed_pct) * DUTY_MAX / 100u;

    /* Один вхід отримує ШІМ, інший у 0. Обидва в 0 = вільний вибіг. */
    ESP_RETURN_ON_ERROR(set_duty(m->in1, speed_pct > 0 ? duty : 0), TAG, "in1");
    return set_duty(m->in2, speed_pct < 0 ? duty : 0);
}

esp_err_t drv8833_brake(drv8833_motor_t motor)
{
    const motor_channels_t *m = &s_motors[motor];
    ESP_RETURN_ON_ERROR(set_duty(m->in1, DUTY_HIGH), TAG, "brake in1");
    return set_duty(m->in2, DUTY_HIGH);
}

esp_err_t drv8833_stop_all(void)
{
    ESP_RETURN_ON_ERROR(drv8833_set_speed(DRV8833_MOTOR_A, 0), TAG, "stop A");
    return drv8833_set_speed(DRV8833_MOTOR_B, 0);
}
