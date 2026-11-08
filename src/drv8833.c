#include "drv8833.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "drv8833";

#define DRV8833_SPEED_MODE LEDC_LOW_SPEED_MODE
#define DRV8833_TIMER      LEDC_TIMER_0
#define DRV8833_DUTY_BITS  LEDC_TIMER_8_BIT
#define DRV8833_DUTY_MAX   ((1u << 8) - 1u)

typedef struct {
    ledc_channel_t in1;
    ledc_channel_t in2;
    int pin_in1;
    int pin_in2;
} drv8833_channel_t;

static const drv8833_channel_t s_motors[] = {
    [DRV8833_MOTOR_A] = {
        .in1 = LEDC_CHANNEL_0,
        .in2 = LEDC_CHANNEL_1,
        .pin_in1 = DRV8833_PIN_AIN1,
        .pin_in2 = DRV8833_PIN_AIN2,
    },
    [DRV8833_MOTOR_B] = {
        .in1 = LEDC_CHANNEL_2,
        .in2 = LEDC_CHANNEL_3,
        .pin_in1 = DRV8833_PIN_BIN1,
        .pin_in2 = DRV8833_PIN_BIN2,
    },
};

static bool s_ready;
static uint32_t s_pwm_hz = DRV8833_PWM_HZ;

static esp_err_t drv8833_set_duty(ledc_channel_t channel, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(DRV8833_SPEED_MODE, channel, duty), TAG, "set duty");
    return ledc_update_duty(DRV8833_SPEED_MODE, channel);
}

static esp_err_t drv8833_config_channel(ledc_channel_t channel, int gpio_num)
{
    ledc_channel_config_t cfg = {
        .gpio_num = gpio_num,
        .speed_mode = DRV8833_SPEED_MODE,
        .channel = channel,
        .timer_sel = DRV8833_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&cfg);
}

static esp_err_t drv8833_timer_init(void)
{
    const uint32_t candidates[] = {DRV8833_PWM_HZ, 10000, 5000, 1000};

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        ledc_timer_config_t timer = {
            .speed_mode = DRV8833_SPEED_MODE,
            .duty_resolution = DRV8833_DUTY_BITS,
            .timer_num = DRV8833_TIMER,
            .freq_hz = candidates[i],
            .clk_cfg = LEDC_AUTO_CLK,
        };

        if (ledc_timer_config(&timer) == ESP_OK) {
            s_pwm_hz = candidates[i];
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

esp_err_t drv8833_set_sleep(bool sleep)
{
#if DRV8833_PIN_NSLEEP < 0
    (void)sleep;
    return ESP_OK;
#else
    return gpio_set_level(DRV8833_PIN_NSLEEP, sleep ? 0 : 1);
#endif
}

esp_err_t drv8833_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    /*
     * Keep the bridge asleep and force all inputs low before PWM starts.
     * Otherwise floating AIN/BIN after reset can spin the motors briefly.
     */
    uint64_t in_mask = 0;
    for (size_t i = 0; i < sizeof(s_motors) / sizeof(s_motors[0]); i++) {
        in_mask |= 1ULL << s_motors[i].pin_in1;
        in_mask |= 1ULL << s_motors[i].pin_in2;
    }

#if DRV8833_PIN_NSLEEP >= 0
    gpio_config_t sleep_cfg = {
        .pin_bit_mask = 1ULL << DRV8833_PIN_NSLEEP,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&sleep_cfg), TAG, "nsleep gpio");
    ESP_RETURN_ON_ERROR(drv8833_set_sleep(true), TAG, "hold sleep");
#endif

    gpio_config_t in_cfg = {
        .pin_bit_mask = in_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&in_cfg), TAG, "inputs gpio");
    for (size_t i = 0; i < sizeof(s_motors) / sizeof(s_motors[0]); i++) {
        gpio_set_level(s_motors[i].pin_in1, 0);
        gpio_set_level(s_motors[i].pin_in2, 0);
    }

    ESP_RETURN_ON_ERROR(drv8833_timer_init(), TAG, "timer");

    for (size_t i = 0; i < sizeof(s_motors) / sizeof(s_motors[0]); i++) {
        ESP_RETURN_ON_ERROR(drv8833_config_channel(s_motors[i].in1, s_motors[i].pin_in1), TAG, "in1");
        ESP_RETURN_ON_ERROR(drv8833_config_channel(s_motors[i].in2, s_motors[i].pin_in2), TAG, "in2");
    }

    s_ready = true;
    ESP_RETURN_ON_ERROR(drv8833_stop_all(), TAG, "stop");

#if DRV8833_PIN_NSLEEP >= 0
    ESP_RETURN_ON_ERROR(drv8833_set_sleep(false), TAG, "wake");
    vTaskDelay(pdMS_TO_TICKS(10));
#endif

    ESP_LOGI(TAG, "ready @ %lu Hz", (unsigned long)s_pwm_hz);
    return ESP_OK;
}

esp_err_t drv8833_set_speed(drv8833_motor_t motor, int speed_pct)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "not init");
    ESP_RETURN_ON_FALSE(motor == DRV8833_MOTOR_A || motor == DRV8833_MOTOR_B,
                        ESP_ERR_INVALID_ARG, TAG, "motor");

    if (speed_pct > 100) {
        speed_pct = 100;
    } else if (speed_pct < -100) {
        speed_pct = -100;
    }

    const drv8833_channel_t *ch = &s_motors[motor];
    const uint32_t duty = (uint32_t)((abs(speed_pct) * (int)DRV8833_DUTY_MAX) / 100);

    if (speed_pct > 0) {
        ESP_RETURN_ON_ERROR(drv8833_set_duty(ch->in1, duty), TAG, "fwd in1");
        ESP_RETURN_ON_ERROR(drv8833_set_duty(ch->in2, 0), TAG, "fwd in2");
    } else if (speed_pct < 0) {
        ESP_RETURN_ON_ERROR(drv8833_set_duty(ch->in1, 0), TAG, "rev in1");
        ESP_RETURN_ON_ERROR(drv8833_set_duty(ch->in2, duty), TAG, "rev in2");
    } else {
        ESP_RETURN_ON_ERROR(drv8833_set_duty(ch->in1, 0), TAG, "stop in1");
        ESP_RETURN_ON_ERROR(drv8833_set_duty(ch->in2, 0), TAG, "stop in2");
    }

    return ESP_OK;
}

esp_err_t drv8833_stop(drv8833_motor_t motor)
{
    return drv8833_set_speed(motor, 0);
}

esp_err_t drv8833_stop_all(void)
{
    ESP_RETURN_ON_ERROR(drv8833_set_speed(DRV8833_MOTOR_A, 0), TAG, "stop A");
    return drv8833_set_speed(DRV8833_MOTOR_B, 0);
}
