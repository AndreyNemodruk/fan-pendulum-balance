#include "bmi160.h"
#include "drv8833.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gyro";

/*
 * Two opposing fans on the ruler, each spins only one way:
 *   Motor A — corrects lean when PD output > 0
 *   Motor B — corrects lean when PD output < 0
 * No reverse → no prop dead time / asymmetric reverse thrust.
 * If lean direction is wrong, swap A/B wiring or flip MOTOR_SIGN.
 */
#define MOTOR_SIGN          (-1.0f)
#define MOTOR_KP            4.95f
#define MOTOR_KD            1.20f
#define MOTOR_CMD_DEAD      2.0f
#define MOTOR_SPEED_MIN     70
#define MOTOR_SPEED_FULL    99

static int thrust_from_cmd(float abs_u)
{
    const float mag = (float)MOTOR_SPEED_MIN +
                      (abs_u / (float)MOTOR_SPEED_FULL) *
                          (float)(MOTOR_SPEED_FULL - MOTOR_SPEED_MIN);
    return (int)lroundf(mag);
}

static void hold_pitch_dual(float pitch_deg, float pitch_rate_dps, int *speed_a, int *speed_b)
{
    float u = MOTOR_SIGN * (MOTOR_KP * pitch_deg + MOTOR_KD * pitch_rate_dps);

    if (u > (float)MOTOR_SPEED_FULL) {
        u = (float)MOTOR_SPEED_FULL;
    } else if (u < -(float)MOTOR_SPEED_FULL) {
        u = -(float)MOTOR_SPEED_FULL;
    }

    *speed_a = 0;
    *speed_b = 0;

    if (fabsf(u) < MOTOR_CMD_DEAD) {
        return;
    }

    const int thrust = thrust_from_cmd(fabsf(u));
    if (u > 0.0f) {
        *speed_a = thrust;   /* only forward on A */
    } else {
        *speed_b = thrust;   /* only forward on B */
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(drv8833_init());
    ESP_ERROR_CHECK(drv8833_stop_all());

    bmi160_t *imu = NULL;
    while (bmi160_init(&imu) != ESP_OK) {
        ESP_LOGE(TAG, "BMI160 init failed, retry in 2 s");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    int log_div = 0;

    while (true) {
        bmi160_sample_t sample;
        if (bmi160_read_sample(imu, &sample) == ESP_OK) {
            int speed_a = 0;
            int speed_b = 0;
            hold_pitch_dual(sample.pitch_deg, sample.gyr_dps[1], &speed_a, &speed_b);

            drv8833_set_speed(DRV8833_MOTOR_A, speed_a);
            drv8833_set_speed(DRV8833_MOTOR_B, speed_b);

            if (++log_div >= 20) {
                log_div = 0;
                ESP_LOGI(TAG, "pitch %+6.1f rate %+6.1f -> A %d%%  B %d%%",
                         sample.pitch_deg, sample.gyr_dps[1], speed_a, speed_b);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
