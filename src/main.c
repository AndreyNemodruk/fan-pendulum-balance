#include "bmi160.h"
#include "drv8833.h"

#include <math.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "gyro";

/*
 * Opposed fans: motor A (forward) pushes to reduce positive angle,
 * motor B (forward) pushes to reduce negative angle.
 * The motor is chosen by the sign of the PID output u (not by the angle),
 * so the D-term can brake before the pendulum crosses vertical.
 * The other motor may help by spinning in REVERSE (thrust in the same
 * direction as the active one) - scaled by REV_GAIN, 0 = keep it stopped.
 */
/*
 * Per-motor calibration (A = logical motor that reduces positive angle).
 * MIN_PWM_x: PWM where this fan starts to give real thrust (dead zone).
 * GAIN_x:    thrust scale - a weaker fan gets GAIN > 1, a stronger one < 1.
 * Resulting PWM = MIN_PWM_x + |u| * GAIN_x, limited to MAX_PWM.
 */
#define MIN_PWM_A 18.0f /* %                                             */
#define MIN_PWM_B 18.0f /* %                                             */
#define GAIN_A 1.0f
#define GAIN_B 1.0f
#define MAX_PWM 100.0f
#define U_DEADBAND 0.5f /* % - below this both motors stopped          */
#define REV_GAIN 0.0f   /* 0..1 - reverse assist of the other motor    */

#define KP 1.0f  /* % per degree                                */
#define KI 0.1f  /* % per (degree * s)  - enable last           */
#define KD 1.35f /* % per (degree / s)  - from gyro             */
/*
 * Boost near vertical: extra term NEAR_BOOST_PCT * sign(e) * sqrt(|e|/ZONE).
 * sqrt is very steep at 0, so even 0.5-1 degree gives strong thrust;
 * at the zone edge the boost reaches NEAR_BOOST_PCT and stays there.
 * KD is raised smoothly towards KD_NEAR as the angle approaches 0
 * to damp the stronger response.
 */
#define NEAR_ZONE_DEG 2.0f   /* degrees                                     */
#define NEAR_BOOST_PCT 35.0f /* % added at the zone edge                    */
#define KD_NEAR 0.25f        /* % per (degree / s) at 0 degrees             */
/*
 * Fan lag compensation.
 * T_LEAD: control uses the predicted angle (angle + rate * T_LEAD), so the
 *         motors switch BEFORE the pendulum reaches 0.
 * BRAKE_MS: when a motor is switched off it is actively braked for this
 *         time instead of coasting (a coasting prop keeps pushing).
 * BRAKE_REV_PWM: 0 = short brake (IN1=IN2=1); >0 = reverse pulse of this %,
 *         stops the prop faster.
 */
#define T_LEAD 0.15f    /* s                                           */
#define BRAKE_MS 50     /* ms                                          */
#define BRAKE_REV_PWM 0 /* %                                           */
#define I_LIMIT 20.0f   /* integral clamp (degree * s)                 */

#define keepAngle 0.0f
#define MOTOR_POLARITY (1) /* prop spin direction                         */
#define SWAP_MOTORS 0      /* 1 if motor A pushes the wrong way           */
/*
 * acc angle = atan2(ax, az) - 180 equals MINUS the rotation about +Y,
 * while gyr_dps[1] is PLUS that rotation -> sign must be -1.
 * With +1 the gyro fights the accelerometer: the estimated angle lags
 * and at fast motion even moves the wrong way, and the D-term pushes
 * instead of damping. Check in log: "rate" and "accr" must have the same sign.
 */
#define GYRO_SIGN (-1.0f)
#define GYRO_CAL_SAMPLES 200 /* startup gyro bias calibration (~2 s still) */
#define ALPHA 0.99f          /* complementary filter: gyro weight           */
/*
 * Accelerometer is trusted only when it sees ~1 g (no strong vibration or
 * jerk). Otherwise the angle runs on the gyro alone for that sample.
 * ACC_LPF: software low-pass on ax/az before atan2 (0..1, smaller = smoother).
 */
#define ACC_TRUST_G 0.15f
#define ACC_LPF 0.2f

#define FALL_ANGLE_DEG 60.0f /* beyond this - fallen, stop motors           */
#define LOOP_PERIOD_MS 10

static float clampf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static float acc_angle_deg(const bmi160_sample_t *s)
{
    float a = atan2f(s->acc_g[0], s->acc_g[2]) * 180.0f / (float)M_PI;
    a += (a >= 0.0f) ? -180.0f : 180.0f;
    return a;
}

void app_main(void)
{
    ESP_ERROR_CHECK(drv8833_init());
    ESP_ERROR_CHECK(drv8833_stop_all());

    bmi160_t *imu = NULL;
    while (bmi160_init(&imu) != ESP_OK)
    {
        ESP_LOGE(TAG, "BMI160 init failed, retry in 2 s");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* Initial angle from accelerometer only. */
    bmi160_sample_t sample;
    while (bmi160_read_sample(imu, &sample) != ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    /* Gyro bias: keep the pendulum still for ~2 s after power-up. */
    float gyro_bias = 0.0f;
    int cal_n = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        if (bmi160_read_sample(imu, &sample) == ESP_OK)
        {
            gyro_bias += sample.gyr_dps[1];
            cal_n++;
        }
        vTaskDelay(1);
    }
    gyro_bias = (cal_n > 0) ? gyro_bias / (float)cal_n : 0.0f;
    ESP_LOGI(TAG, "gyro Y bias %+.2f dps (%d samples)", gyro_bias, cal_n);

    float angle = acc_angle_deg(&sample);
    float prev_acc = angle;
    float ax_f = sample.acc_g[0];
    float ay_f = sample.acc_g[1];
    float az_f = sample.acc_g[2];
    int acc_rej = 0; /* rejected accel samples since last log line */
    float acc_rate = 0.0f;
    float integ = 0.0f;
    int64_t last_us = esp_timer_get_time();
    int log_div = 0;
    int prev_cmd[2] = {0, 0};
    int64_t brake_until_us[2] = {0, 0};

    TickType_t wake = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(LOOP_PERIOD_MS) > 0 ? pdMS_TO_TICKS(LOOP_PERIOD_MS) : 1;

    while (true)
    {
        vTaskDelayUntil(&wake, period);

        if (bmi160_read_sample(imu, &sample) != ESP_OK)
        {
            continue;
        }

        const int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_us) * 1e-6f;
        last_us = now;
        if (dt <= 0.0f || dt > 0.1f)
        {
            dt = LOOP_PERIOD_MS * 1e-3f;
        }

        /* Complementary filter: gyro for fast motion, accel for drift. */
        /* Low-pass accel, check vector length, then angle. */
        ax_f += ACC_LPF * (sample.acc_g[0] - ax_f);
        ay_f += ACC_LPF * (sample.acc_g[1] - ay_f);
        az_f += ACC_LPF * (sample.acc_g[2] - az_f);
        const float acc_norm = sqrtf(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
        const bool acc_ok = fabsf(acc_norm - 1.0f) < ACC_TRUST_G;
        bmi160_sample_t filt = sample;
        filt.acc_g[0] = ax_f;
        filt.acc_g[1] = ay_f;
        filt.acc_g[2] = az_f;
        const float acc_angle = acc_angle_deg(&filt);
        const float rate = GYRO_SIGN * (sample.gyr_dps[1] - gyro_bias); /* deg/s */
        /* Diagnostic only: filtered derivative of acc angle, sign must match rate. */
        acc_rate = 0.8f * acc_rate + 0.2f * (acc_angle - prev_acc) / dt;
        prev_acc = acc_angle;
        angle += rate * dt;
        if (!acc_ok)
        {
            acc_rej++;
        }
        if (acc_ok)
        {
            angle = ALPHA * angle + (1.0f - ALPHA) * acc_angle;
        }

        const float err = angle - keepAngle;

        int pwm_a = 0;
        int pwm_b = 0;
        float u = 0.0f;

        if (fabsf(err) > FALL_ANGLE_DEG)
        {
            /* Fallen over - stop and reset integrator. */
            integ = 0.0f;
        }
        else
        {
            integ = clampf(integ + err * dt, -I_LIMIT, I_LIMIT);
            /* Predicted angle: act on where the pendulum will be. */
            const float pred = err + rate * T_LEAD;
            const float near_err = clampf(pred, -NEAR_ZONE_DEG, NEAR_ZONE_DEG);
            const float w = 1.0f - fabsf(near_err) / NEAR_ZONE_DEG; /* 1 at 0, 0 at edge */
            const float kd = KD + (KD_NEAR - KD) * w;
            const float boost = copysignf(
                NEAR_BOOST_PCT * sqrtf(fabsf(near_err) / NEAR_ZONE_DEG), near_err);
            u = KP * pred + boost + KI * integ + kd * rate;

            const float mag = fabsf(u);
            if (mag > U_DEADBAND)
            {
                /* Jump over each motor's dead zone, then scale by its gain. */
                const float fwd_a = clampf(MIN_PWM_A + mag * GAIN_A, MIN_PWM_A, MAX_PWM);
                const float fwd_b = clampf(MIN_PWM_B + mag * GAIN_B, MIN_PWM_B, MAX_PWM);
                const float rev_a = (REV_GAIN > 0.0f)
                                        ? clampf(fwd_a * REV_GAIN, 0.0f, MAX_PWM)
                                        : 0.0f;
                const float rev_b = (REV_GAIN > 0.0f)
                                        ? clampf(fwd_b * REV_GAIN, 0.0f, MAX_PWM)
                                        : 0.0f;
                if (u > 0.0f)
                {
                    pwm_a = (int)lroundf(fwd_a);  /* A forward */
                    pwm_b = -(int)lroundf(rev_b); /* B reverse, same push */
                }
                else
                {
                    pwm_b = (int)lroundf(fwd_b);  /* B forward */
                    pwm_a = -(int)lroundf(rev_a); /* A reverse, same push */
                }
            }
        }

        /*
         * Zero-crossing cut-off. A motor may push only in two cases:
         *  - it returns the pendulum to vertical (A: angle > 0, B: angle < 0)
         *  - it brakes the motion towards it   (A: rate > 0,  B: rate < 0)
         * So the motor that pushed the pendulum to 0 stops immediately when
         * 0 is passed, but the opposite one may start braking BEFORE 0.
         */
        if (pwm_a > 0 && err <= 0.0f && rate <= 0.0f)
        {
            pwm_a = 0;
        }
        if (pwm_b > 0 && err >= 0.0f && rate >= 0.0f)
        {
            pwm_b = 0;
        }

        /* Active brake of a motor that was pushing and is now off. */
        int cmd[2] = {pwm_a, pwm_b};
        bool braking[2] = {false, false};
        for (int m = 0; m < 2; m++)
        {
            if (cmd[m] != 0)
            {
                brake_until_us[m] = 0;
            }
            else if (prev_cmd[m] > 0)
            {
                brake_until_us[m] = now + (int64_t)BRAKE_MS * 1000;
            }
            braking[m] = (cmd[m] == 0) && (now < brake_until_us[m]);
            prev_cmd[m] = cmd[m];
        }

        for (int m = 0; m < 2; m++)
        {
#if SWAP_MOTORS
            const drv8833_motor_t hw = (m == 0) ? DRV8833_MOTOR_B : DRV8833_MOTOR_A;
#else
            const drv8833_motor_t hw = (m == 0) ? DRV8833_MOTOR_A : DRV8833_MOTOR_B;
#endif
            if (braking[m])
            {
                if (BRAKE_REV_PWM > 0)
                {
                    ESP_ERROR_CHECK(drv8833_set_speed(hw, -MOTOR_POLARITY * BRAKE_REV_PWM));
                }
                else
                {
                    ESP_ERROR_CHECK(drv8833_brake(hw));
                }
            }
            else
            {
                ESP_ERROR_CHECK(drv8833_set_speed(hw, MOTOR_POLARITY * cmd[m]));
            }
        }

        if (++log_div >= 20)
        {
            log_div = 0;
            ESP_LOGI(TAG, "ang %+6.1f acc %+6.1f g %.2f rej %2d rate %+7.1f accr %+7.1f | u %+6.1f I %+5.1f | A %+4d%s B %+4d%s",
                     angle, acc_angle, acc_norm, acc_rej, rate, acc_rate, u, integ,
                     pwm_a, braking[0] ? "br" : "", pwm_b, braking[1] ? "br" : "");
            acc_rej = 0;
        }
    }
}
