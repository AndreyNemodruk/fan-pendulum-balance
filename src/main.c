/*
 * Маятник, що балансує вентиляторами.
 *
 * Лінійка на осі несе два зустрічні вентилятори. BMI160 вимірює нахил,
 * PID-регулятор вирішує, наскільки сильно штовхати і який вентилятор вмикати:
 *   мотор A повертає маятник, коли кут додатний,
 *   мотор B повертає маятник, коли кут від'ємний.
 *
 * Кожні 10 мс:  читання IMU -> оцінка кута -> PID -> вентилятори.
 */

#include "bmi160.h"
#include "drv8833.h"

#include <math.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pendulum";

/* ---------------- PID ---------------- */

/*
 * u = KP * err + KI * інтеграл(err) + KD * rate
 * D-складова береться прямо з гіроскопа (похідна за вимірюванням),
 * тому не потребує диференціювання кута і не стрибає при зміні цілі.
 */
#define TARGET_ANGLE_DEG -1.0f /* кут, який треба утримувати                  */
#define KP 10.0f               /* % на градус                                 */
#define KI 0.0f                /* % на (градус * с), вмикати останнім         */
#define KD 2.0f                /* % на (градус / с)                           */
#define I_LIMIT 20.0f          /* обмеження інтеграла, градус * с             */

/* ---------------- Вентилятори ---------------- */

/*
 * MIN_PWM_x: ШІМ, з якого вентилятор починає давати реальну тягу (мертва зона).
 * GAIN_x:    масштаб тяги; слабшому вентилятору GAIN > 1, щоб при однаковому u
 *            обидва штовхали однаково.
 * ШІМ вентилятора = MIN_PWM_x + |u| * GAIN_x.
 * U_DEADBAND: тиха зона - при |u| менше цього обидва вентилятори вимкнені.
 */
#define MIN_PWM_A 18.0f /* %                                           */
#define MIN_PWM_B 18.0f /* %                                           */
#define GAIN_A 1.0f
#define GAIN_B 1.2f
/*
 * Напрямок обертання кожного мотора: 1 - як є, -1 - інвертувати
 * (те саме, що поміняти місцями два дроти мотора на DRV8833).
 */
#define DIR_A 1
#define DIR_B -1
#define U_DEADBAND 2.5f      /* %                                           */
#define BRAKE_MS 25          /* гальмувати вентилятор після вимкнення, мс    */
#define FALL_ANGLE_DEG 60.0f /* далі цього - маятник упав, вентилятори off   */

/* ---------------- Оцінка кута ---------------- */

/*
 * Кут з акселерометра atan2(ax, az) - 180 - це МІНУС поворот навколо +Y,
 * а gyr_dps[1] - це ПЛЮС той самий поворот, тому гіроскоп береться зі знаком -1.
 * З неправильним знаком гіроскоп бореться з акселерометром, і D-складова
 * штовхає замість того, щоб гасити рух.
 */
#define GYRO_SIGN (-1.0f)
#define GYRO_CAL_SAMPLES 200 /* калібрування зміщення на старті, ~2 с - не рухати   */
#define ALPHA 0.99f          /* комплементарний фільтр: вага гіроскопа             */
#define ACC_TRUST_G 0.15f    /* довіряти акселерометру, лише коли |a| = 1 g ± це   */

#define LOOP_PERIOD_MS 10
#define LOG_EVERY 20 /* рядок логу кожні 20 циклів = 200 мс */

/* Стан оцінювача кута. */
typedef struct
{
    float gyro_bias; /* зміщення гіроскопа Y, виміряне на старті, °/с */
    float angle;     /* оцінений кут, градуси                     */
    float rate;      /* кутова швидкість, °/с                     */
    float acc_angle; /* кут лише з акселерометра, градуси         */
    float acc_norm;  /* |a| у g, ~1.0, коли немає трясіння        */
} attitude_t;

/* Обмежити v діапазоном [lo, hi]. */
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Нахил лише за гравітацією: 0 - вертикально, додатний / від'ємний - в боки. */
static float acc_angle_deg(const bmi160_sample_t *s)
{
    float a = atan2f(s->acc_g[0], s->acc_g[2]) * 180.0f / (float)M_PI;
    return a + ((a >= 0.0f) ? -180.0f : 180.0f);
}

/* Читати IMU, доки не відповість (тільки під час старту). */
static void read_sample_blocking(bmi160_sample_t *s)
{
    while (bmi160_read_sample(s) != ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*
 * Виміряти зміщення гіроскопа по Y (маятник має бути нерухомим) і задати
 * початковий кут за акселерометром.
 */
static void attitude_init(attitude_t *att)
{
    bmi160_sample_t s;
    float sum = 0.0f;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        read_sample_blocking(&s);
        sum += s.gyr_dps[1];
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    att->gyro_bias = sum / GYRO_CAL_SAMPLES;
    att->angle = acc_angle_deg(&s);
    att->rate = 0.0f;
    ESP_LOGI(TAG, "gyro Y bias %+.2f dps, start angle %+.1f deg", att->gyro_bias, att->angle);
}

/*
 * Комплементарний фільтр. Гіроскоп інтегрується для швидких рухів;
 * акселерометр повільно підтягує кут назад, прибираючи дрейф гіроскопа,
 * але лише тоді, коли показує ~1 g (немає сильної вібрації чи ривка).
 */
static void attitude_update(attitude_t *att, const bmi160_sample_t *s, float dt)
{
    const float *a = s->acc_g;
    att->acc_norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    att->acc_angle = acc_angle_deg(s);

    att->rate = GYRO_SIGN * (s->gyr_dps[1] - att->gyro_bias);
    att->angle += att->rate * dt;

    if (fabsf(att->acc_norm - 1.0f) < ACC_TRUST_G)
    {
        att->angle = ALPHA * att->angle + (1.0f - ALPHA) * att->acc_angle;
    }
}

/*
 * Класичний PID. Вихід u у %: знак = який вентилятор, модуль = наскільки сильно.
 * Інтеграл обмежено ±I_LIMIT, щоб він не накопичувався без кінця (anti-windup).
 */
static float pid(float err, float rate, float dt, float *integ)
{
    *integ = clampf(*integ + err * dt, -I_LIMIT, I_LIMIT);
    return KP * err + KI * (*integ) + KD * rate;
}

/*
 * Перетворити u на ШІМ для двох вентиляторів (0..100 %).
 * Вентилятор обирається за знаком u, а не кута, тож D-складова може
 * гальмувати розгойдування ще до того, як маятник перетне вертикаль.
 * |u| масштабується GAIN і до нього додається MIN_PWM, щоб перестрибнути
 * мертву зону вентилятора.
 */
static void fan_mix(float u, int pwm[2])
{
    pwm[0] = 0;
    pwm[1] = 0;

    if (u > U_DEADBAND)
    {
        pwm[0] = (int)lroundf(MIN_PWM_A + u * GAIN_A);
    }
    else if (u < -U_DEADBAND)
    {
        pwm[1] = (int)lroundf(MIN_PWM_B - u * GAIN_B);
    }
}

/*
 * Подати ШІМ на вентилятори (драйвер сам обмежує 100 %). Щойно вимкнений
 * вентилятор коротко гальмується протягом BRAKE_MS - пропелер на вибігу
 * продовжує штовхати. braking[] повідомляє, який вентилятор гальмує (для логу).
 */
static void fans_apply(const int pwm[2], int64_t now_us, bool braking[2])
{
    static int prev_pwm[2];
    static int64_t brake_until_us[2];
    const drv8833_motor_t hw[2] = {DRV8833_MOTOR_A, DRV8833_MOTOR_B};
    const int dir[2] = {DIR_A, DIR_B};

    for (int m = 0; m < 2; m++)
    {
        if (pwm[m] != 0)
        {
            brake_until_us[m] = 0;
        }
        else if (prev_pwm[m] > 0)
        {
            brake_until_us[m] = now_us + (int64_t)BRAKE_MS * 1000;
        }
        prev_pwm[m] = pwm[m];

        braking[m] = (pwm[m] == 0) && (now_us < brake_until_us[m]);
        if (braking[m])
        {
            ESP_ERROR_CHECK(drv8833_brake(hw[m]));
        }
        else
        {
            ESP_ERROR_CHECK(drv8833_set_speed(hw[m], dir[m] * pwm[m]));
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(drv8833_init());

    while (bmi160_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "BMI160 init failed, retry in 2 s");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    attitude_t att;
    attitude_init(&att);

    float integ = 0.0f;
    int log_div = 0;
    int64_t last_us = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));

        bmi160_sample_t s;
        if (bmi160_read_sample(&s) != ESP_OK)
        {
            continue;
        }

        /* Реальний час від попереднього вимірювання; якщо дивний - номінальний період. */
        const int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_us) * 1e-6f;
        last_us = now;
        if (dt <= 0.0f || dt > 0.1f)
        {
            dt = LOOP_PERIOD_MS * 1e-3f;
        }

        attitude_update(&att, &s, dt);
        const float err = att.angle - TARGET_ANGLE_DEG;

        float u = 0.0f;
        if (fabsf(err) > FALL_ANGLE_DEG)
        {
            integ = 0.0f; /* маятник упав: вентилятори вимкнені, інтеграл скидаємо */
        }
        else
        {
            u = pid(err, att.rate, dt, &integ);
        }

        int pwm[2];
        bool braking[2];
        fan_mix(u, pwm);
        fans_apply(pwm, now, braking);

        if (++log_div >= LOG_EVERY)
        {
            log_div = 0;
            ESP_LOGI(TAG, "ang %+6.1f acc %+6.1f g %.2f rate %+7.1f | u %+6.1f I %+5.1f | A %3d%s B %3d%s",
                     att.angle, att.acc_angle, att.acc_norm, att.rate, u, integ,
                     pwm[0], braking[0] ? "br" : "", pwm[1], braking[1] ? "br" : "");
        }
    }
}
