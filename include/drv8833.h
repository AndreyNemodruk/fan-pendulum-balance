#pragma once

/*
 * Мінімальний драйвер подвійного H-моста DRV8833 (ШІМ через LEDC).
 *
 *   GPIO1 -> AIN1    GPIO2  -> AIN2
 *   GPIO3 -> BIN1    GPIO10 -> BIN2
 *   GPIO0 -> nSLEEP
 *   VM / VCC / GND -> живлення моторів / 3V3 / спільна земля
 *
 * BMI160 займає GPIO 4..7. GPIO8 - RGB-світлодіод на DevKitM-1.
 */

#include "esp_err.h"

#define DRV8833_PIN_AIN1   1
#define DRV8833_PIN_AIN2   2
#define DRV8833_PIN_BIN1   3
#define DRV8833_PIN_BIN2   10
#define DRV8833_PIN_NSLEEP 0

#define DRV8833_PWM_HZ 20000 /* вище за поріг чутності */

typedef enum {
    DRV8833_MOTOR_A = 0,
    DRV8833_MOTOR_B = 1,
} drv8833_motor_t;

/* Налаштувати ШІМ на всіх чотирьох входах (мотори стоять) і розбудити міст. */
esp_err_t drv8833_init(void);

/* Швидкість -100..100 %. Додатна: ШІМ на IN1, IN2 = 0. Від'ємна: реверс. 0: вільний вибіг. */
esp_err_t drv8833_set_speed(drv8833_motor_t motor, int speed_pct);

/* Коротке гальмування: IN1 = IN2 = 1, виводи мотора замкнені -> швидка зупинка. */
esp_err_t drv8833_brake(drv8833_motor_t motor);

/* Обидва мотори на 0 % (вільний вибіг). */
esp_err_t drv8833_stop_all(void);
