#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * DRV8833 dual H-bridge.
 * BMI160 uses GPIO 4/5/6/7. GPIO8 is RGB LED on DevKitM-1 — do not use for sleep.
 *
 *   GPIO1  -> AIN1   GPIO2 -> AIN2
 *   GPIO3  -> BIN1   GPIO10 -> BIN2
 *   GPIO0  -> SLP / nSLEEP (or tie SLP to 3V3 and set DRV8833_PIN_NSLEEP to -1)
 *   VM / VCC / GND -> motor supply / 3V3 / common ground
 */
#define DRV8833_PIN_AIN1   1
#define DRV8833_PIN_AIN2   2
#define DRV8833_PIN_BIN1   3
#define DRV8833_PIN_BIN2   10
#define DRV8833_PIN_NSLEEP 0

#define DRV8833_PWM_HZ 20000

typedef enum {
    DRV8833_MOTOR_A = 0,
    DRV8833_MOTOR_B = 1,
} drv8833_motor_t;

esp_err_t drv8833_init(void);

/* Signed duty: -100..100. Positive = IN1 PWM / IN2 low, negative = reverse. */
esp_err_t drv8833_set_speed(drv8833_motor_t motor, int speed_pct);

esp_err_t drv8833_stop(drv8833_motor_t motor);
esp_err_t drv8833_stop_all(void);
esp_err_t drv8833_set_sleep(bool sleep);
