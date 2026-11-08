#pragma once

#include <stdint.h>

#include "esp_err.h"

#define BMI160_PIN_SCLK 4
#define BMI160_PIN_MOSI 6
#define BMI160_PIN_MISO 5
#define BMI160_PIN_CS   7

typedef struct bmi160_dev bmi160_t;

typedef struct {
    float acc_g[3];
    float gyr_dps[3];
    float tilt_deg;
    float pitch_deg;
    float roll_deg;
    float temp_c;
} bmi160_sample_t;

esp_err_t bmi160_init(bmi160_t **dev);
esp_err_t bmi160_read_sample(bmi160_t *dev, bmi160_sample_t *sample);
