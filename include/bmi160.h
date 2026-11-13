#pragma once

/*
 * Мінімальний драйвер BMI160 (SPI, один датчик).
 * Акселерометр ±8 g, гіроскоп ±2000 °/с, обидва 400 Гц.
 */

#include "esp_err.h"

#define BMI160_PIN_SCLK 4
#define BMI160_PIN_MOSI 6
#define BMI160_PIN_MISO 5
#define BMI160_PIN_CS   7

/* Одне вимірювання, вже у фізичних одиницях і в осях плати. */
typedef struct {
    float acc_g[3];   /* прискорення X, Y, Z, g          */
    float gyr_dps[3]; /* кутова швидкість X, Y, Z, °/с   */
} bmi160_sample_t;

/* Налаштувати SPI, скинути датчик, увімкнути й сконфігурувати акселерометр і гіроскоп. */
esp_err_t bmi160_init(void);

/* Прочитати гіроскоп і акселерометр однією SPI-транзакцією та перевести в g і °/с. */
esp_err_t bmi160_read_sample(bmi160_sample_t *sample);
