#include "bmi160.h"

#include <stdint.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmi160";

#define SPI_HZ (4 * 1000 * 1000)

/* Регістри */
#define REG_CHIP_ID   0x00
#define REG_PMU       0x03 /* стан режимів живлення                       */
#define REG_DATA      0x0C /* гіроскоп XYZ, потім акселерометр XYZ, 12 байт */
#define REG_ACC_CONF  0x40
#define REG_ACC_RANGE 0x41
#define REG_GYR_CONF  0x42
#define REG_GYR_RANGE 0x43
#define REG_CMD       0x7E
#define REG_SPI_TEST  0x7F /* будь-яке читання тут перемикає чип у SPI     */

#define CHIP_ID 0xD1

/* Команди */
#define CMD_ACC_NORMAL 0x11
#define CMD_GYR_NORMAL 0x15
#define CMD_SOFT_RESET 0xB6

/* Значення конфігурації */
#define ACC_CONF_400HZ_OSR4   0x0A /* ФНЧ ~40 Гц, гасить вібрацію пропелерів   */
#define ACC_RANGE_8G          0x08 /* на ±2 g вібрація вентиляторів обрізається */
#define GYR_CONF_400HZ_NORMAL 0x2A
#define GYR_RANGE_2000DPS     0x00

#define ACC_LSB_PER_G   4096.0f /* для ±8 g       */
#define GYR_LSB_PER_DPS 16.4f   /* для ±2000 °/с  */

#define PMU_ALL_NORMAL 0x14 /* акселерометр = normal (біти 5:4), гіроскоп = normal (біти 3:2) */

static spi_device_handle_t s_spi;

/* Прочитати `len` байт, починаючи з `reg`. Біт 7 адреси = читання. */
static esp_err_t reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    if (len == 0 || len >= sizeof(tx)) {
        return ESP_ERR_INVALID_SIZE;
    }
    tx[0] = reg | 0x80;

    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_spi, &t), TAG, "spi read");

    /* rx[0] приходить, поки передається адреса, - це не дані регістра. */
    memcpy(data, &rx[1], len);
    return ESP_OK;
}

/* Записати один регістр і зачекати 1 мс, щоб чип його застосував. */
static esp_err_t reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7F, value};
    spi_transaction_t t = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
    };
    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(s_spi, &t), TAG, "spi write");
    esp_rom_delay_us(1000);
    return ESP_OK;
}

/*
 * Після ввімкнення або скидання BMI160 слухає I2C. Фронт CS вгору
 * (кінець будь-якої SPI-передачі) перемикає його в SPI, тому: холосте
 * читання, потім CHIP_ID. Повторюємо, доки ID не збіжеться або не мине таймаут.
 */
static esp_err_t wait_chip_id(int timeout_ms)
{
    uint8_t id = 0;
    for (int waited = 0; waited < timeout_ms; waited += 10) {
        uint8_t dummy;
        ESP_RETURN_ON_ERROR(reg_read(REG_SPI_TEST, &dummy, 1), TAG, "spi switch");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(reg_read(REG_CHIP_ID, &id, 1), TAG, "chip id");
        if (id == CHIP_ID) {
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "CHIP_ID=0x%02X, expected 0x%02X", id, CHIP_ID);
    return ESP_ERR_NOT_FOUND;
}

/* Створити шину SPI2 і підключити до неї датчик (SPI режим 0). */
static esp_err_t spi_init(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = BMI160_PIN_MOSI,
        .miso_io_num = BMI160_PIN_MISO,
        .sclk_io_num = BMI160_PIN_SCLK,
        /* Невикористані лінії мають бути -1, інакше 0 означає GPIO0. */
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED), TAG, "bus");

    spi_device_interface_config_t dev = {
        .clock_speed_hz = SPI_HZ,
        .mode = 0,
        .spics_io_num = BMI160_PIN_CS,
        .queue_size = 1,
    };
    return spi_bus_add_device(SPI2_HOST, &dev, &s_spi);
}

esp_err_t bmi160_init(void)
{
    ESP_RETURN_ON_ERROR(spi_init(), TAG, "spi init");
    ESP_RETURN_ON_ERROR(wait_chip_id(100), TAG, "not found");

    /* Програмне скидання повертає чип в I2C, і деякий час він ігнорує CS. */
    ESP_RETURN_ON_ERROR(reg_write(REG_CMD, CMD_SOFT_RESET), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(wait_chip_id(1000), TAG, "id after reset");

    /* Спочатку вмикаємо живлення (акселерометру треба ~4 мс, гіроскопу ~80 мс), потім конфігуруємо. */
    ESP_RETURN_ON_ERROR(reg_write(REG_CMD, CMD_ACC_NORMAL), TAG, "acc on");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(reg_write(REG_CMD, CMD_GYR_NORMAL), TAG, "gyr on");
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t pmu = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_PMU, &pmu, 1), TAG, "pmu");
    if ((pmu & 0x3C) != PMU_ALL_NORMAL) {
        ESP_LOGE(TAG, "accel/gyro did not start, PMU=0x%02X", pmu);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(reg_write(REG_ACC_CONF, ACC_CONF_400HZ_OSR4), TAG, "acc conf");
    ESP_RETURN_ON_ERROR(reg_write(REG_ACC_RANGE, ACC_RANGE_8G), TAG, "acc range");
    ESP_RETURN_ON_ERROR(reg_write(REG_GYR_CONF, GYR_CONF_400HZ_NORMAL), TAG, "gyr conf");
    ESP_RETURN_ON_ERROR(reg_write(REG_GYR_RANGE, GYR_RANGE_2000DPS), TAG, "gyr range");

    ESP_LOGI(TAG, "ready: accel +-8 g, gyro +-2000 dps, 400 Hz");
    return ESP_OK;
}

/* Два байти little endian -> знакове 16-бітне число. */
static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

esp_err_t bmi160_read_sample(bmi160_sample_t *s)
{
    uint8_t raw[12];
    esp_err_t err = reg_read(REG_DATA, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    for (int i = 0; i < 3; i++) {
        s->gyr_dps[i] = (float)le16(&raw[i * 2]) / GYR_LSB_PER_DPS;
        s->acc_g[i] = (float)le16(&raw[6 + i * 2]) / ACC_LSB_PER_G;
    }

    /* Чип встановлено догори дном (180° навколо X): Y і Z змінюють знак. */
    s->acc_g[1] = -s->acc_g[1];
    s->acc_g[2] = -s->acc_g[2];
    s->gyr_dps[1] = -s->gyr_dps[1];
    s->gyr_dps[2] = -s->gyr_dps[2];
    return ESP_OK;
}
