#include "bmi160.h"

#include <stdbool.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmi160";

#define BMI160_SPI_HZ (4 * 1000 * 1000)

#define BMI160_REG_CHIP_ID   0x00
#define BMI160_REG_ERR       0x02
#define BMI160_REG_PMU       0x03
#define BMI160_REG_DATA      0x0C
#define BMI160_REG_ACC_CONF  0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF  0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_CMD       0x7E
#define BMI160_REG_SPI_TEST  0x7F

#define BMI160_CHIP_ID 0xD1

#define BMI160_CMD_ACC_NORMAL 0x11
#define BMI160_CMD_GYR_NORMAL 0x15
#define BMI160_CMD_SOFT_RESET 0xB6

/* odr = 400 Hz (0x0A), bwp = normal (0x02): less filter delay than 100 Hz */
#define BMI160_CONF_100HZ_NORMAL 0x2A
/* +-8 g: fan vibration clips at +-2 g and the clipped signal shifts the angle */
#define BMI160_ACC_RANGE_8G      0x08
/* acc: odr 400 Hz, bwp = OSR4 (0x00) -> ~40 Hz low-pass, damps prop vibration */
#define BMI160_ACC_CONF_400HZ_OSR4 0x0A
#define BMI160_GYR_RANGE_2000DPS 0x00

#define BMI160_ACC_LSB_PER_G    4096.0f  /* +-8 g */
#define BMI160_GYR_LSB_PER_DPS  16.4f

#define BMI160_PMU_ACC_SHIFT 4
#define BMI160_PMU_GYR_SHIFT 2
#define BMI160_PMU_NORMAL    1

struct bmi160_dev {
    spi_device_handle_t spi;
};

static spi_device_handle_t s_spi;
static bool s_bus_ready;

static int16_t bmi160_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void bmi160_delay_ms(int ms)
{
    /* CONFIG_FREERTOS_HZ is 100, so delays below one tick would not wait at all. */
    vTaskDelay(pdMS_TO_TICKS(ms < 10 ? 10 : ms));
}

static esp_err_t bmi160_read(bmi160_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    if (len == 0 || len >= sizeof(tx)) {
        return ESP_ERR_INVALID_SIZE;
    }

    tx[0] = reg | 0x80;

    spi_transaction_t trans = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(dev->spi, &trans);
    if (err != ESP_OK) {
        return err;
    }

    /* Byte received while the address is shifted out is not register data. */
    memcpy(data, &rx[1], len);
    return ESP_OK;
}

static esp_err_t bmi160_write(bmi160_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7F, value};
    uint8_t rx[2] = {0};
    spi_transaction_t trans = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(dev->spi, &trans);
}

static esp_err_t bmi160_write_checked(bmi160_t *dev, uint8_t reg, uint8_t value)
{
    esp_err_t err = bmi160_write(dev, reg, value);
    if (err != ESP_OK) {
        return err;
    }
    /* Datasheet: >= 450 us between writes in suspend mode, 2 us in normal. */
    esp_rom_delay_us(1000);

    uint8_t readback = 0;
    err = bmi160_read(dev, reg, &readback, 1);
    if (err != ESP_OK) {
        return err;
    }
    if (readback != value) {
        ESP_LOGE(TAG, "reg 0x%02X readback 0x%02X, wrote 0x%02X", reg, readback, value);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t bmi160_select_spi(bmi160_t *dev)
{
    uint8_t dummy = 0;
    /* Rising edge of CS at the end of this transfer leaves I2C and enters SPI. */
    return bmi160_read(dev, BMI160_REG_SPI_TEST, &dummy, 1);
}

static esp_err_t bmi160_check_id(bmi160_t *dev, int timeout_ms)
{
    uint8_t id = 0xFF;
    int waited = 0;

    do {
        /* Each try ends with a rising CS edge, which is what leaves I2C for SPI. */
        ESP_RETURN_ON_ERROR(bmi160_select_spi(dev), TAG, "spi switch");
        bmi160_delay_ms(10);
        waited += 10;

        ESP_RETURN_ON_ERROR(bmi160_read(dev, BMI160_REG_CHIP_ID, &id, 1), TAG, "chip id");
        if (id == BMI160_CHIP_ID) {
            return ESP_OK;
        }
    } while (waited < timeout_ms);

    ESP_LOGW(TAG, "CHIP_ID=0x%02X, expected 0x%02X", id, BMI160_CHIP_ID);
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t bmi160_command(bmi160_t *dev, uint8_t cmd, int shift, int timeout_ms)
{
    ESP_RETURN_ON_ERROR(bmi160_write(dev, BMI160_REG_CMD, cmd), TAG, "cmd");

    int waited = 0;
    while (waited < timeout_ms) {
        bmi160_delay_ms(10);
        waited += 10;

        uint8_t pmu = 0;
        ESP_RETURN_ON_ERROR(bmi160_read(dev, BMI160_REG_PMU, &pmu, 1), TAG, "pmu");
        if (((pmu >> shift) & 0x3) == BMI160_PMU_NORMAL) {
            return ESP_OK;
        }
    }

    uint8_t err_reg = 0;
    bmi160_read(dev, BMI160_REG_ERR, &err_reg, 1);
    ESP_LOGE(TAG, "PMU timeout cmd=0x%02X ERR=0x%02X", cmd, err_reg);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t bmi160_configure(bmi160_t *dev)
{
    ESP_RETURN_ON_ERROR(bmi160_write(dev, BMI160_REG_CMD, BMI160_CMD_SOFT_RESET), TAG, "reset");
    /* Reset drops the chip back to I2C and ignores CS while it starts. */
    bmi160_delay_ms(100);
    ESP_RETURN_ON_ERROR(bmi160_check_id(dev, 1000), TAG, "id after reset");

    /*
     * Power up first: in suspend mode register writes need long gaps and
     * may be ignored (0x28 readback is just the reset default).
     * Accel suspend -> normal is ~4 ms; gyro is ~80 ms.
     */
    ESP_RETURN_ON_ERROR(bmi160_command(dev, BMI160_CMD_ACC_NORMAL, BMI160_PMU_ACC_SHIFT, 50), TAG, "acc on");
    ESP_RETURN_ON_ERROR(bmi160_command(dev, BMI160_CMD_GYR_NORMAL, BMI160_PMU_GYR_SHIFT, 150), TAG, "gyr on");

    ESP_RETURN_ON_ERROR(bmi160_write_checked(dev, BMI160_REG_ACC_CONF, BMI160_ACC_CONF_400HZ_OSR4), TAG, "acc conf");
    ESP_RETURN_ON_ERROR(bmi160_write_checked(dev, BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_8G), TAG, "acc range");
    ESP_RETURN_ON_ERROR(bmi160_write_checked(dev, BMI160_REG_GYR_CONF, BMI160_CONF_100HZ_NORMAL), TAG, "gyr conf");
    ESP_RETURN_ON_ERROR(bmi160_write_checked(dev, BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE_2000DPS), TAG, "gyr range");

    ESP_LOGI(TAG, "ready, accel ±8 g OSR4, gyro ±2000 dps, 400 Hz");
    return ESP_OK;
}

static esp_err_t bmi160_bus_init(void)
{
    if (s_bus_ready) {
        return ESP_OK;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = BMI160_PIN_MOSI,
        .miso_io_num = BMI160_PIN_MISO,
        .sclk_io_num = BMI160_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 32,
    };

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        return err;
    }
    s_bus_ready = true;
    return ESP_OK;
}

static esp_err_t bmi160_add_device(int mode)
{
    if (s_spi != NULL) {
        esp_err_t err = spi_bus_remove_device(s_spi);
        s_spi = NULL;
        if (err != ESP_OK) {
            return err;
        }
    }

    spi_device_interface_config_t devcfg = {
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = BMI160_SPI_HZ,
        .mode = mode,
        .spics_io_num = BMI160_PIN_CS,
        .queue_size = 1,
    };
    return spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi);
}

esp_err_t bmi160_init(bmi160_t **out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "dev");

    static bmi160_t dev;
    const int modes[] = {0, 3};

    ESP_RETURN_ON_ERROR(bmi160_bus_init(), TAG, "bus");

    esp_err_t err = ESP_ERR_NOT_FOUND;
    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        err = bmi160_add_device(modes[i]);
        if (err != ESP_OK) {
            return err;
        }
        dev.spi = s_spi;

        bmi160_delay_ms(10);
        err = bmi160_check_id(&dev, 40);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "no BMI160 in SPI mode %d", modes[i]);
            continue;
        }

        ESP_LOGI(TAG, "SPI%d mode %d, SCK=%d MOSI=%d MISO=%d CS=%d, %d Hz",
                 SPI2_HOST + 1, modes[i], BMI160_PIN_SCLK, BMI160_PIN_MOSI,
                 BMI160_PIN_MISO, BMI160_PIN_CS, BMI160_SPI_HZ);

        err = bmi160_configure(&dev);
        if (err == ESP_OK) {
            *out = &dev;
        }
        return err;
    }

    return err;
}

esp_err_t bmi160_read_sample(bmi160_t *dev, bmi160_sample_t *sample)
{
    ESP_RETURN_ON_FALSE(dev != NULL && sample != NULL, ESP_ERR_INVALID_ARG, TAG, "sample");

    uint8_t raw[12];

    ESP_RETURN_ON_ERROR(bmi160_read(dev, BMI160_REG_DATA, raw, sizeof(raw)), TAG, "data");

    sample->gyr_dps[0] = (float)bmi160_i16(&raw[0]) / BMI160_GYR_LSB_PER_DPS;
    sample->gyr_dps[1] = (float)bmi160_i16(&raw[2]) / BMI160_GYR_LSB_PER_DPS;
    sample->gyr_dps[2] = (float)bmi160_i16(&raw[4]) / BMI160_GYR_LSB_PER_DPS;
    sample->acc_g[0] = (float)bmi160_i16(&raw[6]) / BMI160_ACC_LSB_PER_G;
    sample->acc_g[1] = (float)bmi160_i16(&raw[8]) / BMI160_ACC_LSB_PER_G;
    sample->acc_g[2] = (float)bmi160_i16(&raw[10]) / BMI160_ACC_LSB_PER_G;

    /* 180° about X: chip face flipped. Y and Z reverse, X stays. */
    sample->acc_g[1] = -sample->acc_g[1];
    sample->acc_g[2] = -sample->acc_g[2];
    sample->gyr_dps[1] = -sample->gyr_dps[1];
    sample->gyr_dps[2] = -sample->gyr_dps[2];

    return ESP_OK;
}
