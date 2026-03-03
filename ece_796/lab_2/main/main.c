/**
 * Lab 2 – LSM6DSV16X IMU Data Acquisition
 *
 * Hardware:  ESP32-S3
 *            SDA → GPIO 8  |  SCL → GPIO 9  |  INT2 → GPIO 10
 *            I²C address 0x6B (SA0 pulled HIGH on board)
 *
 * Configuration:
 *   - Accelerometer : HAODR mode, 2 g full-scale, 120 Hz ODR
 *   - Gyroscope     : HAODR mode, 250 dps full-scale, 120 Hz ODR
 *   - SFLP          : game-rotation vector enabled at 120 Hz
 *   - FIFO          : stream mode, batching accel + gyro + SFLP game-rotation
 *   - Interrupt     : gyroscope DRDY → INT2 (GPIO 10), rising edge
 *
 * Output (every sample at ~120 Hz):
 *   [counter] t=<us> | Acc XYZ g | Gyr XYZ dps | Euler Roll/Pitch/Yaw deg
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lsm6dsv16x_reg.h"
#include "ble_server.h"

static const char *TAG = "IMU_APP";

/* ---------- Hardware pin / bus constants ---------------------------------- */
#define I2C_MASTER_SCL_IO    9
#define I2C_MASTER_SDA_IO    8
#define I2C_MASTER_NUM       0
#define I2C_MASTER_FREQ_HZ   400000
#define LSM6DSV16X_I2C_ADD   0x6B
#define INT2_PIN             10

#define RAD2DEG  57.29577951f
#define BLE_SAMPLE_PERIOD_US 20000u  /* 50 Hz notification/print rate */

/* ---------- Globals ------------------------------------------------------- */
static stmdev_ctx_t       g_dev_ctx;
static SemaphoreHandle_t  gpio_sem;
static volatile uint64_t  isr_timestamp;   /* written in ISR, read in task  */
static uint32_t           sample_count = 0;

/* ---------- ISR: capture timestamp, signal main loop --------------------- */
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    isr_timestamp = esp_timer_get_time();
    xSemaphoreGiveFromISR(gpio_sem, NULL);
}

/* ---------- I²C platform glue ------------------------------------------- */
static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSV16X_I2C_ADD << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, bufp, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSV16X_I2C_ADD << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSV16X_I2C_ADD << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, bufp, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, bufp + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}

/* ---------- Half-precision → single-precision float conversion ----------- */
static inline float half_to_float(uint16_t h)
{
    uint32_t bits = lsm6dsv16x_from_f16_to_f32(h);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

/* ========================================================================= */
void app_main(void)
{
    /* 1. Initialise I²C master */
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    /* 2. Bind I²C platform functions to ST driver context */
    g_dev_ctx.write_reg = platform_write;
    g_dev_ctx.read_reg  = platform_read;

    /* 3. Verify WHO_AM_I register */
    ESP_LOGI(TAG, "=== IMU Connection Verification ===");
    uint8_t whoami = 0;
    if (lsm6dsv16x_device_id_get(&g_dev_ctx, &whoami) != 0 ||
        whoami != LSM6DSV16X_ID) {
        ESP_LOGE(TAG, "FAIL: WHO_AM_I = 0x%02X (expected 0x%02X)", whoami, LSM6DSV16X_ID);
        return;
    }
    ESP_LOGI(TAG, "PASS: WHO_AM_I = 0x%02X", whoami);

    /* 4. Software reset; wait for device to reinitialise */
    lsm6dsv16x_sw_reset(&g_dev_ctx);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 5. Block Data Update: prevent reading of split high/low register bytes */
    lsm6dsv16x_block_data_update_set(&g_dev_ctx, PROPERTY_ENABLE);

    /* 6. Accelerometer: 2 g full-scale, HAODR mode, 120 Hz ODR */
    lsm6dsv16x_xl_full_scale_set(&g_dev_ctx, LSM6DSV16X_2g);
    lsm6dsv16x_xl_mode_set(&g_dev_ctx, LSM6DSV16X_XL_HIGH_ACCURACY_ODR_MD);
    lsm6dsv16x_xl_data_rate_set(&g_dev_ctx, LSM6DSV16X_ODR_AT_120Hz);

    /* 7. Gyroscope: 250 dps full-scale, HAODR mode, 120 Hz ODR */
    lsm6dsv16x_gy_full_scale_set(&g_dev_ctx, LSM6DSV16X_250dps);
    lsm6dsv16x_gy_mode_set(&g_dev_ctx, LSM6DSV16X_GY_HIGH_ACCURACY_ODR_MD);
    lsm6dsv16x_gy_data_rate_set(&g_dev_ctx, LSM6DSV16X_ODR_AT_120Hz);

    /* 8. Enable SFLP game-rotation vector at 120 Hz */
    lsm6dsv16x_sflp_game_rotation_set(&g_dev_ctx, PROPERTY_ENABLE);
    lsm6dsv16x_sflp_data_rate_set(&g_dev_ctx, LSM6DSV16X_SFLP_120Hz);

    /* 9. FIFO: batch accel + gyro + SFLP game-rotation in stream mode */
    lsm6dsv16x_fifo_xl_batch_set(&g_dev_ctx, LSM6DSV16X_XL_BATCHED_AT_120Hz);
    lsm6dsv16x_fifo_gy_batch_set(&g_dev_ctx, LSM6DSV16X_GY_BATCHED_AT_120Hz);
    lsm6dsv16x_fifo_sflp_raw_t sflp_batch = {
        .game_rotation = 1,
        .gravity       = 0,
        .gbias         = 0,
    };
    lsm6dsv16x_fifo_sflp_batch_set(&g_dev_ctx, sflp_batch);
    lsm6dsv16x_fifo_mode_set(&g_dev_ctx, LSM6DSV16X_STREAM_MODE);

    /* 10. Pulsed data-ready: generate a clean rising edge for the GPIO ISR */
    lsm6dsv16x_data_ready_mode_set(&g_dev_ctx, LSM6DSV16X_DRDY_PULSED);

    /* 11. Route gyroscope DRDY to INT2 (GPIO 10) */
    lsm6dsv16x_pin_int_route_t int2_route = {0};
    int2_route.drdy_g = 1;
    lsm6dsv16x_pin_int2_route_set(&g_dev_ctx, &int2_route);

    /* 12. Configure GPIO 10 as a rising-edge interrupt input */
    gpio_sem = xSemaphoreCreateBinary();
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INT2_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT2_PIN, gpio_isr_handler, NULL);

    ESP_ERROR_CHECK(nvs_flash_init());
    init_ble_stack();

    ESP_LOGI(TAG, "Configured: HAODR 120 Hz | SFLP game-rotation 120 Hz | FIFO stream");
    ESP_LOGI(TAG, "Acquiring IMU data...\n");

    bool   interrupt_working = true;
    float  ax = 0, ay = 0, az = 0;   /* acceleration, g          */
    float  gx = 0, gy = 0, gz = 0;   /* angular velocity, dps    */
    float  qx = 0, qy = 0, qz = 0;   /* quaternion components    */
    int16_t acc_raw[3] = {0};
    int16_t gyro_raw[3] = {0};
    uint16_t quat_raw[3] = {0};
    uint64_t last_ble_tx_ts = 0;
    uint64_t last_sample_ts = 0;
    bool header_printed = false;

    while (1) {
        /* Wait for gyro DRDY interrupt; fall back to polling after 100 ms */
        if (xSemaphoreTake(gpio_sem, pdMS_TO_TICKS(100)) == pdFALSE) {
            if (interrupt_working) {
                ESP_LOGW(TAG, "No interrupt on GPIO %d – switching to polling", INT2_PIN);
                interrupt_working = false;
            }
            lsm6dsv16x_data_ready_t drdy;
            lsm6dsv16x_flag_data_ready_get(&g_dev_ctx, &drdy);
            if (!drdy.drdy_gy) continue;
        }

        /* Snapshot the timestamp captured in the ISR */
        uint64_t timestamp = isr_timestamp;
        if (last_sample_ts != 0 && timestamp <= last_sample_ts) {
            ESP_LOGW(TAG, "Non-monotonic timestamp: %llu <= %llu",
                     timestamp, last_sample_ts);
        }
        last_sample_ts = timestamp;

        /* Drain all available FIFO records and decode by tag */
        lsm6dsv16x_fifo_status_t fifo_status;
        lsm6dsv16x_fifo_status_get(&g_dev_ctx, &fifo_status);

        for (uint16_t i = 0; i < fifo_status.fifo_level; i++) {
            lsm6dsv16x_fifo_out_raw_t rec;
            lsm6dsv16x_fifo_out_raw_get(&g_dev_ctx, &rec);

            switch ((lsm6dsv16x_fifo_tag)rec.tag) {

                case LSM6DSV16X_XL_NC_TAG:
                    memcpy(acc_raw, rec.data, 6);
                    ax = lsm6dsv16x_from_fs2_to_mg(acc_raw[0]) / 1000.0f;
                    ay = lsm6dsv16x_from_fs2_to_mg(acc_raw[1]) / 1000.0f;
                    az = lsm6dsv16x_from_fs2_to_mg(acc_raw[2]) / 1000.0f;
                    break;

                case LSM6DSV16X_GY_NC_TAG:
                    memcpy(gyro_raw, rec.data, 6);
                    gx = lsm6dsv16x_from_fs250_to_mdps(gyro_raw[0]) / 1000.0f;
                    gy = lsm6dsv16x_from_fs250_to_mdps(gyro_raw[1]) / 1000.0f;
                    gz = lsm6dsv16x_from_fs250_to_mdps(gyro_raw[2]) / 1000.0f;
                    break;

                case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
                    memcpy(quat_raw, rec.data, 6);
                    qx = half_to_float(quat_raw[0]);
                    qy = half_to_float(quat_raw[1]);
                    qz = half_to_float(quat_raw[2]);
                    break;

                default:
                    break;
            }
        }

        /* Recover qw from the unit-quaternion constraint */
        float qw = sqrtf(fmaxf(0.0f, 1.0f - qx*qx - qy*qy - qz*qz));

        /* Quaternion → Euler angles, ZYX convention, degrees */
        float roll  = atan2f(2.0f*(qw*qx + qy*qz),
                             1.0f - 2.0f*(qx*qx + qy*qy)) * RAD2DEG;
        float pitch = asinf(fmaxf(-1.0f, fminf(1.0f,
                             2.0f*(qw*qy - qz*qx)))) * RAD2DEG;
        float yaw   = atan2f(2.0f*(qw*qz + qx*qy),
                             1.0f - 2.0f*(qy*qy + qz*qz)) * RAD2DEG;

        bool do_tx = (timestamp - last_ble_tx_ts) >= BLE_SAMPLE_PERIOD_US;
        if (do_tx) {
            last_ble_tx_ts = timestamp;

            if ((sample_count % 5) == 0) {
                if (!header_printed) {
                    printf("%6s %12s %8s %8s %8s %8s %8s %8s %8s %8s %8s\n",
                           "count", "t_us", "ax_g", "ay_g", "az_g",
                           "gx", "gy", "gz", "roll", "pitch", "yaw");
                    header_printed = true;
                }
                printf("%6lu %12llu %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.2f %8.2f %8.2f\n",
                       (unsigned long)sample_count, timestamp,
                       ax, ay, az,
                       gx, gy, gz,
                       roll, pitch, yaw);
            }

            sensor_packet_t packet = {
                .header = {0xAA, 0xAA, 0xAA},
                .sample_counter = sample_count,
                .timestamp = timestamp,
            };
            memcpy(packet.acc, acc_raw, sizeof(packet.acc));
            memcpy(packet.gyro, gyro_raw, sizeof(packet.gyro));
            memcpy(packet.quat, quat_raw, sizeof(packet.quat));
            ble_send_sensor_packet(&packet);
        }

        sample_count++;
    }
}
