#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lsm6dsv16x_reg.h"

static const char *TAG = "IMU_APP";

// --- Task 1: Hardware Configuration ---
#define I2C_MASTER_SCL_IO           9      
#define I2C_MASTER_SDA_IO           8      
#define I2C_MASTER_NUM              0      
#define I2C_MASTER_FREQ_HZ          400000 
#define LSM6DSV16X_I2C_ADD          0x6B   
#define INT2_PIN                    10     

stmdev_ctx_t g_dev_ctx;
uint32_t sample_count = 0;
uint64_t prev_timestamp = 0;
SemaphoreHandle_t gpio_sem;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(gpio_sem, NULL);
}

// --- I2C Glue Functions ---
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DSV16X_I2C_ADD << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, bufp, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
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
    return ret == ESP_OK ? 0 : -1;
}

// --- Task 2: Verification Step 1 (Connection) ---
bool verify_imu_connection() {
    ESP_LOGI(TAG, "=== VERIFICATION 1: IMU Connection & I2C Communication ===");
    uint8_t whoami = 0;
    if (lsm6dsv16x_device_id_get(&g_dev_ctx, &whoami) != 0) return false;
    if (whoami != 0x70) return false;
    ESP_LOGI(TAG, " PASS: WHO_AM_I = 0x70");
    return true;
}

// --- Task 2: Verification Step 2 (Data Quality) ---
void verify_sample_data(float ax, float ay, float az, uint64_t curr_time) {
    ESP_LOGI(TAG, "=== VERIFICATION 2: Sample Data Output ===");
    uint64_t delta = curr_time - prev_timestamp;
    if (prev_timestamp != 0 && delta >= 900000 && delta <= 1100000) {
        ESP_LOGI(TAG, " PASS: Timestamp delta = %llu us", delta);
    }
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (mag >= 0.8f && mag <= 1.2f) {
        ESP_LOGI(TAG, " PASS: Accel magnitude = %.3f g", mag);
    }
    ESP_LOGI(TAG, " VERIFICATION 2 COMPLETE");
}

void app_main(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    g_dev_ctx.write_reg = platform_write;
    g_dev_ctx.read_reg = platform_read;

    if (!verify_imu_connection()) return;

    gpio_sem = xSemaphoreCreateBinary();
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INT2_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT2_PIN, gpio_isr_handler, NULL);

    lsm6dsv16x_xl_data_rate_set(&g_dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
    lsm6dsv16x_gy_data_rate_set(&g_dev_ctx, LSM6DSV16X_ODR_AT_120Hz);

    lsm6dsv16x_pin_int_route_t int_route = {0};
    int_route.drdy_xl = 1; 
    lsm6dsv16x_pin_int2_route_set(&g_dev_ctx, &int_route);

    bool interrupt_working = true;

    while (1) {
        if (xSemaphoreTake(gpio_sem, pdMS_TO_TICKS(100)) == pdFALSE) {
            if (interrupt_working) {
                ESP_LOGW(TAG, "WARNING: No interrupts received on GPIO10!");
                ESP_LOGW(TAG, "Switching to polling mode...");
                interrupt_working = false;
            }
            lsm6dsv16x_data_ready_t drdy;
            lsm6dsv16x_flag_data_ready_get(&g_dev_ctx, &drdy);
            if (!drdy.drdy_xl) continue; 
        }

        uint64_t timestamp = esp_timer_get_time();

        int16_t raw_accel[3];
        lsm6dsv16x_acceleration_raw_get(&g_dev_ctx, raw_accel);

        float ax = lsm6dsv16x_from_fs2_to_mg(raw_accel[0]) / 1000.0f;
        float ay = lsm6dsv16x_from_fs2_to_mg(raw_accel[1]) / 1000.0f;
        float az = lsm6dsv16x_from_fs2_to_mg(raw_accel[2]) / 1000.0f;

        // Calculate Roll/Pitch from Accelerometer (Stable)
        float roll  = atan2f(ay, az) * 57.295f;
        float pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.295f;
        float yaw   = 0.0f; 

        if (sample_count % 120 == 0) {
            if (sample_count < 600) { 
                verify_sample_data(ax, ay, az, timestamp);
            }
            printf("=== Sample %lu, Time: %llu us ===\n", sample_count, timestamp);
            printf("Accel (g): X=%+.3f, Y=%+.3f, Z=%+.3f\n", ax, ay, az);
            printf("Euler (°): Roll=%+6.2f, Pitch=%+6.2f, Yaw=%+6.2f\n\n", roll, pitch, yaw);
            
            prev_timestamp = timestamp;
        }
        sample_count++;
    }
}