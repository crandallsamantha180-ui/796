#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <stdbool.h>
#include <stdint.h>

// Ensure no padding is added by the compiler
typedef struct __attribute__((packed)) {
    uint8_t header[3];       // 0xAA 0xAA 0xAA
    uint32_t sample_counter; // Packet count (uint32)
    uint64_t timestamp;      // ESP32 local timestamp in microseconds (uint64)
    int16_t acc[3];          // X, Y, Z accelerometer samples (raw LSM6DSV16X counts)
    int16_t gyro[3];         // X, Y, Z gyroscope samples (raw LSM6DSV16X counts)
    uint16_t quat[3];        // qx, qy, qz (IEEE 754 half-float from SFLP)
} sensor_packet_t;

// Total Size Check: 3 + 4 + 8 + 6 + 6 + 6 = 33 bytes

_Static_assert(sizeof(sensor_packet_t) == 33, "sensor_packet_t must be 33 bytes");

// Scaling / conversion notes (per field):
// - acc[xyz]: raw counts at ±2 g full-scale. Convert to g using
//             lsm6dsv16x_from_fs2_to_mg(raw) / 1000.0f.
// - gyro[xyz]: raw counts at ±250 dps full-scale. Convert to dps using
//              lsm6dsv16x_from_fs250_to_mdps(raw) / 1000.0f.
// - quat[xyz]: IEEE 754 binary16 (half-float) from SFLP game-rotation.
//              Convert to float using lsm6dsv16x_from_f16_to_f32() or half_to_float().

void init_ble_stack(void);
bool ble_is_connected(void);
int ble_send_sensor_packet(const sensor_packet_t *packet);

#endif // BLE_SERVER_H