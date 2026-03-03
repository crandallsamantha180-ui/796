# 📄 Project Record: ESP32-S3 BLE Implementation

**Branch:** `feature/ble-transmission`  
**Objective:** Integrate NimBLE stack into the existing project using a modular approach (preserving `main.c`).

---

## 🟢 Checkpoint 1: Environment Configuration
**Goal:** Ensure the project is configured for NimBLE before writing code.

### Action Items
1.  **Create/Switch to branch:**
    ```bash
    git checkout -b feature/ble-transmission
    ```
2.  **Open configuration:**
    ```bash
    idf.py menuconfig
    ```
3.  **Navigate to:** `Component config` → `Bluetooth` → `Bluetooth Host`
4.  **Select:** `NimBLE - BLE only`
5.  **Save and Quit.**

### Validation (Test BEFORE Committing)
* **Command:**
    ```bash
    grep "CONFIG_BT_NIMBLE_ENABLED" sdkconfig
    ```
* **Pass Criteria:** Output must contain `CONFIG_BT_NIMBLE_ENABLED=y`.
* **Fail Criteria:** Output is empty or ends in `=n`.

### Git Commit
```bash
git add sdkconfig
git commit -m "build: enable NimBLE stack in project configuration"
---

## 🟢

Checkpoint 2: Modular Structure Setup
Goal: Create the file structure so the compiler "sees" the new module.

[x] 1. Create Source Files

Create main/ble_server.c.

Create main/ble_server.h with header guards.

[x] 2. Update Build System
Edit main/CMakeLists.txt: Add "ble_server.c" to the SRCS list.

[ ] 3. Verification
Run idf.py build.
Pass Criteria: Build completes successfully (no missing file errors).

💾 Commit
Bash

git add main/ble_server.c main/ble_server.h main/CMakeLists.txt
git commit -m "feat: add empty ble_server module and update cmake"

---

## 🟢
Checkpoint 3: Basic BLE Integration
Goal: Get the ESP32 advertising and visible to a phone.

[x] 1. Implement ble_server.c
Add the NimBLE stack initialization, GATT server setup, and Advertising logic.

[x] 2. Refactor Entry Point
Inside ble_server.c, ensure the main function is named void init_ble_stack(void) (not app_main).

[x] 3. Integrate with Main
In main.c: Add #include "ble_server.h" and call init_ble_stack(); inside app_main.

[ ] 4. Verification (Discovery)
Run idf.py flash monitor.
Open nRF Connect on your phone and scan.
Pass Criteria: Device "ESP32-IMU" is visible.

💾 Commit
Bash

git add .
git commit -m "feat: implement basic BLE advertising and integrate into main"

---

## 🟢🟢
Checkpoint 4: Packet Protocol Definition
Goal: Define the exact binary structure for sensor data.

[x] 1. Define Struct in ble_server.h
Implement the packed struct exactly as required:

C

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t header[3];       // 0xAA 0xAA 0xAA
    uint32_t sample_counter; // Incrementing counter
    uint64_t timestamp;      // ESP32 timestamp (microseconds)
    int16_t acc[3];          // X, Y, Z
    int16_t gyro[3];         // X, Y, Z
    uint16_t quat[3];        // qx, qy, qz (half-float)
} sensor_packet_t;
[x] 2. Scaling Notes
Acc and Gyro are raw LSM6DSV16X counts (convert via lsm6dsv16x_from_fs2_to_mg / lsm6dsv16x_from_fs250_to_mdps).
Quat is IEEE 754 half-float (convert via lsm6dsv16x_from_f16_to_f32 or half_to_float).
[ ] 3. Verification (Size)
Add a temporary log in main.c: ESP_LOGI("TEST", "Size: %d", sizeof(sensor_packet_t));
Pass Criteria: Output must be 33 bytes.

💾 Commit
Bash

git add main/ble_server.h
git commit -m "feat: define binary sensor packet structure"
## 🟢 Checkpoint 5: Live Transmission & Verification
Goal: Send live data and perform the final validation using nRF Connect.

[x] 1. Create Data Path
Send packets from the IMU acquisition loop.

[x] 2. Populate Packet
Set Header to 0xAA 0xAA 0xAA.
Increment Counter.
Set Timestamp (esp_timer_get_time()).
Fill Sensor data (Lab 2 raw values).

[x] 3. Send Notification
Use ble_gatts_notify_custom to send the packet.

[x] 3a. Rate Limit
Throttle notifications and prints to 50 Hz (20 ms period) in main loop.

[ ] 4. 🔍 Final Verification (nRF Connect)

Connect: Verify status is "Connected".

Subscribe: Enable notifications (click "Play/Download" icon).

Check Flow: Verify values are updating continuously.

Check Data: Pause and inspect the Hex:

[ ] Header starts with AA AA AA.

[ ] Counter bytes (offset 3-6) increment.

[ ] Timestamp bytes (offset 7-14) change.

[ ] Sensor bytes are non-zero.

💾 Commit
Bash

git add .
git commit -m "feat: implement periodic binary packet transmission"