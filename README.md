# stm32-simple-imu-reading

Stream IMU (and optional magnetometer) data over UART as a compact binary packet.
Two ready-to-run targets are provided: an **STM32 Nucleo-L476RG** and an **Arduino Nano**.
Both targets share the same pure-C sensor driver library and produce an identical packet
format, so the same host-side receiver works with either board.

---

## Repository structure

```
stm32-simple-imu-reading/
├── arduino/
│   └── arduino_nano_imu/          # Arduino sketch + all driver files (copy here to build)
│       ├── arduino_nano_imu.ino   # main sketch — sensor selector at the top
│       ├── imu_hal.h / imu_hal.c  # IMU abstraction layer
│       ├── mag_hal.h              # magnetometer abstraction layer
│       ├── mpu6050_drv.h/c
│       ├── mpu9250_drv.h/c        # includes AK8963 via I2C bypass
│       ├── icm42688_drv.h/c
│       ├── lsm6dso_drv.h/c
│       ├── ism330dhcx_drv.h/c
│       ├── icm20948_drv.h/c       # includes AK09916 via I2C master
│       ├── lis3mdl_drv.h/c
│       ├── mmc5983ma_drv.h/c
│       ├── qmc5883l_drv.h/c
│       └── hmc5883l_drv.h/c
└── stm32/
    └── stm32-imu-stream/          # STM32CubeIDE project (NUCLEO-L476RG)
        └── Core/
            ├── Inc/
            │   ├── sensor_config.h    # ← one-stop sensor selection
            │   ├── serial.h
            │   ├── imu/               # driver headers
            │   └── mag/               # magnetometer driver headers
            └── Src/
                ├── main.c
                ├── serial.c
                ├── imu/               # driver sources
                └── mag/               # magnetometer driver sources
```

---

## Supported sensors

### IMU (accel + gyro)

| Sensor | Interface | Built-in mag | Default I2C address |
|--------|-----------|--------------|---------------------|
| MPU-6050 | I2C | — | 0x68 |
| MPU-9250 / MPU-9255 | I2C | AK8963 (bypass) | 0x68 |
| ICM-42688-P | I2C | — | 0x68 |
| LSM6DSO | I2C | — | 0x6A |
| ISM330DHCX | I2C | — | 0x6B |
| ICM-20948 | SPI (STM32) / I2C (Arduino) | AK09916 | — |

### Standalone magnetometers (optional, paired with any IMU that has no built-in mag)

| Sensor | I2C address |
|--------|-------------|
| QMC5883L | 0x0D |
| HMC5883L | 0x1E |
| LIS3MDL | 0x1C / 0x1E |
| MMC5983MA | 0x30 |

> **All sensors run at 3.3 V.** Do not connect VDD or I2C lines directly to a 5 V supply
> or 5 V logic — this will permanently damage the sensor.

---

## Packet format

Both targets send the same 20-byte binary packet over UART at **115200 baud**:

| Bytes | Content | Type |
|-------|---------|------|
| 0 | `0xAA` header | — |
| 1 | `0xFF` header | — |
| 2–3 | Accel X | int16, little-endian |
| 4–5 | Accel Y | int16, little-endian |
| 6–7 | Accel Z | int16, little-endian |
| 8–9 | Gyro X | int16, little-endian |
| 10–11 | Gyro Y | int16, little-endian |
| 12–13 | Gyro Z | int16, little-endian |
| 14–15 | Mag X | int16, little-endian (zeros if no mag) |
| 16–17 | Mag Y | int16, little-endian |
| 18–19 | Mag Z | int16, little-endian |

Raw counts are sent — no scaling applied. Convert using the sensor's sensitivity from its
datasheet (e.g. for MPU-9250 at ±4 g: accel LSB = 8192 counts/g).

---

## STM32 quickstart (NUCLEO-L476RG)

### 1. Select your sensor

Open `stm32/stm32-imu-stream/Core/Inc/sensor_config.h` and uncomment exactly one IMU:

```c
// #define SENSOR_MPU6050
#define SENSOR_MPU9250
// #define SENSOR_ICM42688
// #define SENSOR_LSM6DSO
// #define SENSOR_ISM330DHCX
// #define SENSOR_ICM20948
```

If using a standalone magnetometer alongside an IMU that has no built-in mag, uncomment
one of:

```c
// #define MAG_LIS3MDL
// #define MAG_MMC5983MA
```

The host bus is **selected automatically**:
- ICM-20948 → **SPI2** (PA9 = CS, PB13 = SCK, PB14 = MISO, PB15 = MOSI)
- All others → **I2C1** (PB8 = SCL, PB9 = SDA)

### 2. Adjust I2C address if needed

Still in `sensor_config.h`, check the address for your sensor (AD0/SA0 pin strapping):

```c
#define MPU9250_I2C_ADDR  0x68   /* AD0=GND → 0x68; AD0=VCC → 0x69 */
```

### 3. Build and flash

Open the project in **STM32CubeIDE**, build, and flash via the on-board ST-LINK.

### 4. Receive data

Open any serial terminal (or your own host application) at **115200 8N1** on the
ST-LINK virtual COM port. Packets arrive at **100 Hz** (TIM4 interrupt).

---

## Arduino quickstart (Arduino Nano)

### 1. Select your sensor

At the top of `arduino/arduino_nano_imu/arduino_nano_imu.ino`, uncomment exactly one:

```cpp
//#define SENSOR_MPU6050
 #define SENSOR_MPU9250
// #define SENSOR_ICM42688
// #define SENSOR_LSM6DSO
// #define SENSOR_ISM330DHCX
// #define SENSOR_ICM20948
```

If adding a standalone magnetometer, uncomment one of:

```cpp
// #define MAG_QMC5883L
// #define MAG_HMC5883L
// #define MAG_LIS3MDL
// #define MAG_MMC5983MA
```

### 2. Wiring

| Arduino Nano pin | Sensor pin |
|-----------------|-----------|
| 3.3 V | VDD / VCC |
| GND | GND |
| A4 (SDA) | SDA |
| A5 (SCL) | SCL |

> The Arduino Nano runs at **5 V**. Power sensors from the **3.3 V pin only** and add
> **I2C level shifters** on SDA/SCL to avoid damaging 3.3 V sensors with 5 V logic.
> Boards that run at 3.3 V natively (Arduino Nano 33, STM32 Nucleo, etc.) do not need
> level shifters.

### 3. Upload

Open the sketch folder in Arduino IDE, select **Arduino Nano (ATmega328P)**, and upload.

### 4. Receive data

Open **Serial Monitor** at **115200 baud** or read packets from your host application.
Packets arrive at approximately **100 Hz** (`delay(4)` at the end of `loop()`).

---

## Gyro calibration

Both targets run `IMU_CalibrateGyro()` on startup (100-sample average). Keep the board
**stationary** for about one second after power-on while the gyro bias is being measured.

---

## Hard-iron magnetometer calibration

Raw magnetometer values include a hard-iron offset that shifts the X/Y/Z readings by a
constant amount. To calibrate:

1. Stream data while slowly rotating the sensor through all orientations.
2. Find the min/max of each axis; bias = (max + min) / 2.
3. Pass the biases to `MAG_SetBias()` in the setup section:

```c
MAG_SetBias(MAG_HANDLE, bias_x, bias_y, bias_z);
```

`MAG_ReadCorrected()` subtracts these values automatically on every read.

---

## Driver architecture

All sensor drivers are built on two thin abstraction layers:

- **`imu_hal.h`** — `imu_handle_t` with vtable (`init`, `read_raw`, `set_accel_range`, …)
- **`mag_hal.h`** — `mag_handle_t` with vtable (`init`, `read_raw`, …)

Drivers are pure C with no HAL dependencies. They receive `read`, `write`, and `delay`
function pointers at init time, so the same driver files compile unchanged for STM32,
Arduino, or any other platform.

To port to a new platform, implement three callbacks:

```c
int8_t my_i2c_read (uint8_t reg, uint8_t *buf, uint16_t len, void *ctx);
int8_t my_i2c_write(uint8_t reg, const uint8_t *buf, uint16_t len, void *ctx);
void   my_delay_ms (uint32_t ms);
```

then call the appropriate `XXX_HandleInit()` and `IMU_Init()`.
