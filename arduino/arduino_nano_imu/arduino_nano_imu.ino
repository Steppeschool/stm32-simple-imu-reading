#include <Wire.h>
#include "imu_hal.h"
#include "mag_hal.h"

/* -----------------------------------------------------------------------
 * IMU sensor selector — uncomment exactly ONE
 * I2C addresses assume default pin strapping (AD0/SA0 = GND)
 * --------------------------------------------------------------------- */
//#define SENSOR_MPU6050
 #define SENSOR_MPU9250
// #define SENSOR_ICM42688
// #define SENSOR_LSM6DSO
// #define SENSOR_ISM330DHCX
// #define SENSOR_ICM20948

/* -----------------------------------------------------------------------
 * Standalone magnetometer selector — uncomment at most ONE.
 * Leave all commented if using ICM-20948 (has built-in AK09916) or no mag.
 * --------------------------------------------------------------------- */
// #define MAG_QMC5883L
// #define MAG_HMC5883L
// #define MAG_LIS3MDL
// #define MAG_MMC5983MA

/* -----------------------------------------------------------------------
 * IMU range configuration
 * --------------------------------------------------------------------- */
#define CFG_ACCEL_RANGE  IMU_ACCEL_RANGE_4G
#define CFG_GYRO_RANGE   IMU_GYRO_RANGE_2000DPS

/* -----------------------------------------------------------------------
 * IMU conditional includes
 * --------------------------------------------------------------------- */
#if defined(SENSOR_MPU6050)
    #include "mpu6050_drv.h"
    #define SENSOR_NAME     "MPU6050"
    #define SENSOR_I2C_ADDR 0x68
    #define SENSOR_HAS_MAG  0

#elif defined(SENSOR_MPU9250)
    #include "mpu9250_drv.h"
    #define SENSOR_NAME     "MPU9250"
    #define SENSOR_I2C_ADDR 0x68   /* AD0 = GND; use 0x69 if AD0 = VCC */
    #define SENSOR_HAS_MAG  1      /* AK8963 driven via I2C master */

#elif defined(SENSOR_ICM42688)
    #include "icm42688_drv.h"
    #define SENSOR_NAME     "ICM-42688"
    #define SENSOR_I2C_ADDR 0x68
    #define SENSOR_HAS_MAG  0

#elif defined(SENSOR_LSM6DSO)
    #include "lsm6dso_drv.h"
    #define SENSOR_NAME     "LSM6DSO"
    #define SENSOR_I2C_ADDR 0x6A
    #define SENSOR_HAS_MAG  0

#elif defined(SENSOR_ISM330DHCX)
    #include "ism330dhcx_drv.h"
    #define SENSOR_NAME     "ISM330DHCX"
    #define SENSOR_I2C_ADDR 0x6B
    #define SENSOR_HAS_MAG  0

#elif defined(SENSOR_ICM20948)
    #include "icm20948_drv.h"
    #define SENSOR_NAME     "ICM-20948"
    #define SENSOR_I2C_ADDR 0x69
    #define SENSOR_HAS_MAG  1

#else
    #error "No IMU sensor selected — uncomment one SENSOR_xxx define."
#endif

/* -----------------------------------------------------------------------
 * Standalone magnetometer conditional includes
 * --------------------------------------------------------------------- */
#if defined(MAG_QMC5883L)
    #include "qmc5883l_drv.h"
    #define MAG_NAME        "QMC5883L"
    #define MAG_I2C_ADDR    0x0D
    #define MAG_STANDALONE  1

#elif defined(MAG_HMC5883L)
    #include "hmc5883l_drv.h"
    #define MAG_NAME        "HMC5883L"
    #define MAG_I2C_ADDR    0x1E
    #define MAG_STANDALONE  1

#elif defined(MAG_LIS3MDL)
    #include "lis3mdl_drv.h"
    #define MAG_NAME        "LIS3MDL"
    #define MAG_I2C_ADDR    0x1E   /* SA1 = 1; use 0x1C if SA1 = 0 */
    #define MAG_STANDALONE  1

#elif defined(MAG_MMC5983MA)
    #include "mmc5983ma_drv.h"
    #define MAG_NAME        "MMC5983MA"
    #define MAG_I2C_ADDR    0x30
    #define MAG_STANDALONE  1

#else
    #define MAG_STANDALONE  0
#endif

/* -----------------------------------------------------------------------
 * MAG_DEFINED — true when any magnetometer is present (built-in or standalone).
 * MAG_HANDLE  — pointer to the active mag_handle_t regardless of source.
 * --------------------------------------------------------------------- */
#if SENSOR_HAS_MAG
    #define MAG_DEFINED   1
    #define MAG_HANDLE    (&imu_handle.mag)
#elif MAG_STANDALONE
    #define MAG_DEFINED   1
    #define MAG_HANDLE    ((mag_handle_t *)&mag_handle)
#else
    #define MAG_DEFINED   0
#endif

/* -----------------------------------------------------------------------
 * Bus context: carries the 7-bit I2C address into the callbacks
 * --------------------------------------------------------------------- */
typedef struct { uint8_t addr; } i2c_ctx_t;

static i2c_ctx_t imu_bus_ctx = { SENSOR_I2C_ADDR };

#if defined(SENSOR_MPU9250)
static i2c_ctx_t ak_bus_ctx  = { 0x0C };   /* AK8963 on bypass bus */
#endif

#if MAG_STANDALONE
static i2c_ctx_t mag_bus_ctx = { MAG_I2C_ADDR };
#endif

/* -----------------------------------------------------------------------
 * Arduino I2C bus callbacks (shared by IMU and mag — address is in ctx)
 * --------------------------------------------------------------------- */
static int8_t i2c_read(uint8_t reg, uint8_t *buf, uint16_t len, void *ctx)
{
    uint8_t addr = ((i2c_ctx_t *)ctx)->addr;
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    Wire.requestFrom(addr, (uint8_t)len);
    for (uint16_t i = 0; i < len; i++) buf[i] = Wire.read();
    return 0;
}

static int8_t i2c_write(uint8_t reg, const uint8_t *buf, uint16_t len, void *ctx)
{
    uint8_t addr = ((i2c_ctx_t *)ctx)->addr;
    Wire.beginTransmission(addr);
    Wire.write(reg);
    for (uint16_t i = 0; i < len; i++) Wire.write(buf[i]);
    return Wire.endTransmission() == 0 ? 0 : -1;
}

static void arduino_delay(uint32_t ms) { delay(ms); }

/* -----------------------------------------------------------------------
 * IMU handle
 * --------------------------------------------------------------------- */
#if defined(SENSOR_MPU6050)
    static mpu6050_handle_t imu_handle;
#elif defined(SENSOR_MPU9250)
    static mpu9250_handle_t imu_handle;
#elif defined(SENSOR_ICM42688)
    static icm42688_handle_t imu_handle;
#elif defined(SENSOR_LSM6DSO)
    static lsm6dso_handle_t imu_handle;
#elif defined(SENSOR_ISM330DHCX)
    static ism330dhcx_handle_t imu_handle;
#elif defined(SENSOR_ICM20948)
    static icm20948_handle_t imu_handle;
#endif

/* -----------------------------------------------------------------------
 * Standalone magnetometer handle
 * --------------------------------------------------------------------- */
#if defined(MAG_QMC5883L)
    static qmc5883l_handle_t mag_handle;
#elif defined(MAG_HMC5883L)
    static hmc5883l_handle_t mag_handle;
#elif defined(MAG_LIS3MDL)
    static lis3mdl_handle_t mag_handle;
#elif defined(MAG_MMC5983MA)
    static mmc5983ma_handle_t mag_handle;
#endif

#if MAG_DEFINED
static bool mag_ok = false;
#endif

/* -----------------------------------------------------------------------
 * Packet format (20 bytes):
 *   [0]     0xAA  header byte 1
 *   [1]     0xFF  header byte 2
 *   [2-3]   accel X  (int16, little-endian)
 *   [4-5]   accel Y
 *   [6-7]   accel Z
 *   [8-9]   gyro  X
 *   [10-11] gyro  Y
 *   [12-13] gyro  Z
 *   [14-15] mag   X  (zeros if no magnetometer)
 *   [16-17] mag   Y
 *   [18-19] mag   Z
 * --------------------------------------------------------------------- */
static void send_packet(const imu_raw_data_t *imu, const mag_raw_data_t *mag)
{
    uint8_t buf[20];
    buf[0]  = 0xAA;
    buf[1]  = 0xFF;
    buf[2]  = (uint8_t)(imu->accel_x);
    buf[3]  = (uint8_t)(imu->accel_x >> 8);
    buf[4]  = (uint8_t)(imu->accel_y);
    buf[5]  = (uint8_t)(imu->accel_y >> 8);
    buf[6]  = (uint8_t)(imu->accel_z);
    buf[7]  = (uint8_t)(imu->accel_z >> 8);
    buf[8]  = (uint8_t)(imu->gyro_x);
    buf[9]  = (uint8_t)(imu->gyro_x >> 8);
    buf[10] = (uint8_t)(imu->gyro_y);
    buf[11] = (uint8_t)(imu->gyro_y >> 8);
    buf[12] = (uint8_t)(imu->gyro_z);
    buf[13] = (uint8_t)(imu->gyro_z >> 8);
    buf[14] = (uint8_t)(mag->x);
    buf[15] = (uint8_t)(mag->x >> 8);
    buf[16] = (uint8_t)(mag->y);
    buf[17] = (uint8_t)(mag->y >> 8);
    buf[18] = (uint8_t)(mag->z);
    buf[19] = (uint8_t)(mag->z >> 8);
    Serial.write(buf, 20);
}

/* -----------------------------------------------------------------------
 * setup()
 * --------------------------------------------------------------------- */
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    /* --- IMU init --- */
#if defined(SENSOR_MPU6050)
    MPU6050_HandleInit(&imu_handle,
                       i2c_read, i2c_write, arduino_delay, &imu_bus_ctx,
                       MPU6050_ADDR_LOW,
                       CFG_ACCEL_RANGE, CFG_GYRO_RANGE);

#elif defined(SENSOR_MPU9250)
    MPU9250_HandleInit(&imu_handle,
                       i2c_read, i2c_write, arduino_delay, &imu_bus_ctx, &ak_bus_ctx,
                       CFG_ACCEL_RANGE, CFG_GYRO_RANGE);

#elif defined(SENSOR_ICM42688)
    ICM42688_HandleInit(&imu_handle,
                        i2c_read, i2c_write, arduino_delay, &imu_bus_ctx,
                        CFG_ACCEL_RANGE, CFG_GYRO_RANGE);

#elif defined(SENSOR_LSM6DSO)
    LSM6DSO_HandleInit(&imu_handle,
                       i2c_read, i2c_write, arduino_delay, &imu_bus_ctx,
                       CFG_ACCEL_RANGE, CFG_GYRO_RANGE);

#elif defined(SENSOR_ISM330DHCX)
    ISM330DHCX_HandleInit(&imu_handle,
                           i2c_read, i2c_write, arduino_delay, &imu_bus_ctx,
                           CFG_ACCEL_RANGE, CFG_GYRO_RANGE);

#elif defined(SENSOR_ICM20948)
    ICM20948_HandleInit(&imu_handle,
                        i2c_read, i2c_write, arduino_delay, &imu_bus_ctx,
                        CFG_ACCEL_RANGE, CFG_GYRO_RANGE,
                        0 /* spi_mode = 0 for I2C */);
#endif

    if (IMU_Init((imu_handle_t *)&imu_handle) != IMU_OK)
        while (1);

    IMU_CalibrateGyro((imu_handle_t *)&imu_handle, 100);

    /* --- Standalone magnetometer init --- */
#if defined(MAG_QMC5883L)
    QMC5883L_HandleInit(&mag_handle,
                        i2c_read, i2c_write, arduino_delay, &mag_bus_ctx,
                        QMC5883L_RNG_2G, QMC5883L_ODR_100HZ);

#elif defined(MAG_HMC5883L)
    HMC5883L_HandleInit(&mag_handle,
                        i2c_read, i2c_write, arduino_delay, &mag_bus_ctx,
                        HMC5883L_GAIN_1090);

#elif defined(MAG_LIS3MDL)
    LIS3MDL_HandleInit(&mag_handle,
                       i2c_read, i2c_write, arduino_delay, &mag_bus_ctx,
                       LIS3MDL_FS_4G);

#elif defined(MAG_MMC5983MA)
    MMC5983MA_HandleInit(&mag_handle,
                         i2c_read, i2c_write, arduino_delay, &mag_bus_ctx,
                         MMC5983MA_BW_100HZ);
#endif

#if SENSOR_HAS_MAG
    /* Built-in mag (e.g. AK8963 in MPU-9250) is already initialised inside
     * IMU_Init(). Calling MAG_Init() a second time would reset the sensor
     * and leave it with no time to complete its first measurement. */
    mag_ok = true;
    MAG_SetBias(MAG_HANDLE, 0, 0, 0);   /* replace with measured hard-iron biases */
#elif MAG_STANDALONE
    if (MAG_Init(MAG_HANDLE) == IMU_OK) {
        mag_ok = true;
        /* Set known hard-iron biases (replace with your measured values) */
        MAG_SetBias(MAG_HANDLE, 660, -300, 430);
    }
#endif
}

/* -----------------------------------------------------------------------
 * loop()
 * --------------------------------------------------------------------- */
void loop()
{
    imu_raw_data_t imu_data;
    static mag_raw_data_t mag_data = { 0, 0, 0 };  /* retains last good reading */

    if (IMU_ReadRaw((imu_handle_t *)&imu_handle, &imu_data) != IMU_OK)
        return;

#if MAG_DEFINED
    if (mag_ok)
        MAG_ReadCorrected(MAG_HANDLE, &mag_data);
#endif

    send_packet(&imu_data, &mag_data);

    delay(4);  /* ~100 Hz */
}
