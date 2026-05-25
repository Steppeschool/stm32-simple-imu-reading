#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

/* ==========================================================================
 * sensor_config.h — one-stop shop for sensor selection.
 *
 * Edit sections 1–4; everything else is derived automatically.
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * 1. IMU sensor — uncomment exactly ONE
 * -------------------------------------------------------------------------- */
// #define SENSOR_MPU6050
#define SENSOR_MPU9250
// #define SENSOR_ICM42688
// #define SENSOR_LSM6DSO
// #define SENSOR_ISM330DHCX
// #define SENSOR_ICM20948

/* --------------------------------------------------------------------------
 * 1. Standalone magnetometer — uncomment at most ONE.
 *    Leave all commented when using ICM-20948 or MPU-9250 (built-in mag).
 * -------------------------------------------------------------------------- */
//#define MAG_LIS3MDL
// #define MAG_MMC5983MA


/* --------------------------------------------------------------------------
 * 3. Host bus for the IMU — derived automatically from the selected sensor.
 *    ICM-20948 uses SPI (SPI2 + SPI_CS_Pin); all others use I2C (I2C1).
 *    Override here only if you want to force a different bus.
 * -------------------------------------------------------------------------- */
/* (auto-set in the Derived section below) */

/* --------------------------------------------------------------------------
 * 4. I2C addresses — set to match your board's pin strapping.
 *    Only the address for the selected sensor actually matters.
 * -------------------------------------------------------------------------- */
#define MPU6050_I2C_ADDR     0x68   /* AD0=GND → 0x68;  AD0=VCC → 0x69 */
#define MPU9250_I2C_ADDR     0x68   /* AD0=GND → 0x68;  AD0=VCC → 0x69 */
#define ICM42688_I2C_ADDR    0x68   /* AD0=GND → 0x68;  AD0=VCC → 0x69 */
#define LSM6DSO_I2C_ADDR     0x6A   /* SA0=GND → 0x6A;  SA0=VCC → 0x6B */
#define ISM330DHCX_I2C_ADDR  0x6B   /* SA0=GND → 0x6A;  SA0=VCC → 0x6B */
#define ICM20948_I2C_ADDR    0x68   /* AD0=GND → 0x68;  AD0=VCC → 0x69 */
#define LIS3MDL_I2C_ADDR     0x1C   /* SA1=GND → 0x1C;  SA1=VCC → 0x1E */
#define MMC5983MA_I2C_ADDR   0x30   /* fixed */
/* --------------------------------------------------------------------------
 * 5. Accel / gyro full-scale range
 * -------------------------------------------------------------------------- */
#define CFG_ACCEL_RANGE   IMU_ACCEL_RANGE_4G
#define CFG_GYRO_RANGE    IMU_GYRO_RANGE_2000DPS



/* ==========================================================================
 * Derived — do not edit below this line
 * ========================================================================== */

/* Bus selection: ICM-20948 → SPI; everything else → I2C */
#if defined(SENSOR_ICM20948)
    #if !defined(SENSOR_BUS_SPI) && !defined(SENSOR_BUS_I2C)
        #define SENSOR_BUS_SPI
    #endif
#else
    #if !defined(SENSOR_BUS_SPI) && !defined(SENSOR_BUS_I2C)
        #define SENSOR_BUS_I2C
    #endif
#endif

/* Resolve the active IMU I2C address from the per-sensor macro above */
#if defined(SENSOR_MPU6050)
    #define SENSOR_I2C_ADDR  MPU6050_I2C_ADDR
#elif defined(SENSOR_MPU9250)
    #define SENSOR_I2C_ADDR  MPU9250_I2C_ADDR
#elif defined(SENSOR_ICM42688)
    #define SENSOR_I2C_ADDR  ICM42688_I2C_ADDR
#elif defined(SENSOR_LSM6DSO)
    #define SENSOR_I2C_ADDR  LSM6DSO_I2C_ADDR
#elif defined(SENSOR_ISM330DHCX)
    #define SENSOR_I2C_ADDR  ISM330DHCX_I2C_ADDR
#elif defined(SENSOR_ICM20948)
    #define SENSOR_I2C_ADDR  ICM20948_I2C_ADDR
#endif

/* IMU driver include */
#if defined(SENSOR_MPU6050)
    #include "imu/mpu6050_drv.h"
    #define SENSOR_HAS_MAG  0
#elif defined(SENSOR_MPU9250)
    #include "imu/mpu9250_drv.h"
    #define SENSOR_HAS_MAG  1
#elif defined(SENSOR_ICM42688)
    #include "imu/icm42688_drv.h"
    #define SENSOR_HAS_MAG  0
#elif defined(SENSOR_LSM6DSO)
    #include "imu/lsm6dso_drv.h"
    #define SENSOR_HAS_MAG  0
#elif defined(SENSOR_ISM330DHCX)
    #include "imu/ism330dhcx_drv.h"
    #define SENSOR_HAS_MAG  0
#elif defined(SENSOR_ICM20948)
    #include "imu/icm20948_drv.h"
    #define SENSOR_HAS_MAG  1
#else
    #error "sensor_config.h: no IMU selected — uncomment one SENSOR_xxx define."
#endif

/* Standalone magnetometer driver include */
#if defined(MAG_LIS3MDL)
    #include "mag/lis3mdl_drv.h"
    #define MAG_STANDALONE  1
    #define MAG_I2C_ADDR    LIS3MDL_I2C_ADDR
#elif defined(MAG_MMC5983MA)
    #include "mag/mmc5983ma_drv.h"
    #define MAG_STANDALONE  1
    #define MAG_I2C_ADDR    MMC5983MA_I2C_ADDR
#else
    #define MAG_STANDALONE  0
#endif

/* MAG_DEFINED / MAG_HANDLE — unified mag access regardless of source */
#if SENSOR_HAS_MAG
    #define MAG_DEFINED  1
    #define MAG_HANDLE   (&imu.mag)
#elif MAG_STANDALONE
    #define MAG_DEFINED  1
    #define MAG_HANDLE   ((mag_handle_t *)&mag_handle)
#else
    #define MAG_DEFINED  0
#endif

/* IMU_SPI_MODE — passed to ICM20948_HandleInit */
#ifdef SENSOR_BUS_SPI
    #define IMU_SPI_MODE  1
#else
    #define IMU_SPI_MODE  0
#endif

#endif /* SENSOR_CONFIG_H */
