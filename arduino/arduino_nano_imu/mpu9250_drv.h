#ifndef INC_MPU9250_DRV_H_
#define INC_MPU9250_DRV_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "imu_hal.h"
#include "mag_hal.h"

/*
 * MPU9250 / MPU9255 — 6-axis IMU with embedded AK8963 magnetometer.
 *
 * The AK8963 is connected to the MPU9250's auxiliary I2C bus.  This driver
 * configures the MPU9250 as I2C master so the AK8963 is polled automatically
 * and its data is shadowed into EXT_SENS_DATA registers — the same approach
 * used by icm20948_drv for the AK09916.
 *
 * Caller calls IMU_Init() to bring up both accel/gyro and AK8963, then uses
 * MAG_HANDLE (&imu_handle.mag) for magnetometer access.
 *
 * WHO_AM_I: 0x71 (MPU9250) or 0x73 (MPU9255)
 */

typedef struct {
    imu_handle_t      base;
    mag_handle_t      mag;         /* embedded AK8963 mag handle */
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
    uint8_t           ak_asa[3];   /* AK8963 factory sensitivity adjustment (X,Y,Z) */
} mpu9250_handle_t;

extern const imu_driver_t mpu9250_imu_driver;
extern const mag_driver_t mpu9250_mag_driver;

void MPU9250_HandleInit(mpu9250_handle_t *h,
                        imu_read_fn       read,
                        imu_write_fn      write,
                        imu_delay_fn      delay,
                        void             *bus_ctx,     /* MPU9250 I2C context (addr 0x68) */
                        void             *ak_bus_ctx,  /* AK8963  I2C context (addr 0x0C) */
                        imu_accel_range_t accel_range,
                        imu_gyro_range_t  gyro_range);

#ifdef __cplusplus
}
#endif
#endif /* INC_MPU9250_DRV_H_ */
