#ifndef INC_ICM20948_DRV_H_
#define INC_ICM20948_DRV_H_

#include "imu_hal.h"
#include "mag_hal.h"

/*
 * ICM-20948 driver for the abstraction layer.
 *
 * The chip contains both a 6-axis IMU and an AK09916 magnetometer accessed
 * via the ICM's internal I2C master.  Two independent handles are exposed:
 *
 *   imu_handle_t * → (icm20948_handle_t *)  for IMU_ReadRaw() etc.
 *   mag_handle_t * → &handle->mag           for MAG_ReadRaw() etc.
 *
 * The original icm_20948.c is left untouched; this file is a separate driver
 * that follows the imu_hal / mag_hal abstraction.
 */

typedef struct {
    imu_handle_t      base;          /* MUST be first — cast target for IMU API  */
    mag_handle_t      mag;           /* embedded AK09916 handle                  */
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
    uint8_t           current_bank;  /* tracks active register bank (0-3)        */
} icm20948_handle_t;

extern const imu_driver_t icm20948_imu_driver;
extern const mag_driver_t icm20948_mag_driver;  /* AK09916 via I2C master */

/*
 * Populate all handle fields.  Call before IMU_Init() / MAG_Init().
 *
 * The mag handle shares the same bus callbacks and bus_ctx as the IMU handle;
 * its reads/writes go through the ICM's internal I2C master automatically.
 */
void ICM20948_HandleInit(icm20948_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         imu_accel_range_t  accel_range,
                         imu_gyro_range_t   gyro_range);

/*
 * Optional: compute and write gyro offset trim registers (500-sample average).
 * Call after IMU_Init() with the sensor stationary.
 */
int8_t ICM20948_CalibrateGyro(icm20948_handle_t *h);

#endif /* INC_ICM20948_DRV_H_ */
