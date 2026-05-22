#ifndef INC_ICM42688_DRV_H_
#define INC_ICM42688_DRV_H_

#include "imu_hal.h"

typedef struct {
    imu_handle_t      base;
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
} icm42688_handle_t;

extern const imu_driver_t icm42688_driver;

void ICM42688_HandleInit(icm42688_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         imu_accel_range_t  accel_range,
                         imu_gyro_range_t   gyro_range);

#endif /* INC_ICM42688_DRV_H_ */
