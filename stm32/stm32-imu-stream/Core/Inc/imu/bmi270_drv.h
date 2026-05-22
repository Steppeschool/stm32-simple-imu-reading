#ifndef INC_BMI270_DRV_H_
#define INC_BMI270_DRV_H_

#include "imu_hal.h"

extern const uint8_t  bmi270_config_data[];
extern const uint16_t bmi270_config_data_len;

/* Driver-specific handle — base MUST be first */
typedef struct {
    imu_handle_t      base;
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
} bmi270_handle_t;

extern const imu_driver_t bmi270_driver;

void BMI270_HandleInit(bmi270_handle_t   *h,
                       imu_read_fn        read,
                       imu_write_fn       write,
                       imu_delay_fn       delay,
                       void              *bus_ctx,
                       imu_accel_range_t  accel_range,
                       imu_gyro_range_t   gyro_range);

#endif /* INC_BMI270_DRV_H_ */
