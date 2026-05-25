#ifndef INC_ISM330DHCX_DRV_H_
#define INC_ISM330DHCX_DRV_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "imu_hal.h"

typedef struct {
    imu_handle_t      base;
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
} ism330dhcx_handle_t;

extern const imu_driver_t ism330dhcx_driver;

void ISM330DHCX_HandleInit(ism330dhcx_handle_t *h,
                            imu_read_fn          read,
                            imu_write_fn         write,
                            imu_delay_fn         delay,
                            void                *bus_ctx,
                            imu_accel_range_t    accel_range,
                            imu_gyro_range_t     gyro_range);

#ifdef __cplusplus
}
#endif

#endif /* INC_ISM330DHCX_DRV_H_ */
