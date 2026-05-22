#ifndef INC_MPU6050_DRV_H_
#define INC_MPU6050_DRV_H_

#include "imu_hal.h"

/* I2C device address (AD0 pin selects between these) */
#define MPU6050_ADDR_LOW  (0x68 << 1)   /* AD0 = 0 */
#define MPU6050_ADDR_HIGH (0x69 << 1)   /* AD0 = 1 */

/* Driver-specific handle — base MUST be first */
typedef struct {
    imu_handle_t    base;
    uint8_t         dev_addr;       /* I2C address (shifted for HAL) */
    imu_accel_range_t accel_range;
    imu_gyro_range_t  gyro_range;
} mpu6050_handle_t;

/* Shared vtable for all MPU6050 instances */
extern const imu_driver_t mpu6050_driver;

/*
 * Initialise the handle fields before calling IMU_Init().
 *
 * h        : caller-allocated handle
 * read     : I2C read callback
 * write    : I2C write callback
 * delay    : millisecond delay callback
 * bus_ctx  : I2C_HandleTypeDef * passed through to callbacks
 * dev_addr : MPU6050_ADDR_LOW or MPU6050_ADDR_HIGH
 */
void MPU6050_HandleInit(mpu6050_handle_t  *h,
                        imu_read_fn        read,
                        imu_write_fn       write,
                        imu_delay_fn       delay,
                        void              *bus_ctx,
                        uint8_t            dev_addr,
                        imu_accel_range_t  accel_range,
                        imu_gyro_range_t   gyro_range);

#endif /* INC_MPU6050_DRV_H_ */
