#ifndef INC_IMU_HAL_H_
#define INC_IMU_HAL_H_

#include <stdint.h>

/* Return codes */
#define IMU_OK      0
#define IMU_ERR    -1

/* Common range enums — all four sensors support these values */
typedef enum {
    IMU_ACCEL_RANGE_2G  = 0,
    IMU_ACCEL_RANGE_4G  = 1,
    IMU_ACCEL_RANGE_8G  = 2,
    IMU_ACCEL_RANGE_16G = 3,
} imu_accel_range_t;

typedef enum {
    IMU_GYRO_RANGE_250DPS  = 0,
    IMU_GYRO_RANGE_500DPS  = 1,
    IMU_GYRO_RANGE_1000DPS = 2,
    IMU_GYRO_RANGE_2000DPS = 3,
} imu_gyro_range_t;

/* Raw 16-bit counts from the sensor registers */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_raw_data_t;

/*
 * Bus callbacks — implemented by the application for SPI or I2C.
 *
 * reg  : register address
 * buf  : data buffer
 * len  : number of bytes
 * ctx  : user context (e.g. SPI_HandleTypeDef* or I2C_HandleTypeDef*)
 *
 * Returns IMU_OK on success, IMU_ERR on failure.
 */
typedef int8_t (*imu_read_fn) (uint8_t reg, uint8_t *buf, uint16_t len, void *ctx);
typedef int8_t (*imu_write_fn)(uint8_t reg, const uint8_t *buf, uint16_t len, void *ctx);
typedef void   (*imu_delay_fn)(uint32_t ms);

/*
 * Base handle — must be the first member of every driver-specific handle struct
 * so that a pointer to the driver handle can be safely cast to imu_handle_t *.
 */
typedef struct imu_handle_s imu_handle_t;

/* Driver vtable — each sensor driver exposes one of these as a const object */
typedef struct {
    int8_t (*init)           (imu_handle_t *h);
    int8_t (*deinit)         (imu_handle_t *h);
    int8_t (*read_raw)       (imu_handle_t *h, imu_raw_data_t *out);
    int8_t (*set_accel_range)(imu_handle_t *h, imu_accel_range_t range);
    int8_t (*set_gyro_range) (imu_handle_t *h, imu_gyro_range_t  range);
    int8_t (*who_am_i)       (imu_handle_t *h, uint8_t *id);
} imu_driver_t;

struct imu_handle_s {
    const imu_driver_t *driver;
    imu_read_fn         read;
    imu_write_fn        write;
    imu_delay_fn        delay;
    void               *bus_ctx;
};

/* Inline wrappers so call sites don't have to dereference the vtable manually */
static inline int8_t IMU_Init(imu_handle_t *h)
    { return h->driver->init(h); }

static inline int8_t IMU_Deinit(imu_handle_t *h)
    { return h->driver->deinit(h); }

static inline int8_t IMU_ReadRaw(imu_handle_t *h, imu_raw_data_t *out)
    { return h->driver->read_raw(h, out); }

static inline int8_t IMU_SetAccelRange(imu_handle_t *h, imu_accel_range_t r)
    { return h->driver->set_accel_range(h, r); }

static inline int8_t IMU_SetGyroRange(imu_handle_t *h, imu_gyro_range_t r)
    { return h->driver->set_gyro_range(h, r); }

static inline int8_t IMU_WhoAmI(imu_handle_t *h, uint8_t *id)
    { return h->driver->who_am_i(h, id); }

#endif /* INC_IMU_HAL_H_ */
