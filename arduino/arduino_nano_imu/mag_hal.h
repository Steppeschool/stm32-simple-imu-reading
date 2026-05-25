#ifndef INC_MAG_HAL_H_
#define INC_MAG_HAL_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "imu_hal.h"   /* reuses imu_read_fn / imu_write_fn / imu_delay_fn */

/* Raw 16-bit counts from the sensor registers */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mag_raw_data_t;

/*
 * Base handle â€” must be the first member of every driver-specific mag handle.
 *
 * Built-in magnetometers (e.g. AK09916 inside ICM-20948):
 *   The IMU driver allocates a mag_handle_t inside its own handle struct and
 *   exposes a pointer to it so the caller can call MAG_ReadRaw() on it
 *   independently.
 *
 * Standalone magnetometers:
 *   Caller allocates the driver-specific handle (e.g. lis3mdl_handle_t),
 *   calls the driver's HandleInit(), then MAG_Init().
 */
typedef struct mag_handle_s mag_handle_t;

typedef struct {
    int8_t (*init)    (mag_handle_t *h);
    int8_t (*deinit)  (mag_handle_t *h);
    int8_t (*read_raw)(mag_handle_t *h, mag_raw_data_t *out);
    int8_t (*who_am_i)(mag_handle_t *h, uint8_t *id);
} mag_driver_t;

struct mag_handle_s {
    const mag_driver_t *driver;
    imu_read_fn         read;
    imu_write_fn        write;
    imu_delay_fn        delay;
    void               *bus_ctx;
    int16_t             bias_x;
    int16_t             bias_y;
    int16_t             bias_z;
};

static inline int8_t MAG_Init(mag_handle_t *h)
    { return h->driver->init(h); }

static inline int8_t MAG_Deinit(mag_handle_t *h)
    { return h->driver->deinit(h); }

static inline int8_t MAG_ReadRaw(mag_handle_t *h, mag_raw_data_t *out)
    { return h->driver->read_raw(h, out); }

static inline int8_t MAG_WhoAmI(mag_handle_t *h, uint8_t *id)
    { return h->driver->who_am_i(h, id); }

static inline void MAG_SetBias(mag_handle_t *h,
                                int16_t bx, int16_t by, int16_t bz)
{
    h->bias_x = bx;
    h->bias_y = by;
    h->bias_z = bz;
}

static inline int8_t MAG_ReadCorrected(mag_handle_t *h, mag_raw_data_t *out)
{
    int8_t ret = h->driver->read_raw(h, out);
    if (ret == IMU_OK) {
        out->x -= h->bias_x;
        out->y -= h->bias_y;
        out->z -= h->bias_z;
    }
    return ret;
}


#ifdef __cplusplus
}
#endif

#endif /* INC_MAG_HAL_H_ */
