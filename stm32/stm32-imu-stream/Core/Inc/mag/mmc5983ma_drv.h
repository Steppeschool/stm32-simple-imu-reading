#ifndef INC_MMC5983MA_DRV_H_
#define INC_MMC5983MA_DRV_H_

#include "mag_hal.h"

#define MMC5983MA_ADDR  (0x30 << 1)   /* fixed I2C address */

/*
 * Bandwidth controls the measurement duration (and noise floor).
 * Wider BW = faster measurement, higher noise.
 */
typedef enum {
    MMC5983MA_BW_100HZ = 0,   /* 100 Hz, 8 ms */
    MMC5983MA_BW_200HZ = 1,   /* 200 Hz, 4 ms */
    MMC5983MA_BW_400HZ = 2,   /* 400 Hz, 2 ms */
    MMC5983MA_BW_800HZ = 3,   /* 800 Hz, 0.5 ms */
} mmc5983ma_bw_t;

typedef struct {
    mag_handle_t  base;
    mmc5983ma_bw_t bw;
} mmc5983ma_handle_t;

extern const mag_driver_t mmc5983ma_driver;

void MMC5983MA_HandleInit(mmc5983ma_handle_t *h,
                          imu_read_fn         read,
                          imu_write_fn        write,
                          imu_delay_fn        delay,
                          void               *bus_ctx,
                          mmc5983ma_bw_t      bw);

#endif /* INC_MMC5983MA_DRV_H_ */
