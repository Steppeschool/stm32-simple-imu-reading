#ifndef INC_LIS3MDL_DRV_H_
#define INC_LIS3MDL_DRV_H_

#include "mag_hal.h"

/* SA1 pin selects address */
#define LIS3MDL_ADDR_SA1_LOW  (0x1C << 1)
#define LIS3MDL_ADDR_SA1_HIGH (0x1E << 1)

typedef enum {
    LIS3MDL_FS_4G  = 0,   /* ±4  Gauss, 6842 LSB/Ga */
    LIS3MDL_FS_8G  = 1,   /* ±8  Gauss, 3421 LSB/Ga */
    LIS3MDL_FS_12G = 2,   /* ±12 Gauss, 2281 LSB/Ga */
    LIS3MDL_FS_16G = 3,   /* ±16 Gauss, 1711 LSB/Ga */
} lis3mdl_fs_t;

typedef struct {
    mag_handle_t base;
    lis3mdl_fs_t fs;
} lis3mdl_handle_t;

extern const mag_driver_t lis3mdl_driver;

void LIS3MDL_HandleInit(lis3mdl_handle_t *h,
                        imu_read_fn       read,
                        imu_write_fn      write,
                        imu_delay_fn      delay,
                        void             *bus_ctx,
                        lis3mdl_fs_t      fs);

#endif /* INC_LIS3MDL_DRV_H_ */
