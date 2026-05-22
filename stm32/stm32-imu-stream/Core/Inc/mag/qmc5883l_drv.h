#ifndef INC_QMC5883L_DRV_H_
#define INC_QMC5883L_DRV_H_

#include "mag_hal.h"

#define QMC5883L_ADDR  (0x0D << 1)   /* fixed I2C address */

typedef enum {
    QMC5883L_RNG_2G = 0,   /* ±2 Gauss */
    QMC5883L_RNG_8G = 1,   /* ±8 Gauss */
} qmc5883l_range_t;

typedef enum {
    QMC5883L_ODR_10HZ  = 0,
    QMC5883L_ODR_50HZ  = 1,
    QMC5883L_ODR_100HZ = 2,
    QMC5883L_ODR_200HZ = 3,
} qmc5883l_odr_t;

typedef struct {
    mag_handle_t    base;
    qmc5883l_range_t range;
    qmc5883l_odr_t   odr;
} qmc5883l_handle_t;

extern const mag_driver_t qmc5883l_driver;

void QMC5883L_HandleInit(qmc5883l_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         qmc5883l_range_t   range,
                         qmc5883l_odr_t     odr);

#endif /* INC_QMC5883L_DRV_H_ */
