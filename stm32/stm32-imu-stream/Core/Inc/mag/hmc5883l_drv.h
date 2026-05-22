#ifndef INC_HMC5883L_DRV_H_
#define INC_HMC5883L_DRV_H_

#include "mag_hal.h"

#define HMC5883L_ADDR  (0x1E << 1)   /* fixed I2C address */

typedef enum {
    HMC5883L_GAIN_1370 = 0,   /* ±0.88 Ga, 1370 LSB/Ga */
    HMC5883L_GAIN_1090 = 1,   /* ±1.3  Ga, 1090 LSB/Ga (default) */
    HMC5883L_GAIN_820  = 2,   /* ±1.9  Ga,  820 LSB/Ga */
    HMC5883L_GAIN_660  = 3,   /* ±2.5  Ga,  660 LSB/Ga */
    HMC5883L_GAIN_440  = 4,   /* ±4.0  Ga,  440 LSB/Ga */
    HMC5883L_GAIN_390  = 5,   /* ±4.7  Ga,  390 LSB/Ga */
    HMC5883L_GAIN_330  = 6,   /* ±5.6  Ga,  330 LSB/Ga */
    HMC5883L_GAIN_230  = 7,   /* ±8.1  Ga,  230 LSB/Ga */
} hmc5883l_gain_t;

typedef struct {
    mag_handle_t    base;
    hmc5883l_gain_t gain;
} hmc5883l_handle_t;

extern const mag_driver_t hmc5883l_driver;

void HMC5883L_HandleInit(hmc5883l_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         hmc5883l_gain_t    gain);

#endif /* INC_HMC5883L_DRV_H_ */
