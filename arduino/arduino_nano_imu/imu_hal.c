#include "imu_hal.h"

/*
 * IMU_CalibrateGyro
 *
 * Generic gyro bias calibration that works for any IMU driver.
 * Calls h->driver->read_raw() (the sensor-specific function) to collect
 * num_samples readings, averages them, and stores the result in the base
 * handle's gyro_bias fields.
 *
 * IMU_ReadRaw() subtracts the bias automatically on every subsequent call,
 * so no changes are needed at the call site after calibration.
 */
int8_t IMU_CalibrateGyro(imu_handle_t *h, uint16_t num_samples)
{
    imu_raw_data_t raw;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    /* Zero bias so read_raw() returns uncorrected values during collection */
    h->gyro_bias_x = 0;
    h->gyro_bias_y = 0;
    h->gyro_bias_z = 0;

    h->delay(100); /* let sensor settle */

    for (uint16_t i = 0; i < num_samples; i++) {
        /* read_raw is the sensor-specific function -- each driver implements it */
        if (h->driver->read_raw(h, &raw) != IMU_OK) return IMU_ERR;
        sum_x += raw.gyro_x;
        sum_y += raw.gyro_y;
        sum_z += raw.gyro_z;
        h->delay(10);
    }

    h->gyro_bias_x = (int16_t)(sum_x / num_samples);
    h->gyro_bias_y = (int16_t)(sum_y / num_samples);
    h->gyro_bias_z = (int16_t)(sum_z / num_samples);

    return IMU_OK;
}
