#include "icm42688_drv.h"

/* Bank 0 registers */
#define REG_DEVICE_CONFIG     0x11
#define REG_SIGNAL_PATH_RESET 0x4B
#define REG_INTF_CONFIG0      0x4C
#define REG_PWR_MGMT0         0x4E
#define REG_GYRO_CONFIG0      0x4F
#define REG_ACCEL_CONFIG0     0x50
#define REG_ACCEL_DATA_X1     0x1F   /* 6 bytes accel then 6 bytes gyro, big-endian */
#define REG_GYRO_DATA_X1      0x25
#define REG_WHO_AM_I          0x75
#define REG_BANK_SEL          0x76

#define WHO_AM_I_VAL          0x47

/*
 * GYRO_CONFIG0 bits [7:5]: FS_SEL — 000=2000dps, 001=1000, 010=500, 011=250
 * ACCEL_CONFIG0 bits [7:5]: FS_SEL — 000=16g, 001=8g, 010=4g, 011=2g
 *
 * Our enum order: 250, 500, 1000, 2000 → reg values: 3, 2, 1, 0
 * Our enum order: 2g, 4g, 8g, 16g     → reg values: 3, 2, 1, 0
 */
static const uint8_t gyro_fs[]  = { 0x03, 0x02, 0x01, 0x00 }; /* 250 500 1000 2000 */
static const uint8_t accel_fs[] = { 0x03, 0x02, 0x01, 0x00 }; /* 2g 4g 8g 16g */

/* Default ODR: 1 kHz → bits [3:0] = 0x06 */
#define ODR_1KHZ  0x06

static inline icm42688_handle_t *to_icm(imu_handle_t *h)
{
    return (icm42688_handle_t *)h;
}

static int8_t reg_read(imu_handle_t *h, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return h->read(reg, buf, len, h->bus_ctx);
}

static int8_t reg_write_byte(imu_handle_t *h, uint8_t reg, uint8_t val)
{
    return h->write(reg, &val, 1, h->bus_ctx);
}

/* ---------- vtable implementations ---------- */

static int8_t icm42688_init(imu_handle_t *h)
{
    uint8_t id;
    if (h->driver->who_am_i(h, &id) != IMU_OK) return IMU_ERR;
    if (id != WHO_AM_I_VAL) return IMU_ERR;

    /* Soft reset */
    if (reg_write_byte(h, REG_DEVICE_CONFIG, 0x01) != IMU_OK) return IMU_ERR;
    h->delay(10);

    /* Enable accel + gyro in low-noise mode */
    if (reg_write_byte(h, REG_PWR_MGMT0, 0x0F) != IMU_OK) return IMU_ERR;
    h->delay(1);

    if (h->driver->set_accel_range(h, to_icm(h)->accel_range) != IMU_OK) return IMU_ERR;
    if (h->driver->set_gyro_range (h, to_icm(h)->gyro_range)  != IMU_OK) return IMU_ERR;

    return IMU_OK;
}

static int8_t icm42688_deinit(imu_handle_t *h)
{
    /* Accel off, gyro off */
    return reg_write_byte(h, REG_PWR_MGMT0, 0x00);
}

static int8_t icm42688_read_raw(imu_handle_t *h, imu_raw_data_t *out)
{
    uint8_t buf[12];
    if (reg_read(h, REG_ACCEL_DATA_X1, buf, 12) != IMU_OK) return IMU_ERR;

    out->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    out->gyro_x  = (int16_t)((buf[6]  << 8) | buf[7]);
    out->gyro_y  = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_z  = (int16_t)((buf[10] << 8) | buf[11]);

    return IMU_OK;
}

static int8_t icm42688_set_accel_range(imu_handle_t *h, imu_accel_range_t range)
{
    uint8_t val = (uint8_t)((accel_fs[range] << 5) | ODR_1KHZ);
    if (reg_write_byte(h, REG_ACCEL_CONFIG0, val) != IMU_OK) return IMU_ERR;
    to_icm(h)->accel_range = range;
    return IMU_OK;
}

static int8_t icm42688_set_gyro_range(imu_handle_t *h, imu_gyro_range_t range)
{
    uint8_t val = (uint8_t)((gyro_fs[range] << 5) | ODR_1KHZ);
    if (reg_write_byte(h, REG_GYRO_CONFIG0, val) != IMU_OK) return IMU_ERR;
    to_icm(h)->gyro_range = range;
    return IMU_OK;
}

static int8_t icm42688_who_am_i(imu_handle_t *h, uint8_t *id)
{
    return reg_read(h, REG_WHO_AM_I, id, 1);
}

/* ---------- public API ---------- */

const imu_driver_t icm42688_driver = {
    .init            = icm42688_init,
    .deinit          = icm42688_deinit,
    .read_raw        = icm42688_read_raw,
    .set_accel_range = icm42688_set_accel_range,
    .set_gyro_range  = icm42688_set_gyro_range,
    .who_am_i        = icm42688_who_am_i,
};

void ICM42688_HandleInit(icm42688_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         imu_accel_range_t  accel_range,
                         imu_gyro_range_t   gyro_range)
{
    h->base.driver      = &icm42688_driver;
    h->base.read        = read;
    h->base.write       = write;
    h->base.delay       = delay;
    h->base.bus_ctx     = bus_ctx;
    h->base.gyro_bias_x = 0;
    h->base.gyro_bias_y = 0;
    h->base.gyro_bias_z = 0;
    h->accel_range      = accel_range;
    h->gyro_range       = gyro_range;
}
