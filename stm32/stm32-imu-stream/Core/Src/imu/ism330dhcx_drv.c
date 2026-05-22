#include "ism330dhcx_drv.h"

/* Register map — identical to LSM6DSO */
#define REG_FUNC_CFG_ACCESS 0x01
#define REG_WHO_AM_I        0x0F   /* returns 0x6B */
#define REG_CTRL1_XL        0x10   /* accel: ODR[7:4] FS[3:2] LPF2_XL_EN[1] */
#define REG_CTRL2_G         0x11   /* gyro:  ODR[7:4] FS[3:2] FS_125[1]      */
#define REG_CTRL3_C         0x12
#define REG_OUTX_L_G        0x22   /* 6 bytes gyro  (L then H per axis) */
#define REG_OUTX_L_A        0x28   /* 6 bytes accel (L then H per axis) */

#define WHO_AM_I_VAL        0x6B   /* ISM330DHCX — differs from LSM6DSO (0x6C) */

/* CTRL1_XL FS[3:2]: 00=±2g  01=±16g  10=±4g  11=±8g */
static const uint8_t accel_fs[] = { 0x00, 0x02, 0x03, 0x01 };

/* CTRL2_G FS[3:2]: 00=250dps  01=500  10=1000  11=2000 */
static const uint8_t gyro_fs[] = { 0x00, 0x01, 0x02, 0x03 };

#define ODR_104HZ 0x04

static inline ism330dhcx_handle_t *to_ism(imu_handle_t *h)
{
    return (ism330dhcx_handle_t *)h;
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

static int8_t ism330dhcx_init(imu_handle_t *h)
{
    uint8_t id;
    if (h->driver->who_am_i(h, &id) != IMU_OK) return IMU_ERR;
    if (id != WHO_AM_I_VAL) return IMU_ERR;

    if (reg_write_byte(h, REG_CTRL3_C, 0x01) != IMU_OK) return IMU_ERR;
    h->delay(10);

    if (reg_write_byte(h, REG_CTRL3_C, 0x44) != IMU_OK) return IMU_ERR;

    if (h->driver->set_accel_range(h, to_ism(h)->accel_range) != IMU_OK) return IMU_ERR;
    if (h->driver->set_gyro_range (h, to_ism(h)->gyro_range)  != IMU_OK) return IMU_ERR;

    return IMU_OK;
}

static int8_t ism330dhcx_deinit(imu_handle_t *h)
{
    if (reg_write_byte(h, REG_CTRL1_XL, 0x00) != IMU_OK) return IMU_ERR;
    return reg_write_byte(h, REG_CTRL2_G, 0x00);
}

static int8_t ism330dhcx_read_raw(imu_handle_t *h, imu_raw_data_t *out)
{
    uint8_t buf[12];
    if (reg_read(h, REG_OUTX_L_G, buf, 12) != IMU_OK) return IMU_ERR;

    out->gyro_x  = (int16_t)((buf[1]  << 8) | buf[0]);
    out->gyro_y  = (int16_t)((buf[3]  << 8) | buf[2]);
    out->gyro_z  = (int16_t)((buf[5]  << 8) | buf[4]);
    out->accel_x = (int16_t)((buf[7]  << 8) | buf[6]);
    out->accel_y = (int16_t)((buf[9]  << 8) | buf[8]);
    out->accel_z = (int16_t)((buf[11] << 8) | buf[10]);

    return IMU_OK;
}

static int8_t ism330dhcx_set_accel_range(imu_handle_t *h, imu_accel_range_t range)
{
    uint8_t val = (uint8_t)((ODR_104HZ << 4) | (accel_fs[range] << 2));
    if (reg_write_byte(h, REG_CTRL1_XL, val) != IMU_OK) return IMU_ERR;
    to_ism(h)->accel_range = range;
    return IMU_OK;
}

static int8_t ism330dhcx_set_gyro_range(imu_handle_t *h, imu_gyro_range_t range)
{
    uint8_t val = (uint8_t)((ODR_104HZ << 4) | (gyro_fs[range] << 2));
    if (reg_write_byte(h, REG_CTRL2_G, val) != IMU_OK) return IMU_ERR;
    to_ism(h)->gyro_range = range;
    return IMU_OK;
}

static int8_t ism330dhcx_who_am_i(imu_handle_t *h, uint8_t *id)
{
    return reg_read(h, REG_WHO_AM_I, id, 1);
}

/* ---------- public API ---------- */

const imu_driver_t ism330dhcx_driver = {
    .init            = ism330dhcx_init,
    .deinit          = ism330dhcx_deinit,
    .read_raw        = ism330dhcx_read_raw,
    .set_accel_range = ism330dhcx_set_accel_range,
    .set_gyro_range  = ism330dhcx_set_gyro_range,
    .who_am_i        = ism330dhcx_who_am_i,
};

void ISM330DHCX_HandleInit(ism330dhcx_handle_t *h,
                            imu_read_fn          read,
                            imu_write_fn         write,
                            imu_delay_fn         delay,
                            void                *bus_ctx,
                            imu_accel_range_t    accel_range,
                            imu_gyro_range_t     gyro_range)
{
    h->base.driver  = &ism330dhcx_driver;
    h->base.read    = read;
    h->base.write   = write;
    h->base.delay   = delay;
    h->base.bus_ctx = bus_ctx;
    h->accel_range  = accel_range;
    h->gyro_range   = gyro_range;
}
