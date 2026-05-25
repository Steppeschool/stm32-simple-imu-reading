#include "lis3mdl_drv.h"

/* Register map */
#define REG_WHO_AM_I  0x0F   /* returns 0x3D */
#define REG_CTRL1     0x20   /* TEMP_EN OM[1:0] DO[2:0] FAST_ODR ST */
#define REG_CTRL2     0x21   /* FS[1:0] REBOOT SOFT_RST */
#define REG_CTRL3     0x22   /* LP SIM MD[1:0] */
#define REG_CTRL4     0x23   /* OMZ[1:0] BLE */
#define REG_CTRL5     0x24   /* FAST_READ BDU */
#define REG_STATUS    0x27
#define REG_OUT_X_L   0x28   /* 6 bytes: X_L X_H Y_L Y_H Z_L Z_H (auto-inc) */

#define WHO_AM_I_VAL  0x3D

/* CTRL1: ultra-high performance (OM=11), 80 Hz (DO=111), no FAST_ODR */
#define CTRL1_DEFAULT 0xFC   /* 1111 1100 */
/* CTRL4: ultra-high performance on Z axis (OMZ=11), little-endian output */
#define CTRL4_DEFAULT 0x0C
/* CTRL5: block data update enabled */
#define CTRL5_DEFAULT 0x40

#define STATUS_ZYXDA  (1 << 3)

static inline lis3mdl_handle_t *to_lis(mag_handle_t *h)
{
    return (lis3mdl_handle_t *)h;
}

static int8_t reg_read(mag_handle_t *h, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return h->read(reg, buf, len, h->bus_ctx);
}

static int8_t reg_write_byte(mag_handle_t *h, uint8_t reg, uint8_t val)
{
    return h->write(reg, &val, 1, h->bus_ctx);
}

/* ---------- vtable implementations ---------- */

static int8_t lis3mdl_init(mag_handle_t *h)
{
    uint8_t id;
    if (h->driver->who_am_i(h, &id) != IMU_OK) return IMU_ERR;
    if (id != WHO_AM_I_VAL) return IMU_ERR;

    /* Soft reset */
    if (reg_write_byte(h, REG_CTRL2, 0x04) != IMU_OK) return IMU_ERR;
    h->delay(10);

    /* FS bits [6:5] of CTRL2 */
    uint8_t ctrl2 = (uint8_t)(to_lis(h)->fs << 5);
    if (reg_write_byte(h, REG_CTRL2, ctrl2)        != IMU_OK) return IMU_ERR;
    if (reg_write_byte(h, REG_CTRL1, CTRL1_DEFAULT) != IMU_OK) return IMU_ERR;
    if (reg_write_byte(h, REG_CTRL4, CTRL4_DEFAULT) != IMU_OK) return IMU_ERR;
    if (reg_write_byte(h, REG_CTRL5, CTRL5_DEFAULT) != IMU_OK) return IMU_ERR;

    /* Continuous conversion mode */
    if (reg_write_byte(h, REG_CTRL3, 0x00) != IMU_OK) return IMU_ERR;

    return IMU_OK;
}

static int8_t lis3mdl_deinit(mag_handle_t *h)
{
    return reg_write_byte(h, REG_CTRL3, 0x03); /* power-down */
}

static int8_t lis3mdl_read_raw(mag_handle_t *h, mag_raw_data_t *out)
{
    uint8_t buf[6];
    /*
     * LIS3MDL requires the MSB of the sub-address to be set for multi-byte
     * reads in both I2C and SPI modes to enable auto-increment.
     * Without this the register pointer stays at OUT_X_L and all six bytes
     * read the same value.
     */
    if (reg_read(h, REG_OUT_X_L | 0x80, buf, 6) != IMU_OK) return IMU_ERR;

    out->x = (int16_t)((buf[1] << 8) | buf[0]);
    out->y = (int16_t)((buf[3] << 8) | buf[2]);
    out->z = (int16_t)((buf[5] << 8) | buf[4]);
    out->y = -out->y;
    out->z = -out->z;
    return IMU_OK;
}

static int8_t lis3mdl_who_am_i(mag_handle_t *h, uint8_t *id)
{
    return reg_read(h, REG_WHO_AM_I, id, 1);
}

/* ---------- public API ---------- */

const mag_driver_t lis3mdl_driver = {
    .init       = lis3mdl_init,
    .deinit     = lis3mdl_deinit,
    .read_raw   = lis3mdl_read_raw,
    .who_am_i   = lis3mdl_who_am_i,
};

void LIS3MDL_HandleInit(lis3mdl_handle_t *h,
                        imu_read_fn       read,
                        imu_write_fn      write,
                        imu_delay_fn      delay,
                        void             *bus_ctx,
                        lis3mdl_fs_t      fs)
{
    h->base.driver  = &lis3mdl_driver;
    h->base.read    = read;
    h->base.write   = write;
    h->base.delay   = delay;
    h->base.bus_ctx = bus_ctx;
    h->fs           = fs;
}
