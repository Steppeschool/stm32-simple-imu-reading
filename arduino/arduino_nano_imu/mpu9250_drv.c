#include "mpu9250_drv.h"

/* -------------------------------------------------------------------------
 * MPU9250 register map
 * ------------------------------------------------------------------------- */
#define REG_WHO_AM_I      0x75   /* 0x71 = MPU9250, 0x73 = MPU9255 */
#define REG_CONFIG        0x1A   /* DLPF_CFG[2:0] */
#define REG_GYRO_CONFIG   0x1B   /* FS_SEL[4:3] */
#define REG_ACCEL_CONFIG  0x1C   /* FS_SEL[4:3] */
#define REG_ACCEL_CFG2    0x1D   /* A_DLPF_CFG[2:0] */
#define REG_ACCEL_XOUT_H  0x3B   /* 14 bytes: accel(6) + temp(2) + gyro(6) */
#define REG_INT_PIN_CFG   0x37   /* bit1 = BYPASS_EN */
#define REG_USER_CTRL     0x6A   /* I2C_MST_EN = bit5 */
#define REG_PWR_MGMT_1    0x6B   /* DEVICE_RESET=bit7, CLKSEL[2:0] */

#define WHO_AM_I_9250     0x71
#define WHO_AM_I_9255     0x73

/*
 * Accel FS_SEL [4:3]: 0=+-2g  1=+-4g  2=+-8g  3=+-16g  -> val = range << 3
 * Gyro  FS_SEL [4:3]: 0=250   1=500   2=1000  3=2000dps -> val = range << 3
 */

/* -------------------------------------------------------------------------
 * AK8963 register map (direct I2C access via bypass, address 0x0C)
 * ------------------------------------------------------------------------- */
#define AK_REG_WIA    0x00   /* WHO_AM_I -> 0x48 */
#define AK_REG_HXL    0x03   /* 7 bytes: HXL HXH HYL HYH HZL HZH ST2 */
#define AK_REG_CNTL1  0x0A   /* operating mode + output bit width */
#define AK_REG_CNTL2  0x0B   /* SRST bit[0] = soft reset */

#define AK_WHO_AM_I_VAL      0x48
#define AK_CNTL1_16BIT_100HZ 0x16   /* 16-bit continuous mode 2 = 100 Hz */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static inline mpu9250_handle_t *to_mpu(imu_handle_t *h)
{
    return (mpu9250_handle_t *)h;
}

static int8_t reg_read(imu_handle_t *h, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return h->read(reg, buf, len, h->bus_ctx);
}

static int8_t reg_write_byte(imu_handle_t *h, uint8_t reg, uint8_t val)
{
    return h->write(reg, &val, 1, h->bus_ctx);
}

/* -------------------------------------------------------------------------
 * IMU vtable
 * ------------------------------------------------------------------------- */

static int8_t mpu9250_init(imu_handle_t *base)
{
    mpu9250_handle_t *h = to_mpu(base);
    uint8_t id, uctrl;

    if (base->driver->who_am_i(base, &id) != IMU_OK) return IMU_ERR;
    if (id != WHO_AM_I_9250 && id != WHO_AM_I_9255)  return IMU_ERR;

    /* Full device reset */
    if (reg_write_byte(base, REG_PWR_MGMT_1, 0x80) != IMU_OK) return IMU_ERR;
    base->delay(100);

    /* Wake up — select PLL clock for best accuracy */
    if (reg_write_byte(base, REG_PWR_MGMT_1, 0x01) != IMU_OK) return IMU_ERR;
    base->delay(10);

    /* Gyro DLPF ~41 Hz bandwidth */
    if (reg_write_byte(base, REG_CONFIG,     0x03) != IMU_OK) return IMU_ERR;
    /* Accel DLPF ~41 Hz bandwidth */
    if (reg_write_byte(base, REG_ACCEL_CFG2, 0x03) != IMU_OK) return IMU_ERR;

    if (base->driver->set_accel_range(base, h->accel_range) != IMU_OK) return IMU_ERR;
    if (base->driver->set_gyro_range (base, h->gyro_range)  != IMU_OK) return IMU_ERR;

    /*
     * Disable I2C master, enable bypass so AK8963 appears directly on the
     * host I2C bus at address 0x0C.
     */
    if (reg_read(base, REG_USER_CTRL, &uctrl, 1) != IMU_OK) return IMU_ERR;
    if (reg_write_byte(base, REG_USER_CTRL,   uctrl & ~0x20) != IMU_OK) return IMU_ERR;
    if (reg_write_byte(base, REG_INT_PIN_CFG, 0x02)          != IMU_OK) return IMU_ERR;
    base->delay(10);

    /* Init AK8963 directly */
    if (MAG_Init(&h->mag) != IMU_OK) return IMU_ERR;

    return IMU_OK;
}

static int8_t mpu9250_deinit(imu_handle_t *base)
{
    return reg_write_byte(base, REG_PWR_MGMT_1, 0x40); /* sleep mode */
}

static int8_t mpu9250_read_raw(imu_handle_t *base, imu_raw_data_t *out)
{
    uint8_t buf[14];
    if (reg_read(base, REG_ACCEL_XOUT_H, buf, 14) != IMU_OK) return IMU_ERR;

    /* MPU9250 is big-endian (high byte first) */
    out->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature, skipped */
    out->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return IMU_OK;
}

static int8_t mpu9250_set_accel_range(imu_handle_t *base, imu_accel_range_t range)
{
    uint8_t val = (uint8_t)(range << 3);
    if (reg_write_byte(base, REG_ACCEL_CONFIG, val) != IMU_OK) return IMU_ERR;
    to_mpu(base)->accel_range = range;
    return IMU_OK;
}

static int8_t mpu9250_set_gyro_range(imu_handle_t *base, imu_gyro_range_t range)
{
    uint8_t val = (uint8_t)(range << 3);
    if (reg_write_byte(base, REG_GYRO_CONFIG, val) != IMU_OK) return IMU_ERR;
    to_mpu(base)->gyro_range = range;
    return IMU_OK;
}

static int8_t mpu9250_who_am_i(imu_handle_t *base, uint8_t *id)
{
    return reg_read(base, REG_WHO_AM_I, id, 1);
}

/* -------------------------------------------------------------------------
 * AK8963 vtable — bypass mode: m->bus_ctx is the AK8963 I2C context (0x0C)
 * ------------------------------------------------------------------------- */

static int8_t ak8963_init(mag_handle_t *m)
{
    uint8_t val;

    /* Soft reset */
    val = 0x01;
    if (m->write(AK_REG_CNTL2, &val, 1, m->bus_ctx) != IMU_OK) return IMU_ERR;
    m->delay(10);

    /* Power down before mode change (datasheet requirement) */
    val = 0x00;
    if (m->write(AK_REG_CNTL1, &val, 1, m->bus_ctx) != IMU_OK) return IMU_ERR;
    m->delay(10);

    /* 16-bit continuous measurement at 100 Hz */
    val = AK_CNTL1_16BIT_100HZ;
    if (m->write(AK_REG_CNTL1, &val, 1, m->bus_ctx) != IMU_OK) return IMU_ERR;
    m->delay(10);

    return IMU_OK;
}

static int8_t ak8963_deinit(mag_handle_t *m)
{
    uint8_t val = 0x00;
    return m->write(AK_REG_CNTL1, &val, 1, m->bus_ctx); /* power down */
}

static int8_t ak8963_read_raw(mag_handle_t *m, mag_raw_data_t *out)
{
    uint8_t buf[7];  /* HXL HXH HYL HYH HZL HZH ST2 */

    if (m->read(AK_REG_HXL, buf, 7, m->bus_ctx) != IMU_OK) return IMU_ERR;

    /* AK8963 is little-endian (low byte first) */
    out->y = -(int16_t)((buf[1] << 8) | buf[0]);
    out->x = (int16_t)((buf[3] << 8) | buf[2]);
    out->z = (int16_t)((buf[5] << 8) | buf[4]);

    return IMU_OK;
}

static int8_t ak8963_who_am_i(mag_handle_t *m, uint8_t *id)
{
    return m->read(AK_REG_WIA, id, 1, m->bus_ctx);
}

/* -------------------------------------------------------------------------
 * Public vtables
 * ------------------------------------------------------------------------- */

const imu_driver_t mpu9250_imu_driver = {
    .init            = mpu9250_init,
    .deinit          = mpu9250_deinit,
    .read_raw        = mpu9250_read_raw,
    .set_accel_range = mpu9250_set_accel_range,
    .set_gyro_range  = mpu9250_set_gyro_range,
    .who_am_i        = mpu9250_who_am_i,
};

const mag_driver_t mpu9250_mag_driver = {
    .init       = ak8963_init,
    .deinit     = ak8963_deinit,
    .read_raw   = ak8963_read_raw,
    .who_am_i   = ak8963_who_am_i,
};

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void MPU9250_HandleInit(mpu9250_handle_t *h,
                        imu_read_fn       read,
                        imu_write_fn      write,
                        imu_delay_fn      delay,
                        void             *bus_ctx,
                        void             *ak_bus_ctx,
                        imu_accel_range_t accel_range,
                        imu_gyro_range_t  gyro_range)
{
    /* IMU base — talks to MPU9250 at bus_ctx (0x68) */
    h->base.driver      = &mpu9250_imu_driver;
    h->base.read        = read;
    h->base.write       = write;
    h->base.delay       = delay;
    h->base.bus_ctx     = bus_ctx;
    h->base.gyro_bias_x = 0;
    h->base.gyro_bias_y = 0;
    h->base.gyro_bias_z = 0;
    h->accel_range      = accel_range;
    h->gyro_range       = gyro_range;

    /*
     * Mag handle — talks directly to AK8963 at ak_bus_ctx (0x0C).
     * Bypass mode is enabled by mpu9250_init before MAG_Init is called,
     * so the AK8963 is already visible on the host bus when ak8963_init runs.
     */
    h->mag.driver  = &mpu9250_mag_driver;
    h->mag.read    = read;
    h->mag.write   = write;
    h->mag.delay   = delay;
    h->mag.bus_ctx = ak_bus_ctx;
    h->mag.bias_x  = 0;
    h->mag.bias_y  = 0;
    h->mag.bias_z  = 0;
}
