#include "icm20948_drv.h"

/* -------------------------------------------------------------------------
 * Register map — bank 0
 * ------------------------------------------------------------------------- */
#define B0_WHO_AM_I          0x00   /* returns 0xEA */
#define B0_USER_CTRL         0x03
#define B0_LP_CONFIG         0x05
#define B0_PWR_MGMT_1        0x06
#define B0_REG_BANK_SEL      0x7F   /* global — accessible from all banks */
#define B0_ACCEL_XOUT_H      0x2D   /* 12-byte burst: 6 accel + 6 gyro    */
#define B0_EXT_SLV_SENS_DATA_00  0x3B   /* 8 bytes: AK09916 HXL..ST2      */

/* Bank 2 */
#define B2_GYRO_SMPLRT_DIV   0x00
#define B2_GYRO_CONFIG_1     0x01
#define B2_GYRO_CONFIG_2     0x02
#define B2_XG_OFFS_USRH      0x03
#define B2_XG_OFFS_USRL      0x04
#define B2_YG_OFFS_USRH      0x05
#define B2_YG_OFFS_USRL      0x06
#define B2_ZG_OFFS_USRH      0x07
#define B2_ZG_OFFS_USRL      0x08
#define B2_ACCEL_SMPLRT_DIV_1 0x10
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_CONFIG      0x14

/* Bank 3 — I2C master */
#define B3_I2C_MST_ODR_CONFIG 0x00
#define B3_I2C_MST_CTRL       0x01
#define B3_I2C_MST_DELAY_CTRL 0x02
#define B3_I2C_SLV0_ADDR      0x03
#define B3_I2C_SLV0_REG       0x04
#define B3_I2C_SLV0_CTRL      0x05
#define B3_I2C_SLV0_DO        0x06

#define WHO_AM_I_VAL         0xEA

/* AK09916 I2C address and registers */
#define AK09916_I2C_ADDR     0x0C
#define AK_MAG_DATA_ONSET    0x11   /* HXL – first data register */
#define AK_CTRL2             0x31
#define AK_CTRL3             0x32

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static inline icm20948_handle_t *to_icm(imu_handle_t *h)
{
    return (icm20948_handle_t *)h;
}

/* Switch register bank when needed.  REG_BANK_SEL is bank-agnostic. */
static int8_t set_bank(icm20948_handle_t *h, uint8_t bank)
{
    if (h->current_bank == bank) return IMU_OK;
    uint8_t val = (uint8_t)(bank << 4);
    if (h->base.write(B0_REG_BANK_SEL, &val, 1, h->base.bus_ctx) != IMU_OK)
        return IMU_ERR;
    h->current_bank = bank;
    return IMU_OK;
}

static int8_t write_reg(icm20948_handle_t *h, uint8_t bank, uint8_t reg, uint8_t val)
{
    if (set_bank(h, bank) != IMU_OK) return IMU_ERR;
    return h->base.write(reg, &val, 1, h->base.bus_ctx);
}

static int8_t read_reg(icm20948_handle_t *h, uint8_t bank,
                       uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (set_bank(h, bank) != IMU_OK) return IMU_ERR;
    return h->base.read(reg, buf, len, h->base.bus_ctx);
}

/* -------------------------------------------------------------------------
 * AK09916 access through ICM I2C master
 * ------------------------------------------------------------------------- */

static int8_t ak09916_write(icm20948_handle_t *h, uint8_t reg, uint8_t val)
{
    if (write_reg(h, 3, B3_I2C_SLV0_ADDR, AK09916_I2C_ADDR)  != IMU_OK) return IMU_ERR;
    if (write_reg(h, 3, B3_I2C_SLV0_REG,  reg)                != IMU_OK) return IMU_ERR;
    if (write_reg(h, 3, B3_I2C_SLV0_DO,   val)                != IMU_OK) return IMU_ERR;
    h->base.delay(50);
    if (write_reg(h, 3, B3_I2C_SLV0_CTRL, 0x80 | 0x01)       != IMU_OK) return IMU_ERR;
    h->base.delay(50);
    return IMU_OK;
}

/* Configure SLV0 for continuous read of `len` bytes starting at `onset_reg` */
static int8_t ak09916_setup_read(icm20948_handle_t *h, uint8_t onset_reg, uint8_t len)
{
    if (write_reg(h, 3, B3_I2C_SLV0_ADDR, 0x80 | AK09916_I2C_ADDR) != IMU_OK) return IMU_ERR;
    if (write_reg(h, 3, B3_I2C_SLV0_REG,  onset_reg)                != IMU_OK) return IMU_ERR;
    h->base.delay(50);
    if (write_reg(h, 3, B3_I2C_SLV0_CTRL, 0x80 | len)               != IMU_OK) return IMU_ERR;
    h->base.delay(50);
    return IMU_OK;
}

/* -------------------------------------------------------------------------
 * IMU vtable
 * ------------------------------------------------------------------------- */

static int8_t icm20948_init(imu_handle_t *base)
{
    icm20948_handle_t *h = to_icm(base);
    uint8_t id;

    h->current_bank = 0xFF; /* force bank switch on first access */

    if (base->driver->who_am_i(base, &id) != IMU_OK) return IMU_ERR;
    if (id != WHO_AM_I_VAL) return IMU_ERR;

    /* Reset, then wake with best-available clock */
    if (write_reg(h, 0, B0_PWR_MGMT_1, 0xC1) != IMU_OK) return IMU_ERR;
    base->delay(100);
    if (write_reg(h, 0, B0_PWR_MGMT_1, 0x01) != IMU_OK) return IMU_ERR;

    /* SPI-only mode — disable I2C interface when using SPI to prevent bus conflicts */
    if (h->spi_mode)
        if (write_reg(h, 0, B0_USER_CTRL, 0x10) != IMU_OK) return IMU_ERR;

    /* Accel: max sample rate, LPF on */
    if (write_reg(h, 2, B2_ACCEL_SMPLRT_DIV_1, 0x00) != IMU_OK) return IMU_ERR;
    if (write_reg(h, 2, B2_ACCEL_SMPLRT_DIV_2, 0x00) != IMU_OK) return IMU_ERR;

    /* Gyro: max sample rate, LPF on */
    if (write_reg(h, 2, B2_GYRO_SMPLRT_DIV, 0x00) != IMU_OK) return IMU_ERR;

    if (base->driver->set_accel_range(base, h->accel_range) != IMU_OK) return IMU_ERR;
    if (base->driver->set_gyro_range (base, h->gyro_range)  != IMU_OK) return IMU_ERR;

    /* Initialise AK09916 magnetometer */
    if (MAG_Init(&h->mag) != IMU_OK) return IMU_ERR;

    if (set_bank(h, 0) != IMU_OK) return IMU_ERR;
    return IMU_OK;
}

static int8_t icm20948_deinit(imu_handle_t *base)
{
    icm20948_handle_t *h = to_icm(base);
    return write_reg(h, 0, B0_PWR_MGMT_1, 0x41); /* sleep + disable temp */
}

static int8_t icm20948_read_raw(imu_handle_t *base, imu_raw_data_t *out)
{
    icm20948_handle_t *h = to_icm(base);
    uint8_t buf[12];

    if (read_reg(h, 0, B0_ACCEL_XOUT_H, buf, 12) != IMU_OK) return IMU_ERR;

    out->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    out->gyro_x  = (int16_t)((buf[6]  << 8) | buf[7]);
    out->gyro_y  = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_z  = (int16_t)((buf[10] << 8) | buf[11]);

    return IMU_OK;
}

static int8_t icm20948_set_accel_range(imu_handle_t *base, imu_accel_range_t range)
{
    icm20948_handle_t *h = to_icm(base);
    /* ACCEL_CONFIG: range at bits [2:1], LPF enable at bit [0] */
    uint8_t val = (uint8_t)((range << 1) | 0x01);
    if (write_reg(h, 2, B2_ACCEL_CONFIG, val) != IMU_OK) return IMU_ERR;
    h->accel_range = range;
    return IMU_OK;
}

static int8_t icm20948_set_gyro_range(imu_handle_t *base, imu_gyro_range_t range)
{
    icm20948_handle_t *h = to_icm(base);
    /* GYRO_CONFIG_1: range at bits [2:1], LPF enable at bit [0] */
    uint8_t val = (uint8_t)((range << 1) | 0x01);
    if (write_reg(h, 2, B2_GYRO_CONFIG_1, val) != IMU_OK) return IMU_ERR;
    h->gyro_range = range;
    return IMU_OK;
}

static int8_t icm20948_who_am_i(imu_handle_t *base, uint8_t *id)
{
    icm20948_handle_t *h = to_icm(base);
    return read_reg(h, 0, B0_WHO_AM_I, id, 1);
}

/* -------------------------------------------------------------------------
 * AK09916 (mag) vtable — bus_ctx points to the parent icm20948_handle_t
 * ------------------------------------------------------------------------- */

static int8_t ak09916_init(mag_handle_t *m)
{
    icm20948_handle_t *h = (icm20948_handle_t *)m->bus_ctx;

    /* Enable I2C master */
    uint8_t uctrl;
    if (read_reg(h, 0, B0_USER_CTRL, &uctrl, 1) != IMU_OK) return IMU_ERR;
    uctrl |= 0x02; /* I2C master reset */
    if (write_reg(h, 0, B0_USER_CTRL, uctrl) != IMU_OK) return IMU_ERR;
    h->base.delay(100);

    uctrl |= 0x20; /* I2C master enable */
    if (write_reg(h, 0, B0_USER_CTRL, uctrl) != IMU_OK) return IMU_ERR;
    h->base.delay(10);

    /* I2C master clock: 400 kHz */
    if (write_reg(h, 3, B3_I2C_MST_CTRL, 0x07) != IMU_OK) return IMU_ERR;
    h->base.delay(10);

    /* ODR: I2C master drives rate */
    if (write_reg(h, 0, B0_LP_CONFIG, 0x40) != IMU_OK) return IMU_ERR;
    h->base.delay(10);

    /* I2C master ODR: ~136 Hz (1.1kHz / 2^3) */
    if (write_reg(h, 3, B3_I2C_MST_ODR_CONFIG, 0x03) != IMU_OK) return IMU_ERR;
    h->base.delay(10);

    /* Delay shadowed external sensor data */
    if (write_reg(h, 3, B3_I2C_MST_DELAY_CTRL, 0x80) != IMU_OK) return IMU_ERR;
    h->base.delay(10);

    /* AK09916 soft reset, then continuous mode 4 (100 Hz) */
    if (ak09916_write(h, AK_CTRL3, 0x01) != IMU_OK) return IMU_ERR;
    h->base.delay(100);
    if (ak09916_write(h, AK_CTRL2, 0x08) != IMU_OK) return IMU_ERR;

    /* Configure SLV0 to continuously shadow 8 bytes from AK09916 */
    if (ak09916_setup_read(h, AK_MAG_DATA_ONSET, 8) != IMU_OK) return IMU_ERR;


    return IMU_OK;
}

static int8_t ak09916_deinit(mag_handle_t *m)
{
    icm20948_handle_t *h = (icm20948_handle_t *)m->bus_ctx;
    return ak09916_write(h, AK_CTRL2, 0x00); /* power-down */
}

static int8_t ak09916_read_raw(mag_handle_t *m, mag_raw_data_t *out)
{
    icm20948_handle_t *h = (icm20948_handle_t *)m->bus_ctx;
    uint8_t buf[8];

    /* EXT_SLV_SENS_DATA_00..07 shadow HXL..ST2 from AK09916 */
    if (read_reg(h, 0, B0_EXT_SLV_SENS_DATA_00, buf, 8) != IMU_OK) return IMU_ERR;

    /* AK09916 data is little-endian */
    out->x = (int16_t)((buf[1] << 8) | buf[0]);
    out->y = (int16_t)((buf[3] << 8) | buf[2]);
    out->z = (int16_t)((buf[5] << 8) | buf[4]);
    /* buf[6] = TMPS, buf[7] = ST2 (data latch release) */

    return IMU_OK;
}

static int8_t ak09916_who_am_i(mag_handle_t *m, uint8_t *id)
{
    icm20948_handle_t *h = (icm20948_handle_t *)m->bus_ctx;
    /* AK09916 WIA2 = 0x01 at register 0x01, read back via I2C master */
    if (ak09916_setup_read(h, 0x01, 1) != IMU_OK) return IMU_ERR;
    h->base.delay(10);
    return read_reg(h, 0, B0_EXT_SLV_SENS_DATA_00, id, 1);
}

/* -------------------------------------------------------------------------
 * Public vtables
 * ------------------------------------------------------------------------- */

const imu_driver_t icm20948_imu_driver = {
    .init            = icm20948_init,
    .deinit          = icm20948_deinit,
    .read_raw        = icm20948_read_raw,
    .set_accel_range = icm20948_set_accel_range,
    .set_gyro_range  = icm20948_set_gyro_range,
    .who_am_i        = icm20948_who_am_i,
};

const mag_driver_t icm20948_mag_driver = {
    .init       = ak09916_init,
    .deinit     = ak09916_deinit,
    .read_raw   = ak09916_read_raw,
    .who_am_i   = ak09916_who_am_i,
};

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void ICM20948_HandleInit(icm20948_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         imu_accel_range_t  accel_range,
                         imu_gyro_range_t   gyro_range,
                         uint8_t            spi_mode)
{
    /* IMU base */
    h->base.driver      = &icm20948_imu_driver;
    h->base.read        = read;
    h->base.write       = write;
    h->base.delay       = delay;
    h->base.bus_ctx     = bus_ctx;
    h->base.gyro_bias_x = 0;
    h->base.gyro_bias_y = 0;
    h->base.gyro_bias_z = 0;
    h->accel_range      = accel_range;
    h->gyro_range       = gyro_range;
    h->current_bank     = 0xFF;
    h->spi_mode         = spi_mode;

    /*
     * Mag handle: shares the same delay but uses h itself as bus_ctx so
     * the ak09916 vtable functions can reach the ICM's SPI callbacks.
     */
    h->mag.driver  = &icm20948_mag_driver;
    h->mag.read    = read;
    h->mag.write   = write;
    h->mag.delay   = delay;
    h->mag.bus_ctx = h;
}

int8_t ICM20948_CalibrateGyro(icm20948_handle_t *h)
{
    imu_raw_data_t data;
    int32_t x = 0, y = 0, z = 0;

    for (int i = 0; i < 100; i++) {
        if (icm20948_read_raw(&h->base, &data) != IMU_OK) return IMU_ERR;
        x += data.gyro_x;
        y += data.gyro_y;
        z += data.gyro_z;
        h->base.delay(2);
    }

    int16_t bx = -(int16_t)(x / 100);
    int16_t by = -(int16_t)(y / 100);
    int16_t bz = -(int16_t)(z / 100);

    /* Average over 500 / 4 = 125 pairs for the 16-bit offset registers */
    bx /= 4; by /= 4; bz /= 4;

    if (write_reg(h, 2, B2_XG_OFFS_USRH, (uint8_t)(bx >> 8)) != IMU_OK) return IMU_ERR;
    if (write_reg(h, 2, B2_XG_OFFS_USRL, (uint8_t)(bx))      != IMU_OK) return IMU_ERR;
    if (write_reg(h, 2, B2_YG_OFFS_USRH, (uint8_t)(by >> 8)) != IMU_OK) return IMU_ERR;
    if (write_reg(h, 2, B2_YG_OFFS_USRL, (uint8_t)(by))      != IMU_OK) return IMU_ERR;
    if (write_reg(h, 2, B2_ZG_OFFS_USRH, (uint8_t)(bz >> 8)) != IMU_OK) return IMU_ERR;
    if (write_reg(h, 2, B2_ZG_OFFS_USRL, (uint8_t)(bz))      != IMU_OK) return IMU_ERR;

    return IMU_OK;
}
