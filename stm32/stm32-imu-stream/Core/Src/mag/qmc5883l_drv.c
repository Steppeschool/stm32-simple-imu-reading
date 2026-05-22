#include "qmc5883l_drv.h"

/* Register map */
#define REG_XOUT_L    0x00   /* 6 bytes: X_L X_H Y_L Y_H Z_L Z_H */
#define REG_STATUS    0x06
#define REG_TOUT_L    0x07
#define REG_CTRL1     0x09   /* OSR[7:6] RNG[5:4] ODR[3:2] MODE[1:0] */
#define REG_CTRL2     0x0A
#define REG_SET_RESET 0x0B   /* write 0x01 to enable set/reset */
#define REG_CHIP_ID   0x0D

#define CHIP_ID_VAL   0xFF

#define STATUS_DRDY   (1 << 0)
#define MODE_CONT     0x01

static inline qmc5883l_handle_t *to_qmc(mag_handle_t *h)
{
    return (qmc5883l_handle_t *)h;
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

static int8_t qmc5883l_init(mag_handle_t *h)
{
    qmc5883l_handle_t *dev = to_qmc(h);

    /* Mandatory: enable set/reset function */
    if (reg_write_byte(h, REG_SET_RESET, 0x01) != IMU_OK) return IMU_ERR;

    /* OSR=512(00), configure RNG, ODR, continuous mode */
    uint8_t ctrl1 = (uint8_t)((0x00 << 6)           /* OSR = 512 */
                             | (dev->range << 4)
                             | (dev->odr   << 2)
                             | MODE_CONT);
    if (reg_write_byte(h, REG_CTRL1, ctrl1) != IMU_OK) return IMU_ERR;

    return IMU_OK;
}

static int8_t qmc5883l_deinit(mag_handle_t *h)
{
    /* MODE = 00 → standby */
    return reg_write_byte(h, REG_CTRL1, 0x00);
}

static int8_t qmc5883l_read_raw(mag_handle_t *h, mag_raw_data_t *out)
{
    uint8_t buf[6];
    if (reg_read(h, REG_XOUT_L, buf, 6) != IMU_OK) return IMU_ERR;

    out->x = (int16_t)((buf[1] << 8) | buf[0]);
    out->y = (int16_t)((buf[3] << 8) | buf[2]);
    out->z = (int16_t)((buf[5] << 8) | buf[4]);

    return IMU_OK;
}

static int8_t qmc5883l_data_ready(mag_handle_t *h, uint8_t *ready)
{
    uint8_t status;
    if (reg_read(h, REG_STATUS, &status, 1) != IMU_OK) return IMU_ERR;
    *ready = (status & STATUS_DRDY) ? 1u : 0u;
    return IMU_OK;
}

static int8_t qmc5883l_who_am_i(mag_handle_t *h, uint8_t *id)
{
    return reg_read(h, REG_CHIP_ID, id, 1);
}

/* ---------- public API ---------- */

const mag_driver_t qmc5883l_driver = {
    .init       = qmc5883l_init,
    .deinit     = qmc5883l_deinit,
    .read_raw   = qmc5883l_read_raw,
    .data_ready = qmc5883l_data_ready,
    .who_am_i   = qmc5883l_who_am_i,
};

void QMC5883L_HandleInit(qmc5883l_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         qmc5883l_range_t   range,
                         qmc5883l_odr_t     odr)
{
    h->base.driver  = &qmc5883l_driver;
    h->base.read    = read;
    h->base.write   = write;
    h->base.delay   = delay;
    h->base.bus_ctx = bus_ctx;
    h->range        = range;
    h->odr          = odr;
}
