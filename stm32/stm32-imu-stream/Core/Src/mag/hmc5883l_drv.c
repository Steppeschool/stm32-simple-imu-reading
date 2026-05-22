#include "hmc5883l_drv.h"

/* Register map */
#define REG_CFG_A     0x00   /* MA[6:5] DO[4:2] MS[1:0] */
#define REG_CFG_B     0x01   /* GN[7:5] */
#define REG_MODE      0x02   /* MR[1:0]: 00=continuous 01=single 10=idle */
#define REG_X_MSB     0x03   /* data order: X_H X_L Z_H Z_L Y_H Y_L */
#define REG_STATUS    0x09
#define REG_ID_A      0x0A   /* 'H' */
#define REG_ID_B      0x0B   /* '4' */
#define REG_ID_C      0x0C   /* '3' */

#define STATUS_RDY    (1 << 0)

/* 75 Hz output rate, normal measurement mode */
#define CFG_A_DEFAULT 0x78   /* MA=11(8 samples), DO=110(75Hz), MS=00(normal) */

static inline hmc5883l_handle_t *to_hmc(mag_handle_t *h)
{
    return (hmc5883l_handle_t *)h;
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

static int8_t hmc5883l_init(mag_handle_t *h)
{
    /* Verify identity registers: 'H', '4', '3' */
    uint8_t id[3];
    if (reg_read(h, REG_ID_A, id, 3) != IMU_OK) return IMU_ERR;
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') return IMU_ERR;

    if (reg_write_byte(h, REG_CFG_A, CFG_A_DEFAULT) != IMU_OK) return IMU_ERR;

    /* Set gain */
    uint8_t cfg_b = (uint8_t)(to_hmc(h)->gain << 5);
    if (reg_write_byte(h, REG_CFG_B, cfg_b) != IMU_OK) return IMU_ERR;

    /* Continuous measurement mode */
    if (reg_write_byte(h, REG_MODE, 0x00) != IMU_OK) return IMU_ERR;

    h->delay(10);
    return IMU_OK;
}

static int8_t hmc5883l_deinit(mag_handle_t *h)
{
    return reg_write_byte(h, REG_MODE, 0x02); /* idle */
}

static int8_t hmc5883l_read_raw(mag_handle_t *h, mag_raw_data_t *out)
{
    uint8_t buf[6];
    /* HMC5883L outputs in X Z Y order — reorder to X Y Z */
    if (reg_read(h, REG_X_MSB, buf, 6) != IMU_OK) return IMU_ERR;

    out->x = (int16_t)((buf[0] << 8) | buf[1]);   /* X */
    out->z = (int16_t)((buf[2] << 8) | buf[3]);   /* Z comes before Y in register layout */
    out->y = (int16_t)((buf[4] << 8) | buf[5]);   /* Y */

    return IMU_OK;
}

static int8_t hmc5883l_data_ready(mag_handle_t *h, uint8_t *ready)
{
    uint8_t status;
    if (reg_read(h, REG_STATUS, &status, 1) != IMU_OK) return IMU_ERR;
    *ready = (status & STATUS_RDY) ? 1u : 0u;
    return IMU_OK;
}

static int8_t hmc5883l_who_am_i(mag_handle_t *h, uint8_t *id)
{
    /* Return first ID register ('H') as the identifier */
    return reg_read(h, REG_ID_A, id, 1);
}

/* ---------- public API ---------- */

const mag_driver_t hmc5883l_driver = {
    .init       = hmc5883l_init,
    .deinit     = hmc5883l_deinit,
    .read_raw   = hmc5883l_read_raw,
    .data_ready = hmc5883l_data_ready,
    .who_am_i   = hmc5883l_who_am_i,
};

void HMC5883L_HandleInit(hmc5883l_handle_t *h,
                         imu_read_fn        read,
                         imu_write_fn       write,
                         imu_delay_fn       delay,
                         void              *bus_ctx,
                         hmc5883l_gain_t    gain)
{
    h->base.driver  = &hmc5883l_driver;
    h->base.read    = read;
    h->base.write   = write;
    h->base.delay   = delay;
    h->base.bus_ctx = bus_ctx;
    h->gain         = gain;
}
