#include "mmc5983ma_drv.h"

/* Register map */
#define REG_X_OUT_0       0x00   /* bits [17:10] of X */
#define REG_X_OUT_1       0x01   /* bits [9:2]   of X */
#define REG_Y_OUT_0       0x02
#define REG_Y_OUT_1       0x03
#define REG_Z_OUT_0       0x04
#define REG_Z_OUT_1       0x05
#define REG_XYZ_OUT_2     0x06   /* bits [1:0] of each axis */
#define REG_CTRL0         0x09   /* TM_M [0], SET [3], RESET [4] — write-only */
#define CTRL0_SET         (1u << 3)
#define CTRL0_RESET       (1u << 4)
#define REG_CTRL1         0x0A   /* BW[1:0], SW_RST [7] — write-only */
#define REG_CTRL2         0x0B   /* CMM_FREQ[2:0], CMM_EN [4] — write-only */
#define REG_PRODUCT_ID    0x2F   /* returns 0x30 — read-only */

#define PRODUCT_ID_VAL    0x30
#define CTRL2_CMM_EN      (1u << 4)

/*
 * Continuous measurement mode (CMM) frequency codes (CTRL2 bits [2:0]).
 * Chosen to match the bandwidth setting so the sensor is never idle.
 *   BW_100HZ (8 ms/sample)  → 100 Hz = 0x05
 *   BW_200HZ (4 ms/sample)  → 200 Hz = 0x06
 *   BW_400HZ (2 ms/sample)  → 200 Hz = 0x06  (1000 Hz needs BW=3)
 *   BW_800HZ (0.5 ms/sample)→ 1000 Hz = 0x07
 */
static const uint8_t cmm_freq_for_bw[] = { 0x05, 0x06, 0x06, 0x07 };

static int8_t reg_read(mag_handle_t *h, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return h->read(reg, buf, len, h->bus_ctx);
}

static int8_t reg_write_byte(mag_handle_t *h, uint8_t reg, uint8_t val)
{
    return h->write(reg, &val, 1, h->bus_ctx);
}

/* ---------- vtable implementations ---------- */

static int8_t mmc5983ma_init(mag_handle_t *h)
{
    uint8_t id;
    if (h->driver->who_am_i(h, &id) != IMU_OK) return IMU_ERR;
    if (id != PRODUCT_ID_VAL) return IMU_ERR;

    /* Soft reset — all registers return to 0x00 */
    if (reg_write_byte(h, REG_CTRL1, 0x80) != IMU_OK) return IMU_ERR;
    h->delay(50);   /* wait for POR to complete; 20 ms per datasheet, 50 ms margin */

    /* Confirm the sensor is back on the bus after reset */
    if (h->driver->who_am_i(h, &id) != IMU_OK) return IMU_ERR;
    if (id != PRODUCT_ID_VAL) return IMU_ERR;
    h->delay(50);
    /* RESET then SET: put the sensing bridge in a fully known state */
    if (reg_write_byte(h, REG_CTRL0, CTRL0_RESET) != IMU_OK) return IMU_ERR;
    h->delay(50);
    if (reg_write_byte(h, REG_CTRL0, CTRL0_SET)   != IMU_OK) return IMU_ERR;
    h->delay(50);

    /* Configure bandwidth */
    mmc5983ma_handle_t *dev = (mmc5983ma_handle_t *)h;
    if (reg_write_byte(h, REG_CTRL1, (uint8_t)dev->bw) != IMU_OK) return IMU_ERR;
    h->delay(50);

    /* Enable continuous measurement mode at the matching output rate */
    uint8_t cmm = CTRL2_CMM_EN | cmm_freq_for_bw[dev->bw & 0x03];
    if (reg_write_byte(h, REG_CTRL2, cmm) != IMU_OK) return IMU_ERR;

    /* Wait for several CMM measurements to complete before the app starts reading */
    h->delay(100);

    return IMU_OK;
}

static int8_t mmc5983ma_deinit(mag_handle_t *h)
{
    /* Stop continuous mode */
    return reg_write_byte(h, REG_CTRL2, 0x00);
}

static int8_t mmc5983ma_read_raw(mag_handle_t *h, mag_raw_data_t *out)
{
    /* Read 7 bytes: X[17:10] X[9:2] Y[17:10] Y[9:2] Z[17:10] Z[9:2] XYZ[1:0] */
    uint8_t buf[7];
    if (reg_read(h, REG_X_OUT_0, buf, 7) != IMU_OK) return IMU_ERR;

    /*
     * MMC5983MA has 18-bit output. We use the top 16 bits (discard bit[1:0]
     * from REG_XYZ_OUT_2) and subtract the 2^17 zero-field offset so the
     * result is a signed int16.
     */
    int32_t x = (int32_t)(((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((buf[6] >> 6) & 0x03));
    int32_t y = (int32_t)(((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[6] >> 4) & 0x03));
    int32_t z = (int32_t)(((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[6] >> 2) & 0x03));

    /* Remove 2^17 = 131072 zero-field offset, then shift to 16-bit range */
    out->x = (int16_t)((x - 131072) >> 2);
    out->y = -(int16_t)((y - 131072) >> 2);
    out->z = (int16_t)((z - 131072) >> 2);

    return IMU_OK;
}

static int8_t mmc5983ma_who_am_i(mag_handle_t *h, uint8_t *id)
{
    return reg_read(h, REG_PRODUCT_ID, id, 1);
}

/* ---------- public API ---------- */

const mag_driver_t mmc5983ma_driver = {
    .init       = mmc5983ma_init,
    .deinit     = mmc5983ma_deinit,
    .read_raw   = mmc5983ma_read_raw,
    .who_am_i   = mmc5983ma_who_am_i,
};

void MMC5983MA_HandleInit(mmc5983ma_handle_t *h,
                          imu_read_fn         read,
                          imu_write_fn        write,
                          imu_delay_fn        delay,
                          void               *bus_ctx,
                          mmc5983ma_bw_t      bw)
{
    h->base.driver  = &mmc5983ma_driver;
    h->base.read    = read;
    h->base.write   = write;
    h->base.delay   = delay;
    h->base.bus_ctx = bus_ctx;
    h->bw           = bw;
}
