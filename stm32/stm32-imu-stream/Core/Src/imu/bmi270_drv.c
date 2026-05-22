#include "bmi270_drv.h"

/* Register map */
#define REG_CHIP_ID       0x00   /* returns 0x24 */
#define REG_ERR_REG       0x02
#define REG_STATUS        0x03
#define REG_ACC_X_LSB     0x0C   /* 6 bytes: AX_L AX_H AY_L AY_H AZ_L AZ_H */
#define REG_GYR_X_LSB     0x12   /* 6 bytes: GX_L GX_H GY_L GY_H GZ_L GZ_H */
#define REG_INTERNAL_STATUS 0x21
#define REG_ACC_CONF      0x40
#define REG_ACC_RANGE     0x41
#define REG_GYR_CONF      0x42
#define REG_GYR_RANGE     0x43
#define REG_PWR_CONF      0x7C
#define REG_PWR_CTRL      0x7D
#define REG_CMD           0x7E
#define REG_INIT_CTRL     0x59
#define REG_INIT_DATA     0x5E
#define REG_INIT_ADDR_0   0x5B
#define REG_INIT_ADDR_1   0x5C

#define WHO_AM_I_VAL      0x24

/* ACC_RANGE values (datasheet Table 13) */
static const uint8_t accel_range_reg[] = { 0x00, 0x01, 0x02, 0x03 }; /* 2g 4g 8g 16g */

/* GYR_RANGE values (datasheet Table 15): 2000=0, 1000=1, 500=2, 250=3 */
static const uint8_t gyro_range_reg[]  = { 0x03, 0x02, 0x01, 0x00 }; /* 250 500 1000 2000 dps */

static inline bmi270_handle_t *to_bmi(imu_handle_t *h)
{
    return (bmi270_handle_t *)h;
}

static int8_t reg_read(imu_handle_t *h, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return h->read(reg, buf, len, h->bus_ctx);
}

static int8_t reg_write_byte(imu_handle_t *h, uint8_t reg, uint8_t val)
{
    return h->write(reg, &val, 1, h->bus_ctx);
}

static int8_t load_config_file(imu_handle_t *h)
{
    /* Disable advanced power save, disable load_config flag */
    if (reg_write_byte(h, REG_PWR_CONF,  0x00) != IMU_OK) return IMU_ERR;
    h->delay(1);
    if (reg_write_byte(h, REG_INIT_CTRL, 0x00) != IMU_OK) return IMU_ERR;

    /* Write config in 256-byte chunks */
    uint16_t index = 0;
    while (index < bmi270_config_data_len) {
        uint16_t chunk = bmi270_config_data_len - index;
        if (chunk > 256) chunk = 256;

        uint8_t addr_msn = (uint8_t)((index / 2) >> 4);
        uint8_t addr_lsn = (uint8_t)((index / 2) & 0x0F);

        if (reg_write_byte(h, REG_INIT_ADDR_0, addr_lsn) != IMU_OK) return IMU_ERR;
        if (reg_write_byte(h, REG_INIT_ADDR_1, addr_msn) != IMU_OK) return IMU_ERR;
        if (h->write(REG_INIT_DATA, &bmi270_config_data[index], chunk, h->bus_ctx) != IMU_OK)
            return IMU_ERR;

        index += chunk;
    }

    /* Enable config load and wait for INIT_OK */
    if (reg_write_byte(h, REG_INIT_CTRL, 0x01) != IMU_OK) return IMU_ERR;
    h->delay(20);

    uint8_t status;
    if (reg_read(h, REG_INTERNAL_STATUS, &status, 1) != IMU_OK) return IMU_ERR;
    if ((status & 0x0F) != 0x01) return IMU_ERR; /* message != init_ok */

    return IMU_OK;
}

/* ---------- vtable implementations ---------- */

static int8_t bmi270_init(imu_handle_t *h)
{
    uint8_t id;
    if (h->driver->who_am_i(h, &id) != IMU_OK) return IMU_ERR;
    if (id != WHO_AM_I_VAL) return IMU_ERR;

    /* Soft reset */
    if (reg_write_byte(h, REG_CMD, 0xB6) != IMU_OK) return IMU_ERR;
    h->delay(10);

    /* Re-read chip ID after reset (SPI requires one dummy read first) */
    if (reg_read(h, REG_CHIP_ID, &id, 1) != IMU_OK) return IMU_ERR;

    if (load_config_file(h) != IMU_OK) return IMU_ERR;

    /* Enable accel + gyro */
    if (reg_write_byte(h, REG_PWR_CTRL, 0x0E) != IMU_OK) return IMU_ERR;
    h->delay(2);

    if (h->driver->set_accel_range(h, IMU_ACCEL_RANGE_2G)   != IMU_OK) return IMU_ERR;
    if (h->driver->set_gyro_range (h, IMU_GYRO_RANGE_250DPS) != IMU_OK) return IMU_ERR;

    /* Normal power mode */
    if (reg_write_byte(h, REG_ACC_CONF, 0xA8) != IMU_OK) return IMU_ERR; /* ODR=100Hz, BWP=normal */
    if (reg_write_byte(h, REG_GYR_CONF, 0xA9) != IMU_OK) return IMU_ERR;

    return IMU_OK;
}

static int8_t bmi270_deinit(imu_handle_t *h)
{
    return reg_write_byte(h, REG_PWR_CTRL, 0x00); /* disable accel + gyro */
}

static int8_t bmi270_read_raw(imu_handle_t *h, imu_raw_data_t *out)
{
    uint8_t buf[12];
    if (reg_read(h, REG_ACC_X_LSB, buf, 6) != IMU_OK) return IMU_ERR;
    if (reg_read(h, REG_GYR_X_LSB, buf + 6, 6) != IMU_OK) return IMU_ERR;

    out->accel_x = (int16_t)((buf[1]  << 8) | buf[0]);
    out->accel_y = (int16_t)((buf[3]  << 8) | buf[2]);
    out->accel_z = (int16_t)((buf[5]  << 8) | buf[4]);
    out->gyro_x  = (int16_t)((buf[7]  << 8) | buf[6]);
    out->gyro_y  = (int16_t)((buf[9]  << 8) | buf[8]);
    out->gyro_z  = (int16_t)((buf[11] << 8) | buf[10]);

    return IMU_OK;
}

static int8_t bmi270_set_accel_range(imu_handle_t *h, imu_accel_range_t range)
{
    if (reg_write_byte(h, REG_ACC_RANGE, accel_range_reg[range]) != IMU_OK) return IMU_ERR;
    to_bmi(h)->accel_range = range;
    return IMU_OK;
}

static int8_t bmi270_set_gyro_range(imu_handle_t *h, imu_gyro_range_t range)
{
    if (reg_write_byte(h, REG_GYR_RANGE, gyro_range_reg[range]) != IMU_OK) return IMU_ERR;
    to_bmi(h)->gyro_range = range;
    return IMU_OK;
}

static int8_t bmi270_who_am_i(imu_handle_t *h, uint8_t *id)
{
    return reg_read(h, REG_CHIP_ID, id, 1);
}

/* ---------- public API ---------- */

const imu_driver_t bmi270_driver = {
    .init            = bmi270_init,
    .deinit          = bmi270_deinit,
    .read_raw        = bmi270_read_raw,
    .set_accel_range = bmi270_set_accel_range,
    .set_gyro_range  = bmi270_set_gyro_range,
    .who_am_i        = bmi270_who_am_i,
};

void BMI270_HandleInit(bmi270_handle_t   *h,
                       imu_read_fn        read,
                       imu_write_fn       write,
                       imu_delay_fn       delay,
                       void              *bus_ctx,
                       imu_accel_range_t  accel_range,
                       imu_gyro_range_t   gyro_range)
{
    h->base.driver  = &bmi270_driver;
    h->base.read    = read;
    h->base.write   = write;
    h->base.delay   = delay;
    h->base.bus_ctx = bus_ctx;
    h->accel_range  = accel_range;
    h->gyro_range   = gyro_range;
}
