#include "mpu6050_drv.h"

/* Register map */
#define REG_SELF_TEST_X   0x0D
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B   /* 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L */
#define REG_GYRO_XOUT_H   0x43   /* 6 bytes: GX_H GX_L GY_H GY_L GZ_H GZ_L */
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

#define WHO_AM_I_VAL      0x68

/* FS_SEL / AFS_SEL bit position in config registers */
#define GYRO_FS_SHIFT     3
#define ACCEL_FS_SHIFT    3

static inline mpu6050_handle_t *to_mpu(imu_handle_t *h)
{
    return (mpu6050_handle_t *)h;
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

static int8_t mpu6050_init(imu_handle_t *h)
{
    uint8_t id;
    if (h->driver->who_am_i(h, &id) != IMU_OK) return IMU_ERR;
    if (id != WHO_AM_I_VAL) return IMU_ERR;

    /* Wake up, use PLL with gyro X as clock source */
    if (reg_write_byte(h, REG_PWR_MGMT_1, 0x01) != IMU_OK) return IMU_ERR;
    h->delay(100);

    if (h->driver->set_accel_range(h, to_mpu(h)->accel_range) != IMU_OK) return IMU_ERR;
    if (h->driver->set_gyro_range (h, to_mpu(h)->gyro_range)  != IMU_OK) return IMU_ERR;

    return IMU_OK;
}

static int8_t mpu6050_deinit(imu_handle_t *h)
{
    /* Sleep mode */
    return reg_write_byte(h, REG_PWR_MGMT_1, 0x40);
}

static int8_t mpu6050_read_raw(imu_handle_t *h, imu_raw_data_t *out)
{
    uint8_t buf[14];
    /* Read accel (6) + temp (2) + gyro (6) in a single burst */
    if (reg_read(h, REG_ACCEL_XOUT_H, buf, 14) != IMU_OK) return IMU_ERR;

    out->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature, skip */
    out->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return IMU_OK;
}

static int8_t mpu6050_set_accel_range(imu_handle_t *h, imu_accel_range_t range)
{
    /* AFS_SEL[1:0] at bits [4:3] */
    uint8_t val = (uint8_t)(range << ACCEL_FS_SHIFT);
    if (reg_write_byte(h, REG_ACCEL_CONFIG, val) != IMU_OK) return IMU_ERR;
    to_mpu(h)->accel_range = range;
    return IMU_OK;
}

static int8_t mpu6050_set_gyro_range(imu_handle_t *h, imu_gyro_range_t range)
{
    /* FS_SEL[1:0] at bits [4:3] */
    uint8_t val = (uint8_t)(range << GYRO_FS_SHIFT);
    if (reg_write_byte(h, REG_GYRO_CONFIG, val) != IMU_OK) return IMU_ERR;
    to_mpu(h)->gyro_range = range;
    return IMU_OK;
}

static int8_t mpu6050_who_am_i(imu_handle_t *h, uint8_t *id)
{
    return reg_read(h, REG_WHO_AM_I, id, 1);
}

/* ---------- public API ---------- */

const imu_driver_t mpu6050_driver = {
    .init            = mpu6050_init,
    .deinit          = mpu6050_deinit,
    .read_raw        = mpu6050_read_raw,
    .set_accel_range = mpu6050_set_accel_range,
    .set_gyro_range  = mpu6050_set_gyro_range,
    .who_am_i        = mpu6050_who_am_i,
};

void MPU6050_HandleInit(mpu6050_handle_t  *h,
                        imu_read_fn        read,
                        imu_write_fn       write,
                        imu_delay_fn       delay,
                        void              *bus_ctx,
                        uint8_t            dev_addr,
                        imu_accel_range_t  accel_range,
                        imu_gyro_range_t   gyro_range)
{
    h->base.driver     = &mpu6050_driver;
    h->base.read       = read;
    h->base.write      = write;
    h->base.delay      = delay;
    h->base.bus_ctx    = bus_ctx;
    h->base.gyro_bias_x = 0;
    h->base.gyro_bias_y = 0;
    h->base.gyro_bias_z = 0;
    h->dev_addr        = dev_addr;
    h->accel_range     = accel_range;
    h->gyro_range      = gyro_range;
}
