#include "serial.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

#define serial_port  huart2
static uint8_t serial_data[PACKET_SIZE];

static void serial_send_data(void)
{
    HAL_UART_Transmit_DMA(&serial_port, serial_data, PACKET_SIZE);
}

void serial_send_packet(const imu_raw_data_t *imu, const mag_raw_data_t *mag)
{
    serial_data[0]  = PACKET_BYTE1;
    serial_data[1]  = PACKET_BYTE2;

    serial_data[2]  = (uint8_t)(imu->accel_x);
    serial_data[3]  = (uint8_t)(imu->accel_x >> 8);
    serial_data[4]  = (uint8_t)(imu->accel_y);
    serial_data[5]  = (uint8_t)(imu->accel_y >> 8);
    serial_data[6]  = (uint8_t)(imu->accel_z);
    serial_data[7]  = (uint8_t)(imu->accel_z >> 8);

    serial_data[8]  = (uint8_t)(imu->gyro_x);
    serial_data[9]  = (uint8_t)(imu->gyro_x >> 8);
    serial_data[10] = (uint8_t)(imu->gyro_y);
    serial_data[11] = (uint8_t)(imu->gyro_y >> 8);
    serial_data[12] = (uint8_t)(imu->gyro_z);
    serial_data[13] = (uint8_t)(imu->gyro_z >> 8);

    serial_data[14] = (uint8_t)(mag->x);
    serial_data[15] = (uint8_t)(mag->x >> 8);
    serial_data[16] = (uint8_t)(mag->y);
    serial_data[17] = (uint8_t)(mag->y >> 8);
    serial_data[18] = (uint8_t)(mag->z);
    serial_data[19] = (uint8_t)(mag->z >> 8);

    serial_send_data();
}

