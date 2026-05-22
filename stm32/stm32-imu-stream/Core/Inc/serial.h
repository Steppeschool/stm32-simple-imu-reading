/*
 * uart_data_transmit.h
 *
 *  Created on: 14 May 2026
 *      Author: yerke
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "imu_hal.h"
#include "mag_hal.h"

/*
 * Packet layout (20 bytes):
 *   [0]     0xAA  header byte 1
 *   [1]     0xFF  header byte 2
 *   [2-3]   accel X  (little-endian int16)
 *   [4-5]   accel Y
 *   [6-7]   accel Z
 *   [8-9]   gyro  X
 *   [10-11] gyro  Y
 *   [12-13] gyro  Z
 *   [14-15] mag   X
 *   [16-17] mag   Y
 *   [18-19] mag   Z
 */
#define PACKET_BYTE1   0xAA
#define PACKET_BYTE2   0xFF
#define PACKET_SIZE    20

void serial_send_packet(const imu_raw_data_t *imu, const mag_raw_data_t *mag);

#endif /* INC_SERIAL_H_ */
