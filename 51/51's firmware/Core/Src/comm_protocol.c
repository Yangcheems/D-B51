/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : comm_protocol.c
  ******************************************************************************
  */
/* USER CODE END Header */

#include "comm_protocol.h"
#include <string.h>

uint16_t Comm_CRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint16_t Comm_BuildPacket(CommPacket_t* packet, uint8_t msg_type,
                          const uint8_t* payload, uint16_t length) {
    if (length > COMM_MAX_PAYLOAD) {
        return 0;
    }

    packet->header[0] = COMM_HEADER_0;
    packet->header[1] = COMM_HEADER_1;
    packet->msg_type = msg_type;
    packet->length = length;

    if (length > 0 && payload != NULL) {
        memcpy(packet->payload, payload, length);
    }

    uint8_t* crc_start = (uint8_t*)&packet->msg_type;
    uint16_t crc_len = 1 + 2 + length;
    packet->crc16 = Comm_CRC16(crc_start, crc_len);

    return COMM_PACKET_OVERHEAD + length;
}

void Comm_InitRxBuffer(CommRxBuffer_t* rx_buf) {
    memset(rx_buf, 0, sizeof(CommRxBuffer_t));
    rx_buf->state = RX_STATE_WAIT_HEADER0;
}

int8_t Comm_ProcessByte(CommRxBuffer_t* rx_buf, uint8_t byte,
                        uint8_t* msg_type, uint8_t* payload, uint16_t* length) {
    switch (rx_buf->state) {
    case RX_STATE_WAIT_HEADER0:
        if (byte == COMM_HEADER_0) {
            rx_buf->state = RX_STATE_WAIT_HEADER1;
        }
        break;

    case RX_STATE_WAIT_HEADER1:
        if (byte == COMM_HEADER_1) {
            rx_buf->state = RX_STATE_GET_TYPE;
        } else {
            rx_buf->state = RX_STATE_WAIT_HEADER0;
        }
        break;

    case RX_STATE_GET_TYPE:
        rx_buf->msg_type = byte;
        rx_buf->state = RX_STATE_GET_LENGTH_H;
        break;

    case RX_STATE_GET_LENGTH_H:
        rx_buf->length = (uint16_t)byte;
        rx_buf->state = RX_STATE_GET_LENGTH_L;
        break;

    case RX_STATE_GET_LENGTH_L:
        rx_buf->length |= ((uint16_t)byte << 8);
        if (rx_buf->length > COMM_MAX_PAYLOAD) {
            rx_buf->state = RX_STATE_WAIT_HEADER0;
            rx_buf->error_count++;
        } else {
            rx_buf->payload_index = 0;
            if (rx_buf->length > 0) {
                rx_buf->state = RX_STATE_GET_PAYLOAD;
            } else {
                rx_buf->state = RX_STATE_GET_CRC_H;
            }
        }
        break;

    case RX_STATE_GET_PAYLOAD:
        rx_buf->payload[rx_buf->payload_index++] = byte;
        if (rx_buf->payload_index >= rx_buf->length) {
            rx_buf->state = RX_STATE_GET_CRC_H;
        }
        break;

    case RX_STATE_GET_CRC_H:
        rx_buf->crc_received = (uint16_t)byte;
        rx_buf->state = RX_STATE_GET_CRC_L;
        break;

    case RX_STATE_GET_CRC_L:
        rx_buf->crc_received |= ((uint16_t)byte << 8);
        rx_buf->state = RX_STATE_WAIT_HEADER0;

        uint8_t temp_buf[1 + 2 + COMM_MAX_PAYLOAD];
        temp_buf[0] = rx_buf->msg_type;
        temp_buf[1] = rx_buf->length & 0xFF;
        temp_buf[2] = (rx_buf->length >> 8) & 0xFF;
        memcpy(&temp_buf[3], rx_buf->payload, rx_buf->length);

        uint16_t crc_calc = Comm_CRC16(temp_buf, 3 + rx_buf->length);

        if (crc_calc == rx_buf->crc_received) {
            *msg_type = rx_buf->msg_type;
            memcpy(payload, rx_buf->payload, rx_buf->length);
            *length = rx_buf->length;
            rx_buf->packet_count++;
            return 1;
        } else {
            rx_buf->error_count++;
            return -1;
        }
    }

    return 0;
}

void Comm_SendOdometry(UART_HandleTypeDef* huart, const OdometryData_t* odom) {
    CommPacket_t packet;
    uint16_t pkt_len = Comm_BuildPacket(&packet, MSG_TYPE_ODOMETRY,
                                        (const uint8_t*)odom, sizeof(OdometryData_t));
    if (pkt_len > 0) {
        HAL_UART_Transmit(huart, (uint8_t*)&packet, pkt_len, 100);
    }
}

void Comm_SendLidarData(UART_HandleTypeDef* huart, const LidarData_t* lidar) {
    CommPacket_t packet;
    uint16_t data_size = sizeof(uint32_t) + sizeof(uint16_t) +
                         lidar->num_points * (sizeof(float) * 2 + sizeof(uint8_t));
    uint16_t pkt_len = Comm_BuildPacket(&packet, MSG_TYPE_LIDAR_DATA,
                                        (const uint8_t*)lidar, data_size);
    if (pkt_len > 0) {
        HAL_UART_Transmit(huart, (uint8_t*)&packet, pkt_len, 100);
    }
}

void Comm_SendImuData(UART_HandleTypeDef* huart, const ImuData_t* imu) {
    CommPacket_t packet;
    uint16_t pkt_len = Comm_BuildPacket(&packet, MSG_TYPE_IMU_DATA,
                                        (const uint8_t*)imu, sizeof(ImuData_t));
    if (pkt_len > 0) {
        HAL_UART_Transmit(huart, (uint8_t*)&packet, pkt_len, 100);
    }
}

void Comm_SendAck(UART_HandleTypeDef* huart, uint8_t msg_type) {
    CommPacket_t packet;
    uint16_t pkt_len = Comm_BuildPacket(&packet, MSG_TYPE_ACK, &msg_type, 1);
    if (pkt_len > 0) {
        HAL_UART_Transmit(huart, (uint8_t*)&packet, pkt_len, 100);
    }
}