/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms in LICENSE file or provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "pose_controller.h"
#include "comm_protocol.h"
#include "odometry.h"
#include "safety.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float integral_limit;
} PID_Controller;

#pragma pack(push, 1)
typedef struct {
    uint8_t sync_quality;
    uint16_t angle_q6;
    uint16_t distance_q2;
} LidarDataPacket;
#pragma pack(pop)

typedef struct {
    float distance;
    uint8_t quality;
    uint32_t update_time;
    uint8_t valid;
} AngleCache;

/* Private define ------------------------------------------------------------*/
#define MAX_SPEED 400
#define MAX_SPEED_RIGHT 420
#define SPEED_UPDATE_INTERVAL 10
#define TURN_DURATION 800
#define TURN_SPEED 300
#define DIR_STOP 0
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_AUTO_LEFT 5
#define DIR_AUTO_RIGHT 6
#define PPR 360
#define SAMPLE_TIME_MS 100
#define M_PI 3.14159265358979323846
#define DMA_BUFFER_SIZE 256
#define LIDAR_PACKET_SIZE 5
#define ANGLE_FILTER_THRESHOLD 1.0f
#define MIN_VALID_DISTANCE 50.0f
#define MAX_VALID_DISTANCE 12000.0f
#define LIDAR_TIMEOUT_THRESHOLD 500

#define PID_KP 1.2f
#define PID_KI 0.1f
#define PID_KD 0.05f
#define PID_INTEGRAL_LIMIT 200.0f
#define RPM_TO_PWM_FACTOR 2.0f

#define MPU6500_ADDR 0xD0
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75
#define CALIBRATION_SAMPLES 500
#define ACCEL_FS_SEL_4G 0x08
#define GYRO_FS_SEL_500DPS 0x08

#define TURN_ANGLE_THRESHOLD 3.0f
#define AUTO_TURN_SPEED 300
#define MIN_TURN_SPEED 120
#define SLOWDOWN_ANGLE 45.0f
#define OVERSHOOT_THRESHOLD 30.0f

#define GYRO_BIAS_UPDATE_INTERVAL 10
#define COMPLEMENTARY_FILTER_GAIN 0.02f
#define STATIONARY_THRESHOLD 0.1f
#define STATIONARY_DURATION 1000

/* Private variables ---------------------------------------------------------*/
uint8_t bluetooth_rx_data = 0;
uint8_t target_direction = DIR_STOP;
uint8_t current_direction = DIR_STOP;
uint32_t current_speed = 0;
uint32_t target_speed = 0;
uint32_t last_speed_update_time = 0;
uint32_t last_encoder_time = 0;

uint8_t bluetooth_connected = 0;
uint8_t connection_announced = 0;
uint8_t connection_msg[] = "Connected\r\n";

int32_t lastEncoderA = 0;
int32_t lastEncoderB = 0;

char uart_buf[256];

uint8_t lidar_dma_buffer[DMA_BUFFER_SIZE];
uint8_t lidar_dataBuffer[LIDAR_PACKET_SIZE];
uint8_t lidar_dataIndex = 0;
uint32_t lidar_rxIndex = 0;
volatile uint8_t lidar_process_packet = 0;
float lastLidarAngle = 0.0f;
uint32_t lastLidarRxTime = 0;

AngleCache angle_cache[360] = { 0 };
uint8_t current_output_angle = 0;
uint32_t last_output_time = 0;

int16_t accel_offset_x, accel_offset_y, accel_offset_z;
int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temp;
float roll, pitch;
float yaw = 0.0f;
uint32_t last_imu_time = 0;

PID_Controller pid_left = { PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, PID_INTEGRAL_LIMIT };
PID_Controller pid_right = { PID_KP, PID_KI, PID_KD, 0.0f, 0.0f, PID_INTEGRAL_LIMIT };
float target_rpm = 0.0f;
float pwm_left = 0, pwm_right = 0;

uint8_t auto_turn_mode = 0;
float start_yaw = 0.0f;
float target_yaw = 0.0f;
float prev_angle_diff = 0.0f;
uint8_t overshoot_count = 0;

float gyro_z_bias = 0.0f;
uint8_t is_stationary = 0;
uint32_t stationary_start_time = 0;

PoseController_t g_pose_controller;
PoseControllerConfig_t g_pose_config = {
    .linear_kp = 150.0f,
    .linear_ki = 3.0f,
    .linear_kd = 5.0f,
    .angular_kp = 100.0f,
    .angular_ki = 2.0f,
    .angular_kd = 5.0f,
    .tolerance_pos = 0.05f,
    .tolerance_ang = 3.0f,
    .max_linear_vel = 0.3f,
    .max_angular_vel = 1.0f
};

Odometry_t g_odometry;
OdometryConfig_t g_odom_config = {
    .wheel_radius = 0.0325f,
    .wheel_base = 0.165f,
    .encoder_ppr = 360,
    .gear_ratio = 1.0f
};

CommRxBuffer_t g_comm_rx_buffer;

SafetyStatus_t g_safety_status = {0};
SafetyConfig_t g_safety_config = {
    .max_linear_vel = 0.8f,
    .max_angular_vel = 2.0f,
    .max_pwm = 400,
    .estop_gpio_pin = GPIO_PIN_0,
    .estop_gpio_port = GPIOA,
    .watchdog_timeout_ms = 60000
};

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ControlMotor(uint8_t direction, uint32_t speed);
void UpdateSpeedRamp(void);
void SendConnectionNotification(void);
void ProcessLidarData(uint8_t* data);
void SendLidarToBluetooth(float angle, float distance, uint8_t quality);
float PID_Update(PID_Controller* pid, float setpoint, float measurement);
void MPU6500_Init(I2C_HandleTypeDef* hi2c);
void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c);
void MPU6500_Read_All(I2C_HandleTypeDef* hi2c);
void Calculate_Angles(void);
void Send_IMU_Data_Bluetooth(void);
float CalculateAngleDifference(float current, float target);
void HandleAutoTurn(void);

/* Private user code ---------------------------------------------------------*/

float CalculateAngleDifference(float current, float target) {
    float diff = target - current;

    if (diff > 180.0f) {
        diff -= 360.0f;
    }
    else if (diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}

float PID_Update(PID_Controller* pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart1) {
        if (!bluetooth_connected) bluetooth_connected = 1;

        Safety_FeedWatchdog(&g_safety_status);

        uint8_t msg_type;
        uint8_t payload[128];
        uint16_t length;
        int8_t result = Comm_ProcessByte(&g_comm_rx_buffer, bluetooth_rx_data,
                                         &msg_type, payload, &length);

        if (result == 1) {
            if (msg_type == MSG_TYPE_POSE_SETPOINT) {
                PoseSetpoint_t* setpoint = (PoseSetpoint_t*)payload;

                float theta_deg = setpoint->theta * 180.0f / M_PI;

                PoseController_SetTarget(&g_pose_controller,
                                        setpoint->x, setpoint->y, theta_deg);
                PoseController_Enable(&g_pose_controller, 1);

                Comm_SendAck(&huart1, MSG_TYPE_POSE_SETPOINT);

                auto_turn_mode = 0;
                target_direction = DIR_STOP;
            }
            else if (msg_type == MSG_TYPE_SET_ODOMETRY) {
                SetOdometryData_t* set_odom = (SetOdometryData_t*)payload;

                Odometry_SetPose(&g_odometry, set_odom->x, set_odom->y, set_odom->theta);

                Comm_SendAck(&huart1, MSG_TYPE_SET_ODOMETRY);

                char dbg[100];
                snprintf(dbg, sizeof(dbg), "[ODOM-SET] 里程计初始位置已设置: x=%.3f y=%.3f θ=%.3f\r\n",
                        set_odom->x, set_odom->y, set_odom->theta);
                HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
            }
        }

        if (bluetooth_rx_data >= '0' && bluetooth_rx_data <= '7') {
            PoseController_Enable(&g_pose_controller, 0);

            target_direction = bluetooth_rx_data - '0';

            if (target_direction == DIR_AUTO_LEFT || target_direction == DIR_AUTO_RIGHT) {
                auto_turn_mode = target_direction;
                start_yaw = yaw;
                prev_angle_diff = 0.0f;
                overshoot_count = 0;

                if (target_direction == DIR_AUTO_LEFT) {
                    target_yaw = yaw + 80.0f;
                    if (target_yaw >= 360.0f) target_yaw -= 360.0f;
                }
                else {
                    target_yaw = yaw - 80.0f;
                    if (target_yaw < 0.0f) target_yaw += 360.0f;
                }

                current_speed = AUTO_TURN_SPEED;
                target_speed = AUTO_TURN_SPEED;

                char msg[60];
                snprintf(msg, sizeof(msg), "Auto turn started. Start:%.1f��, Target:%.1f��\r\n", start_yaw, target_yaw);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
            else {
                auto_turn_mode = 0;

                switch (target_direction) {
                case DIR_STOP:
                    target_speed = 0;
                    current_speed = 0;
                    target_rpm = 0.0f;
                    break;
                case DIR_FORWARD:
                case DIR_BACKWARD:
                    target_speed = MAX_SPEED;
                    current_speed = MAX_SPEED;
                    target_rpm = 60.0f;
                    break;
                case DIR_LEFT:
                case DIR_RIGHT:
                    target_speed = TURN_SPEED;
                    current_speed = TURN_SPEED;
                    target_rpm = 30.0f;
                    break;
                }
                ControlMotor(target_direction, current_speed);
            }
        }
        HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);
    }
    else if (huart == &huart6) {
        lidar_process_packet = 1;
        HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);
    }
}

void ControlMotor(uint8_t direction, uint32_t speed) {
    switch (direction) {
    case DIR_STOP:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_FORWARD:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (pwm_left > 0) ? (uint32_t)pwm_left : speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (pwm_right > 0) ? (uint32_t)pwm_right : speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_BACKWARD:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (pwm_left > 0) ? (uint32_t)pwm_left : speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (pwm_right > 0) ? (uint32_t)pwm_right : speed);
        break;
    case DIR_LEFT:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        break;
    case DIR_RIGHT:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TURN_SPEED);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, TURN_SPEED);
        break;
    }
}

void UpdateSpeedRamp(void) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        last_speed_update_time = current_time;

        if (auto_turn_mode != 0) return;

        if (target_direction == DIR_STOP && current_direction != DIR_STOP) {
            current_speed = 0;
            current_direction = DIR_STOP;
            ControlMotor(DIR_STOP, 0);
            return;
        }

        if (target_direction != current_direction) {
            current_direction = target_direction;
            if (current_direction == DIR_LEFT || current_direction == DIR_RIGHT) {
                current_speed = TURN_SPEED;
                ControlMotor(current_direction, current_speed);
            }
            else if (current_direction != DIR_STOP) {
                current_speed = target_speed = MAX_SPEED;
                ControlMotor(current_direction, current_speed);
            }
        }
        else if (current_direction != DIR_STOP) {
            if (current_speed < target_speed) {
                current_speed = target_speed;
                ControlMotor(current_direction, current_speed);
            }
        }
    }
}

void SendConnectionNotification(void) {
    if (bluetooth_connected && !connection_announced) {
        HAL_UART_Transmit(&huart1, connection_msg, sizeof(connection_msg) - 1, 1000);
        connection_announced = 1;
    }
}

void HandleAutoTurn(void) {
    float angle_diff = CalculateAngleDifference(yaw, target_yaw);
    float abs_diff = fabsf(angle_diff);

    if ((angle_diff * prev_angle_diff < 0) && (abs_diff > OVERSHOOT_THRESHOLD)) {
        overshoot_count++;
        if (overshoot_count > 2) {
            ControlMotor(DIR_STOP, 0);
            auto_turn_mode = 0;
            overshoot_count = 0;

            gyro_z_bias += angle_diff * 0.01f;

            char msg[60];
            snprintf(msg, sizeof(msg), "Auto turn overshoot! Stopped. Adjusted bias: %.3f\r\n", gyro_z_bias);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            return;
        }
    }
    else {
        overshoot_count = 0;
    }
    prev_angle_diff = angle_diff;

    if (abs_diff <= TURN_ANGLE_THRESHOLD) {
        ControlMotor(DIR_STOP, 0);
        auto_turn_mode = 0;

        float current_gyro = gyro_z / 65.5f;
        gyro_z_bias = 0.95f * gyro_z_bias + 0.05f * current_gyro;

        HAL_UART_Transmit(&huart1, (uint8_t*)"Auto turn completed!\r\n", 22, 100);
    }
    else {
        float speed_factor = 1.0f;
        if (abs_diff < SLOWDOWN_ANGLE) {
            speed_factor = 0.3f + 0.7f * (abs_diff / SLOWDOWN_ANGLE);
        }

        uint32_t adjusted_speed = (uint32_t)(AUTO_TURN_SPEED * speed_factor);
        if (adjusted_speed < MIN_TURN_SPEED) adjusted_speed = MIN_TURN_SPEED;

        if (auto_turn_mode == DIR_AUTO_LEFT) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, adjusted_speed);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, adjusted_speed);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        }
        else if (auto_turn_mode == DIR_AUTO_RIGHT) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, adjusted_speed);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, adjusted_speed);
        }

        static uint32_t last_debug_time = 0;
        if (HAL_GetTick() - last_debug_time > 200) {
            last_debug_time = HAL_GetTick();

            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
        }
    }
}

void ProcessLidarData(uint8_t* data) {
    LidarDataPacket* pkt = (LidarDataPacket*)data;

    if ((pkt->sync_quality & 0x80) && (pkt->angle_q6 & 0x01)) {
        uint8_t quality = pkt->sync_quality & 0x7F;

        float raw_angle = (pkt->angle_q6 >> 1) / 64.0f;
        float distance = pkt->distance_q2 / 4.0f;


        if (raw_angle >= 0.0f && raw_angle <= 360.0f) {
            float angle = fmodf(raw_angle, 360.0f);
            if (angle < 0) angle += 360.0f;

            if (distance >= MIN_VALID_DISTANCE &&
                distance <= MAX_VALID_DISTANCE &&
                quality > 0) {

                int32_t angle_deg = (int32_t)(angle + 0.5f);
                angle_deg %= 360;
                if (angle_deg < 0) angle_deg += 360;

                angle_cache[angle_deg].distance = distance;
                angle_cache[angle_deg].quality = quality;
                angle_cache[angle_deg].update_time = HAL_GetTick();
                angle_cache[angle_deg].valid = 1;
            }
        }
        else {
            static uint32_t invalid_angle_count = 0;
            invalid_angle_count++;

            if (invalid_angle_count % 100 == 0) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Invalid angle detected: %.2f (Count: %lu)\n",
                        raw_angle, invalid_angle_count);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            }
        }
    }
}

void SendLidarToBluetooth(float angle, float distance, uint8_t quality) {
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "Lidar: A=%.2f, D=%.2fmm, Q=%d\n",
                      angle, distance, quality);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 100);
}

void OutputAngleDataInOrder(void) {
    #define ANGLE_DATA_TIMEOUT 1000

    uint32_t current_time = HAL_GetTick();
    AngleCache* cache = &angle_cache[current_output_angle];

    if (cache->valid && (current_time - cache->update_time > ANGLE_DATA_TIMEOUT)) {
        cache->valid = 0;
    }

    if (cache->valid) {
        char buffer[32];
        int len = snprintf(buffer, sizeof(buffer),
            "Lidar: A=%d, D=%.0fmm, Q=%d\n",
            current_output_angle,
            cache->distance,
            cache->quality);
        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, 10);
    }

    current_output_angle = (current_output_angle + 1) % 360;
}


void MPU6500_Init(I2C_HandleTypeDef* hi2c) {
    uint8_t check, data;

    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, WHO_AM_I, 1, &check, 1, 100);
    if (check == 0x70) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6500 Found!\r\n", 16, 100);
    }
    else {
        char msg[30];
        snprintf(msg, sizeof(msg), "MPU6500 Not Found! ID:0x%X\r\n", check);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        Error_Handler();
    }

    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    HAL_Delay(100);

    data = GYRO_FS_SEL_500DPS;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, GYRO_CONFIG, 1, &data, 1, 100);

    data = ACCEL_FS_SEL_4G;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, ACCEL_CONFIG, 1, &data, 1, 100);

    data = 0x06;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, CONFIG, 1, &data, 1, 100);

    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, SMPLRT_DIV, 1, &data, 1, 100);
}

void MPU6500_Calibrate(I2C_HandleTypeDef* hi2c) {
    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Calibrating MPU6500...\r\n", 24, 100);

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        uint8_t buf[14];
        HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

        accel_sum_x += (int16_t)((buf[0] << 8) | buf[1]);
        accel_sum_y += (int16_t)((buf[2] << 8) | buf[3]);
        accel_sum_z += (int16_t)((buf[4] << 8) | buf[5]);
        gyro_sum_x += (int16_t)((buf[8] << 8) | buf[9]);
        gyro_sum_y += (int16_t)((buf[10] << 8) | buf[11]);
        gyro_sum_z += (int16_t)((buf[12] << 8) | buf[13]);

        HAL_Delay(5);
    }

    accel_offset_x = accel_sum_x / CALIBRATION_SAMPLES;
    accel_offset_y = accel_sum_y / CALIBRATION_SAMPLES;
    accel_offset_z = (accel_sum_z / CALIBRATION_SAMPLES) - 16384;

    gyro_offset_x = gyro_sum_x / CALIBRATION_SAMPLES;
    gyro_offset_y = gyro_sum_y / CALIBRATION_SAMPLES;
    gyro_offset_z = gyro_sum_z / CALIBRATION_SAMPLES;

    char cal_msg[128];
    snprintf(cal_msg, sizeof(cal_msg),
        "Calibration Complete:\r\n"
        "Accel Offsets: X:%d Y:%d Z:%d\r\n"
        "Gyro Offsets: X:%d Y:%d Z:%d\r\n",
        accel_offset_x, accel_offset_y, accel_offset_z,
        gyro_offset_x, gyro_offset_y, gyro_offset_z);
    HAL_UART_Transmit(&huart1, (uint8_t*)cal_msg, strlen(cal_msg), 100);
}

void MPU6500_Read_All(I2C_HandleTypeDef* hi2c) {
    uint8_t buf[14];

    HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);

    accel_x = (int16_t)((buf[0] << 8) | buf[1]) - accel_offset_x;
    accel_y = (int16_t)((buf[2] << 8) | buf[3]) - accel_offset_y;
    accel_z = (int16_t)((buf[4] << 8) | buf[5]) - accel_offset_z;

    temp = (int16_t)((buf[6] << 8) | buf[7]);

    gyro_x = (int16_t)((buf[8] << 8) | buf[9]) - gyro_offset_x;
    gyro_y = (int16_t)((buf[10] << 8) | buf[11]) - gyro_offset_y;
    gyro_z = (int16_t)((buf[12] << 8) | buf[13]) - gyro_offset_z;
}

void Calculate_Angles(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    float delta_time = (current_time - last_time) / 1000.0f;
    if (delta_time > 0.2f) delta_time = 0.01f;
    last_time = current_time;

    float ax = accel_x / 8192.0f;
    float ay = accel_y / 8192.0f;
    float az = accel_z / 8192.0f;

    float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);

    if (fabsf(accel_magnitude - 1.0f) < STATIONARY_THRESHOLD) {
        if (is_stationary == 0) {
            is_stationary = 1;
            stationary_start_time = current_time;
        }
        else if (current_time - stationary_start_time > STATIONARY_DURATION) {
            float current_bias = gyro_z / 65.5f;
            gyro_z_bias = 0.98f * gyro_z_bias + 0.02f * current_bias;
        }
    }
    else {
        is_stationary = 0;
    }

    roll = atan2f(ay, az) * 180.0f / M_PI;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

    float gyro_z_dps = (gyro_z / 65.5f) - gyro_z_bias;
    yaw += gyro_z_dps * delta_time;

    yaw = fmodf(yaw, 360.0f);
    if (yaw < 0.0f) yaw += 360.0f;
}

void Send_IMU_Data_Bluetooth(void) {
    char bluetooth_buf[256];

    int len = snprintf(bluetooth_buf, sizeof(bluetooth_buf),
        "\"angles\":[%.1f,%.1f,%.1f],\n\"accel\":[%d,%d,%d],\n\"gyro\":[%d,%d,%d]\r\n",
        yaw, roll, pitch,
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z
        );

    HAL_UART_Transmit(&huart1, (uint8_t*)bluetooth_buf, len, 100);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
HAL_NVIC_SetPriority(USART6_IRQn, 3, 0);
HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 3, 0);
HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);
HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1, 0);

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_I2C1_Init();

    HAL_Delay(100);
    char startup_msg[] = "\r\n=== STM32 STARTED (v2025-10-21) ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)startup_msg, strlen(startup_msg), 1000);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_UART_Receive_IT(&huart1, &bluetooth_rx_data, 1);

    uint8_t mpu_ok = 0;
    uint8_t mpu_id = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, WHO_AM_I, 1, &mpu_id, 1, 100) == HAL_OK && mpu_id == 0x70) {
        MPU6500_Init(&hi2c1);
        MPU6500_Calibrate(&hi2c1);
        mpu_ok = 1;
    } else {
        char msg[50];
        snprintf(msg, sizeof(msg), "MPU6500 Skip (ID:0x%02X)\r\n", mpu_id);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
        yaw = 0.0f;
    }

    gyro_z_bias = 0.0f;
    is_stationary = 0;
    stationary_start_time = 0;

    PoseController_Init(&g_pose_controller, &g_pose_config);

    Odometry_Init(&g_odometry, &g_odom_config);

    Comm_InitRxBuffer(&g_comm_rx_buffer);

    Safety_Init(&g_safety_config);
    g_safety_status.last_command_time = HAL_GetTick();

    HAL_Delay(50);
    uint8_t startCmd[] = { 0xA5, 0x20 };
    HAL_UART_Transmit(&huart6, startCmd, sizeof(startCmd), 100);
    uint8_t setScanMode[] = {0xA5, 0x60, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
HAL_UART_Transmit(&huart6, setScanMode, sizeof(setScanMode), 100);
HAL_Delay(50);
    HAL_UART_Receive_DMA(&huart6, lidar_dma_buffer, DMA_BUFFER_SIZE);

    lastEncoderA = __HAL_TIM_GET_COUNTER(&htim2);
    lastEncoderB = __HAL_TIM_GET_COUNTER(&htim4);

    pid_left.Kp = PID_KP;
    pid_left.Ki = PID_KI;
    pid_left.Kd = PID_KD;
    pid_left.integral = 0.0f;
    pid_left.prev_error = 0.0f;
    pid_left.integral_limit = PID_INTEGRAL_LIMIT;

    pid_right.Kp = PID_KP;
    pid_right.Ki = PID_KI;
    pid_right.Kd = PID_KD;
    pid_right.integral = 0.0f;
    pid_right.prev_error = 0.0f;
    pid_right.integral_limit = PID_INTEGRAL_LIMIT;

    uint32_t last_imu_time = HAL_GetTick();
    uint32_t last_debug_time = HAL_GetTick();
    uint32_t heartbeat_time = HAL_GetTick();

    while (1) {
    uint32_t current_time = HAL_GetTick();

    Safety_Update(&g_safety_status);
    if (g_safety_status.estop_active || g_safety_status.watchdog_expired) {
        static uint32_t last_warn = 0;
        if (current_time - last_warn >= 5000) {
            char warn[80];
            snprintf(warn, sizeof(warn), "[WARN] Safety block! estop=%d wd=%d last_cmd=%lu\r\n",
                    g_safety_status.estop_active, g_safety_status.watchdog_expired,
                    g_safety_status.last_command_time);
            HAL_UART_Transmit(&huart1, (uint8_t*)warn, strlen(warn), 100);
            last_warn = current_time;
        }
        Safety_EmergencyStop();
        HAL_Delay(5);
        continue;
    }

    if (current_time - heartbeat_time >= 1000) {
        char hb[30];
        snprintf(hb, sizeof(hb), "HB:%lu\r\n", current_time);
        HAL_UART_Transmit(&huart1, (uint8_t*)hb, strlen(hb), 100);
        heartbeat_time = current_time;
    }

    SendConnectionNotification();

    static uint32_t last_odom_update = 0;
    static int32_t last_encoderA = 0;
    static int32_t last_encoderB = 0;

    if (current_time - last_odom_update >= 33) {
        int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);

        int32_t deltaA = encoderA - last_encoderA;
        int32_t deltaB = encoderB - last_encoderB;

        if (deltaA > 32768) deltaA -= 65536;
        else if (deltaA < -32768) deltaA += 65536;

        if (deltaB > 32768) deltaB -= 65536;
        else if (deltaB < -32768) deltaB += 65536;

        static int32_t accum_encoderA = 0;
        static int32_t accum_encoderB = 0;
        accum_encoderA += deltaA;
        accum_encoderB += deltaB;

        last_encoderA = encoderA;
        last_encoderB = encoderB;

        Odometry_Update(&g_odometry, accum_encoderA, accum_encoderB);

        OdometryData_t odom_data;
        odom_data.timestamp = current_time;
        Odometry_GetPose(&g_odometry, &odom_data.x, &odom_data.y, &odom_data.theta);
        Odometry_GetVelocity(&g_odometry, &odom_data.v_linear, &odom_data.v_angular);
        Comm_SendOdometry(&huart1, &odom_data);

        static uint32_t last_encoder_debug = 0;
        if (current_time - last_encoder_debug >= 1000) {
            char dbg[120];
            snprintf(dbg, sizeof(dbg), "[ENC-DBG] A=%ld B=%ld | Odom: x=%.3f y=%.3f θ=%.3f | V: lin=%.3f ang=%.3f\r\n",
                    encoderA, encoderB, odom_data.x, odom_data.y, odom_data.theta,
                    odom_data.v_linear, odom_data.v_angular);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
            last_encoder_debug = current_time;
        }

        PoseController_UpdateCurrent(&g_pose_controller,
                                    odom_data.x, odom_data.y, odom_data.theta);

        last_odom_update = current_time;
    }

    static uint32_t last_pose_ctrl = 0;
    if (current_time - last_pose_ctrl >= 20 && g_pose_controller.is_active) {
        float dt = 0.02f;
        MotorOutput_t motor_out = PoseController_Update(&g_pose_controller, dt);

        if (!PoseController_IsReached(&g_pose_controller)) {
            uint16_t left_pwm = Safety_LimitPWM(fabsf(motor_out.left_speed), g_safety_config.max_pwm);
            uint16_t right_pwm = Safety_LimitPWM(fabsf(motor_out.right_speed), g_safety_config.max_pwm);

            if (motor_out.direction == 1) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
            } else if (motor_out.direction == 2) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
            } else {
                if (motor_out.left_speed > 0) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, left_pwm);
                } else {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_pwm);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                }
                if (motor_out.right_speed > 0) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_pwm);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                } else {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pwm);
                }
            }
        } else {
            Safety_EmergencyStop();
            PoseController_Enable(&g_pose_controller, 0);
            HAL_UART_Transmit(&huart1, (uint8_t*)"Pose reached!\r\n", 15, 100);
        }
        last_pose_ctrl = current_time;
    }

    if (current_time - last_speed_update_time >= SPEED_UPDATE_INTERVAL) {
        UpdateSpeedRamp();
        last_speed_update_time = current_time;
    }

    if (auto_turn_mode != 0) {
        HandleAutoTurn();
    }

    if (mpu_ok && current_time - last_imu_time >= 100) {
        MPU6500_Read_All(&hi2c1);
        Calculate_Angles();

        ImuData_t imu_data;
        imu_data.timestamp = current_time;
        imu_data.yaw = yaw;
        imu_data.roll = roll;
        imu_data.pitch = pitch;
        imu_data.accel[0] = accel_x;
        imu_data.accel[1] = accel_y;
        imu_data.accel[2] = accel_z;
        imu_data.gyro[0] = gyro_x;
        imu_data.gyro[1] = gyro_y;
        imu_data.gyro[2] = gyro_z;
        Comm_SendImuData(&huart1, &imu_data);

        last_imu_time = current_time;
    }

    if (current_time - last_encoder_time >= 50) {
        int32_t encoderA = __HAL_TIM_GET_COUNTER(&htim2);
        int32_t encoderB = __HAL_TIM_GET_COUNTER(&htim4);
        int32_t deltaA = encoderA - lastEncoderA;
        int32_t deltaB = encoderB - lastEncoderB;
        lastEncoderA = encoderA;
        lastEncoderB = encoderB;

        if (current_direction == DIR_FORWARD || current_direction == DIR_BACKWARD) {
            float rpmA = (float)deltaA / PPR * (60000.0f / 50);
            float rpmB = (float)deltaB / PPR * (60000.0f / 50);

            float pid_output_left = PID_Update(&pid_left, target_rpm, rpmA);
            float pid_output_right = PID_Update(&pid_right, target_rpm, rpmB);

            pwm_left = fmaxf(0, fminf(current_speed + pid_output_left, MAX_SPEED));
            pwm_right = fmaxf(0, fminf(current_speed + pid_output_right, MAX_SPEED_RIGHT));

            if (current_time - last_debug_time > 200) {
                snprintf(uart_buf, sizeof(uart_buf),
                    "PID: Tgt=%.1f, RPM_A=%.1f, RPM_B=%.1f, PWM_L=%d, PWM_R=%d\n",
                    target_rpm, rpmA, rpmB, (int)pwm_left, (int)pwm_right);
                HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), 100);
                last_debug_time = current_time;
            }
        }
        last_encoder_time = current_time;
    }

    if (lidar_process_packet) {
        lidar_process_packet = 0;
        uint16_t data_length = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        uint8_t processed_packets = 0;

        for (uint16_t i = 0; i <= data_length - sizeof(LidarDataPacket) && processed_packets < 5; i++) {
            if (lidar_dma_buffer[i] & 0x80) {
                LidarDataPacket packet;
                memcpy(&packet, &lidar_dma_buffer[i], sizeof(LidarDataPacket));
                ProcessLidarData((uint8_t*)&packet);
                i += sizeof(LidarDataPacket) - 1;
                processed_packets++;
            }
        }

        __HAL_DMA_DISABLE(huart6.hdmarx);
        huart6.hdmarx->Instance->NDTR = DMA_BUFFER_SIZE;
        __HAL_DMA_ENABLE(huart6.hdmarx);
    }

    if (current_time - last_output_time >= 10) {
        OutputAngleDataInOrder();
        last_output_time = current_time;
    }

    HAL_Delay(5);
}
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */