/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : safety.c
  ******************************************************************************
  */
/* USER CODE END Header */

#include "safety.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;

const SafetyConfig_t DEFAULT_SAFETY_CONFIG = {
    .max_linear_vel = 0.8f,
    .max_angular_vel = 2.0f,
    .max_pwm = 400,
    .estop_gpio_pin = GPIO_PIN_0,
    .estop_gpio_port = GPIOA,
    .watchdog_timeout_ms = 1000
};

static SafetyConfig_t g_safety_config;

void Safety_Init(SafetyConfig_t* config) {
    if (config != NULL) {
        g_safety_config = *config;
    } else {
        g_safety_config = DEFAULT_SAFETY_CONFIG;
    }
}

uint8_t Safety_CheckEstop(void) {
    return 0;
}

uint8_t Safety_CheckWatchdog(SafetyStatus_t* status) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - status->last_command_time > g_safety_config.watchdog_timeout_ms) {
        status->watchdog_expired = 1;
        return 1;
    }
    status->watchdog_expired = 0;
    return 0;
}

void Safety_FeedWatchdog(SafetyStatus_t* status) {
    status->last_command_time = HAL_GetTick();
    status->watchdog_expired = 0;
}

void Safety_EmergencyStop(void) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void Safety_Update(SafetyStatus_t* status) {
    if (Safety_CheckEstop()) {
        if (!status->estop_active) {
            status->estop_active = 1;
            status->estop_count++;
            Safety_EmergencyStop();
        }
    } else {
        status->estop_active = 0;
    }

    if (Safety_CheckWatchdog(status)) {
        Safety_EmergencyStop();
    }
}

float Safety_LimitSpeed(float speed, float max_speed) {
    if (speed > max_speed) return max_speed;
    if (speed < -max_speed) return -max_speed;
    return speed;
}

uint16_t Safety_LimitPWM(float pwm, uint16_t max_pwm) {
    if (pwm > (float)max_pwm) return max_pwm;
    if (pwm < 0.0f) return 0;
    return (uint16_t)pwm;
}