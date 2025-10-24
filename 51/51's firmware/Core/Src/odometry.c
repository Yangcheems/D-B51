/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : odometry.c
  ******************************************************************************
  */
/* USER CODE END Header */

#include "odometry.h"
#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846f

const OdometryConfig_t DEFAULT_ODOM_CONFIG = {
    .wheel_radius = 0.0325f,
    .wheel_base = 0.165f,
    .encoder_ppr = 360,
    .gear_ratio = 1.0f
};

static OdometryConfig_t g_odom_config;

void Odometry_Init(Odometry_t* odom, const OdometryConfig_t* config) {
    memset(odom, 0, sizeof(Odometry_t));

    if (config != NULL) {
        memcpy(&g_odom_config, config, sizeof(OdometryConfig_t));
    } else {
        memcpy(&g_odom_config, &DEFAULT_ODOM_CONFIG, sizeof(OdometryConfig_t));
    }

    odom->last_update_time = HAL_GetTick();
}

void Odometry_Update(Odometry_t* odom, int32_t encoder_left, int32_t encoder_right) {
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - odom->last_update_time) / 1000.0f;

    if (dt < 0.001f) {
        return;
    }

    int32_t delta_left = encoder_left - odom->last_encoder_left;
    int32_t delta_right = encoder_right - odom->last_encoder_right;

    if (delta_left > 32768) delta_left -= 65536;
    if (delta_left < -32768) delta_left += 65536;
    if (delta_right > 32768) delta_right -= 65536;
    if (delta_right < -32768) delta_right += 65536;

    odom->last_encoder_left = encoder_left;
    odom->last_encoder_right = encoder_right;
    odom->last_update_time = current_time;

    float pulses_per_meter = g_odom_config.encoder_ppr * g_odom_config.gear_ratio /
                             (2.0f * M_PI * g_odom_config.wheel_radius);
    float dist_left = delta_left / pulses_per_meter;
    float dist_right = delta_right / pulses_per_meter;

    float dist_center = (dist_left + dist_right) / 2.0f;
    float delta_theta = (dist_right - dist_left) / g_odom_config.wheel_base;

    odom->v_linear = dist_center / dt;
    odom->v_angular = delta_theta / dt;

    float theta_mid = odom->theta + delta_theta / 2.0f;
    odom->x += dist_center * cosf(theta_mid);
    odom->y += dist_center * sinf(theta_mid);
    odom->theta += delta_theta;

    while (odom->theta > M_PI) odom->theta -= 2.0f * M_PI;
    while (odom->theta < -M_PI) odom->theta += 2.0f * M_PI;
}

void Odometry_Reset(Odometry_t* odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
    odom->v_linear = 0.0f;
    odom->v_angular = 0.0f;
}

void Odometry_SetPose(Odometry_t* odom, float x, float y, float theta) {
    odom->x = x;
    odom->y = y;
    odom->theta = theta;
}

void Odometry_GetPose(const Odometry_t* odom, float* x, float* y, float* theta) {
    *x = odom->x;
    *y = odom->y;
    *theta = odom->theta;
}

void Odometry_GetVelocity(const Odometry_t* odom, float* v_linear, float* v_angular) {
    *v_linear = odom->v_linear;
    *v_angular = odom->v_angular;
}