/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pose_controller.c
  ******************************************************************************
  */
/* USER CODE END Header */

#include "pose_controller.h"
#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846f
#define WHEEL_BASE 0.20f
#define MAX_PWM 400.0f

const PoseControllerConfig_t DEFAULT_POSE_CONFIG = {
    .linear_kp = 200.0f,
    .linear_ki = 5.0f,
    .linear_kd = 10.0f,
    .angular_kp = 150.0f,
    .angular_ki = 3.0f,
    .angular_kd = 8.0f,
    .tolerance_pos = 0.05f,
    .tolerance_ang = 3.0f,
    .max_linear_vel = 0.5f,
    .max_angular_vel = 1.5f
};

static PoseControllerConfig_t g_config;

static float NormalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void PoseController_Init(PoseController_t* ctrl, PoseControllerConfig_t* config) {
    memset(ctrl, 0, sizeof(PoseController_t));

    if (config != NULL) {
        memcpy(&g_config, config, sizeof(PoseControllerConfig_t));
    } else {
        memcpy(&g_config, &DEFAULT_POSE_CONFIG, sizeof(PoseControllerConfig_t));
    }
}

void PoseController_SetTarget(PoseController_t* ctrl, float x, float y, float theta) {
    ctrl->target_pose.x = x;
    ctrl->target_pose.y = y;
    ctrl->target_pose.theta = theta;
    ctrl->reached = 0;

    ctrl->linear_integral = 0.0f;
    ctrl->angular_integral = 0.0f;
}

void PoseController_UpdateCurrent(PoseController_t* ctrl, float x, float y, float theta) {
    ctrl->current_pose.x = x;
    ctrl->current_pose.y = y;
    ctrl->current_pose.theta = theta;
}

void PoseController_Enable(PoseController_t* ctrl, uint8_t enable) {
    ctrl->is_active = enable;
    if (!enable) {
        ctrl->linear_integral = 0.0f;
        ctrl->angular_integral = 0.0f;
    }
}

uint8_t PoseController_IsReached(PoseController_t* ctrl) {
    float dx = ctrl->target_pose.x - ctrl->current_pose.x;
    float dy = ctrl->target_pose.y - ctrl->current_pose.y;
    float distance = sqrtf(dx * dx + dy * dy);
    float angle_diff = fabsf(NormalizeAngle(ctrl->target_pose.theta - ctrl->current_pose.theta));

    if (distance < g_config.tolerance_pos && angle_diff < g_config.tolerance_ang) {
        ctrl->reached = 1;
        return 1;
    }
    return 0;
}

MotorOutput_t PoseController_Update(PoseController_t* ctrl, float dt) {
    MotorOutput_t output = {0};

    if (!ctrl->is_active || ctrl->reached) {
        return output;
    }

    float dx = ctrl->target_pose.x - ctrl->current_pose.x;
    float dy = ctrl->target_pose.y - ctrl->current_pose.y;
    float distance = sqrtf(dx * dx + dy * dy);

    float target_heading = atan2f(dy, dx) * 180.0f / M_PI;

    float heading_error = NormalizeAngle(target_heading - ctrl->current_pose.theta);

    float final_angle_error = NormalizeAngle(ctrl->target_pose.theta - ctrl->current_pose.theta);

    float linear_velocity = 0.0f;
    float angular_velocity = 0.0f;

    if (distance < g_config.tolerance_pos) {
        if (fabsf(final_angle_error) > g_config.tolerance_ang) {
            ctrl->angular_integral += final_angle_error * dt;
            ctrl->angular_integral = Clamp(ctrl->angular_integral, -50.0f, 50.0f);

            float angular_derivative = (final_angle_error - ctrl->prev_angular_error) / dt;
            ctrl->prev_angular_error = final_angle_error;

            angular_velocity = g_config.angular_kp * final_angle_error +
                             g_config.angular_ki * ctrl->angular_integral +
                             g_config.angular_kd * angular_derivative;
            angular_velocity = Clamp(angular_velocity, -g_config.max_angular_vel, g_config.max_angular_vel);
        } else {
            ctrl->reached = 1;
        }
    } else if (fabsf(heading_error) > 30.0f) {
        ctrl->angular_integral += heading_error * dt;
        ctrl->angular_integral = Clamp(ctrl->angular_integral, -50.0f, 50.0f);

        float angular_derivative = (heading_error - ctrl->prev_angular_error) / dt;
        ctrl->prev_angular_error = heading_error;

        angular_velocity = g_config.angular_kp * heading_error +
                         g_config.angular_ki * ctrl->angular_integral +
                         g_config.angular_kd * angular_derivative;
        angular_velocity = Clamp(angular_velocity, -g_config.max_angular_vel, g_config.max_angular_vel);
    } else {
        ctrl->linear_integral += distance * dt;
        ctrl->linear_integral = Clamp(ctrl->linear_integral, -20.0f, 20.0f);

        float linear_derivative = (distance - ctrl->prev_linear_error) / dt;
        ctrl->prev_linear_error = distance;

        linear_velocity = g_config.linear_kp * distance +
                         g_config.linear_ki * ctrl->linear_integral +
                         g_config.linear_kd * linear_derivative;
        linear_velocity = Clamp(linear_velocity, 0.0f, g_config.max_linear_vel);

        angular_velocity = g_config.angular_kp * 0.15f * heading_error;
        angular_velocity = Clamp(angular_velocity, -g_config.max_angular_vel * 0.3f,
                                g_config.max_angular_vel * 0.3f);
    }

    float v_left = linear_velocity - (angular_velocity * WHEEL_BASE / 2.0f);
    float v_right = linear_velocity + (angular_velocity * WHEEL_BASE / 2.0f);

    output.left_speed = v_left * MAX_PWM / g_config.max_linear_vel;
    output.right_speed = v_right * MAX_PWM / g_config.max_linear_vel;

    output.left_speed = Clamp(output.left_speed, -MAX_PWM, MAX_PWM);
    output.right_speed = Clamp(output.right_speed, -MAX_PWM, MAX_PWM);

    if (output.left_speed > 0 && output.right_speed > 0) {
        output.direction = 1;
    } else if (output.left_speed < 0 && output.right_speed < 0) {
        output.direction = 2;
        output.left_speed = -output.left_speed;
        output.right_speed = -output.right_speed;
    } else {
        output.direction = 3;
    }

    return output;
}

void PoseController_Reset(PoseController_t* ctrl) {
    ctrl->linear_integral = 0.0f;
    ctrl->angular_integral = 0.0f;
    ctrl->prev_linear_error = 0.0f;
    ctrl->prev_angular_error = 0.0f;
    ctrl->reached = 0;
}