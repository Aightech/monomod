#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20948.h"
#include "network_manager.hpp"

/**
 * @brief Start the IMU streaming task
 *
 * Polls ICM-20948 at configured rate, packs raw accel + gyro into
 * AXON_TYPE_IMU packets, sends via UDP.
 *
 * @param imu ICM-20948 driver handle (already initialized)
 * @param net NetworkManager for UDP sending
 * @param rate_hz Polling rate (default 100 Hz)
 * @param handle Output task handle
 */
void imu_task_start(icm20948_t *imu, NetworkManager *net,
                    int rate_hz, TaskHandle_t *handle);

void imu_task_stop(TaskHandle_t handle);
