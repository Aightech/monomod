#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ads1293.h"
#include "ina_emg.h"
#include "ring_buffer.hpp"

/**
 * @brief Start the ADS1293 acquisition task (SPI polling, 3 ch × 24 bit).
 */
void acq_task_start(ads1293_t *adc, MonomodRingBuffer *ring,
                    SemaphoreHandle_t tx_sem, TaskHandle_t *handle);

/** @brief Stop the ADS1293 acquisition task */
void acq_task_stop(TaskHandle_t handle);

/**
 * @brief Start the INA (analog ADC) acquisition task (1 ch × 16 bit).
 *
 * Pulls samples from the ESP32-C3 ADC continuous driver and pushes them
 * into the ring buffer as SampleFrames.
 */
void acq_task_start_ina(ina_emg_t *ina, MonomodRingBuffer *ring,
                        SemaphoreHandle_t tx_sem, TaskHandle_t *handle);

/** @brief Stop the INA acquisition task */
void acq_task_stop_ina(TaskHandle_t handle);
