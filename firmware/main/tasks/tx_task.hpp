#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ring_buffer.hpp"
#include "network_manager.hpp"

/**
 * @brief Start the transmission task
 *
 * Waits on semaphore, drains ring buffer, builds UDP data packets,
 * sends via NetworkManager. Batches samples into packets with 3ms deadline.
 */
void tx_task_start(MonomodRingBuffer *ring, NetworkManager *net,
                   SemaphoreHandle_t sem, TaskHandle_t *handle);

void tx_task_stop(TaskHandle_t handle);
