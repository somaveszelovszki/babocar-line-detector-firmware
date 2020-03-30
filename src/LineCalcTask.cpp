#include <micro/utils/log.hpp>
#include <micro/task/common.hpp>

#include <cfg_board.h>
#include <LinePanel.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

#define MEASUREMENTS_QUEUE_LENGTH 1
QueueHandle_t measurementsQueue;
static uint8_t measurementsQueueStorageBuffer[MEASUREMENTS_QUEUE_LENGTH * sizeof(measurements_t)];
static StaticQueue_t measurementsQueueBuffer;

extern "C" void runLineCalcTask(void) {

    measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_LENGTH, sizeof(measurements_t), measurementsQueueStorageBuffer, &measurementsQueueBuffer);

    while (true) {

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}

void micro_Command_Uart_RxCpltCallback(void) {
    //panelLink_onNewRxData((panelLink_t*)&panelLink);
}
