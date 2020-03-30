#include <micro/utils/log.hpp>
#include <micro/task/common.hpp>

#include <cfg_board.h>
#include <SensorHandler.hpp>
#include <LinePosCalculator.hpp>
#include <LineFilter.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern SemaphoreHandle_t lineCalcSemaphore;

#define MEASUREMENTS_QUEUE_LENGTH 1
QueueHandle_t measurementsQueue;
static uint8_t measurementsQueueStorageBuffer[MEASUREMENTS_QUEUE_LENGTH * sizeof(measurements_t)];
static StaticQueue_t measurementsQueueBuffer;

namespace {

LinePosCalculator linePosCalc;
LineFilter lineFilter;

} // namespace

extern "C" void runLineCalcTask(void) {

    measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_LENGTH, sizeof(measurements_t), measurementsQueueStorageBuffer, &measurementsQueueBuffer);

    measurements_t measurements;

    while (true) {
        if (xQueueReceive(measurementsQueue, &measurements, 0)) {
            const linePositions_t linePositions = linePosCalc.calculate(measurements);
            const trackedLines_t trackedLines = lineFilter.update(linePositions);
            // TODO send back on panel link
            xSemaphoreGive(lineCalcSemaphore);
        }
    }

    vTaskDelete(nullptr);
}

void uart_Command_RxCpltCallback(void) {
    //panelLink_onNewRxData((panelLink_t*)&panelLink);
}
