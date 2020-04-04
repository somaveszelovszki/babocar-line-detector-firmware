#include <cfg_board.h>
#include <globals.hpp>
#include <SensorHandler.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern QueueHandle_t measurementsQueue;

#define LEDS_QUEUE_LENGTH 1
QueueHandle_t ledsQueue;
static uint8_t ledsQueueStorageBuffer[LEDS_QUEUE_LENGTH * sizeof(leds_t)];
static StaticQueue_t ledsQueueBuffer;

SemaphoreHandle_t lineCalcSemaphore;

namespace {

SensorHandler sensorHandler;
StaticSemaphore_t lineCalcSemaphoreBuffer;

} // namespace

extern "C" void runSensorTask(void) {

    sensorHandler.initialize();

    ledsQueue = xQueueCreateStatic(LEDS_QUEUE_LENGTH, sizeof(leds_t), ledsQueueStorageBuffer, &ledsQueueBuffer);

    lineCalcSemaphore = xSemaphoreCreateBinaryStatic(&lineCalcSemaphoreBuffer);
    xSemaphoreGive(lineCalcSemaphore);

    measurements_t measurements;
    leds_t leds;

    while (true) {
        vTaskSuspendAll();
        const uint8_t first = globals::scanRangeRadius > 0 ? globals::scanRangeCenter - globals::scanRangeRadius : 0;
        const uint8_t last  = globals::scanRangeRadius > 0 ? globals::scanRangeCenter + globals::scanRangeRadius : ARRAY_SIZE(measurements) - 1;
        xTaskResumeAll();

        sensorHandler.readSensors(measurements, first, last);
        xSemaphoreTake(lineCalcSemaphore, portMAX_DELAY);
        xQueueSend(measurementsQueue, &measurements, 0);
    }

    vTaskDelete(nullptr);
}

extern void spi_SensorTxCpltCallback() {
    sensorHandler.onTxFinished();
}

extern void spi_SensorTxRxCpltCallback() {
    sensorHandler.onTxFinished();
}
