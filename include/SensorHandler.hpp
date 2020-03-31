#pragma once

#include <micro/utils/types.hpp>

#include <cfg_sensor.hpp>

#include <FreeRTOS.h>
#include <semphr.h>

typedef uint8_t measurements_t[cfg::NUM_SENSORS];

class SensorHandler {
public:
    void initialize();

    void readSensors(measurements_t& OUT measurements, const uint8_t first, const uint8_t last);
    void writeLeds(const bool * const leds);

    void onTxFinished() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(this->semaphore_, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

private:
    uint8_t readAdc(const uint8_t channel);

private:
    SemaphoreHandle_t semaphore_;
    StaticSemaphore_t semaphoreBuffer_;
};
