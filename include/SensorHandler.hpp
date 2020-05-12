#pragma once

#include <micro/port/task.hpp>

#include <SensorHandlerData.hpp>

typedef uint8_t measurements_t[cfg::NUM_SENSORS];
typedef micro::bit_array<cfg::NUM_SENSORS> leds_t;

class SensorHandler {
public:
    void initialize();

    void readSensors(measurements_t& OUT measurements, const uint8_t first, const uint8_t last);
    void writeLeds(const leds_t& leds);

    void onTxFinished();

private:
    uint8_t readAdc(const uint8_t channel);

private:
    micro::semaphore_t semaphore_;
};
