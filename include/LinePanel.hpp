#pragma once

#include <micro/utils/types.hpp>

#include <cfg_sensor.hpp>

typedef uint8_t measurements_t[cfg::NUM_SENSORS];

class SensorHandler {
public:
    SensorHandler();

    void readSensors(measurements_t& OUT measurements);

    uint8_t readAdc(const uint8_t channel);

    void writeLeds(const bool * const leds);
};
