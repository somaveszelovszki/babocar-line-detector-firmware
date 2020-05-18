#pragma once

#include <micro/container/vec.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/spi.hpp>
#include <micro/port/task.hpp>

#include <SensorHandlerData.hpp>

class SensorHandler {
public:
    SensorHandler(SPI_HandleTypeDef *hspi,
        const micro::vec<micro::gpio_t, cfg::NUM_SENSORS / 8>& adcEnPins,
        const micro::gpio_t& LE_opto,
        const micro::gpio_t& OE_opto,
        const micro::gpio_t& LE_ind,
        const micro::gpio_t& OE_ind);

    void initialize();

    void readSensors(measurements_t& OUT measurements, const uint8_t first, const uint8_t last);
    void writeLeds(const leds_t& leds);

    void onTxFinished();

private:
    uint8_t readAdc(const uint8_t channel);

private:
    micro::semaphore_t semaphore_;
    SPI_HandleTypeDef * const hspi_;
    const micro::vec<micro::gpio_t, cfg::NUM_SENSORS / 8> adcEnPins_;
    const micro::gpio_t LE_opto_;
    const micro::gpio_t OE_opto_;
    const micro::gpio_t LE_ind_;
    const micro::gpio_t OE_ind_;
};
