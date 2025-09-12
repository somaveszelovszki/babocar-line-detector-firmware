#pragma once

#include <SensorData.hpp>
#include <utility>

#include <micro/container/vector.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/spi.hpp>

class SensorHandler {
  public:
    SensorHandler(const micro::spi_t& spi,
                  const micro::vector<micro::gpio_t, cfg::NUM_SENSORS / 8>& adcEnPins,
                  const micro::gpio_t& LE_opto, const micro::gpio_t& OE_opto,
                  const micro::gpio_t& LE_ind, const micro::gpio_t& OE_ind);

    void initialize();

    void readSensors(Measurements& OUT measurements, const std::pair<uint8_t, uint8_t>& scanRange);
    void writeLeds(const Leds& leds);

    void onTxFinished();

  private:
    uint8_t readAdc(const uint8_t channel);
    void exchangeData(const uint8_t* txBuf, uint8_t* rxBuf, const uint32_t size);

  private:
    micro::semaphore_t semaphore_;
    const micro::spi_t spi_;
    const micro::vector<micro::gpio_t, cfg::NUM_SENSORS / 8> adcEnPins_;
    const micro::gpio_t LE_opto_;
    const micro::gpio_t OE_opto_;
    const micro::gpio_t LE_ind_;
    const micro::gpio_t OE_ind_;
};
