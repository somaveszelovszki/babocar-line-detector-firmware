#include <micro/math/numeric.hpp>

#include <cfg_sensor.hpp>
#include <SensorHandler.hpp>

#include <utility>

using namespace micro;

namespace {

constexpr uint8_t SENSOR_POSITIONS[16] = {
    0, 8,  4, 12,
    1, 9,  5, 13,
    2, 10, 6, 14,
    3, 11, 7, 15
};

constexpr uint8_t SENSOR_SELECTORS[16][cfg::NUM_SENSORS / 8] = {
    { 0,   1,   0,   1,   0,   1   },
    { 0,   2,   0,   2,   0,   2   },
    { 0,   4,   0,   4,   0,   4   },
    { 0,   8,   0,   8,   0,   8   },
    { 0,   16,  0,   16,  0,   16  },
    { 0,   32,  0,   32,  0,   32  },
    { 0,   64,  0,   64,  0,   64  },
    { 0,   128, 0,   128, 0,   128 },
    { 1,   0,   1,   0,   1,   0   },
    { 2,   0,   2,   0,   2,   0   },
    { 4,   0,   4,   0,   4,   0   },
    { 8,   0,   8,   0,   8,   0   },
    { 16,  0,   16,  0,   16,  0   },
    { 32,  0,   32,  0,   32,  0   },
    { 64,  0,   64,  0,   64,  0   },
    { 128, 0,   128, 0,   128, 0   }
};

} // namespace

SensorHandler::SensorHandler(const spi_t& spi,
    const micro::vec<micro::gpio_t, cfg::NUM_SENSORS / 8>& adcEnPins,
    const micro::gpio_t& LE_opto,
    const micro::gpio_t& OE_opto,
    const micro::gpio_t& LE_ind,
    const micro::gpio_t& OE_ind)
    : spi_(spi)
    , adcEnPins_(adcEnPins)
    , LE_opto_(LE_opto)
    , OE_opto_(OE_opto)
    , LE_ind_(LE_ind)
    , OE_ind_(OE_ind) {}

void SensorHandler::initialize() {
    gpio_write(this->LE_opto_, gpioPinState_t::RESET);
    gpio_write(this->OE_opto_, gpioPinState_t::SET);

    gpio_write(this->LE_ind_,  gpioPinState_t::RESET);
    gpio_write(this->OE_ind_,  gpioPinState_t::SET);

    for (const gpio_t& adcEnPin : this->adcEnPins_) {
        gpio_write(adcEnPin, gpioPinState_t::SET);
    }
}

void SensorHandler::readSensors(Measurements& OUT measurements, const std::pair<uint8_t, uint8_t>& scanRange) {

    for (uint8_t i = 0; i < 16; ++i) {
        const uint8_t optoIdx = SENSOR_POSITIONS[i];
        this->exchangeData(SENSOR_SELECTORS[optoIdx], nullptr, cfg::NUM_SENSORS / 8);

        gpio_write(this->LE_opto_, gpioPinState_t::SET);
        gpio_write(this->LE_opto_, gpioPinState_t::RESET);
        gpio_write(this->OE_opto_, gpioPinState_t::RESET);

        for (volatile uint32_t t = 0; t < 800; ++t) {} // waits between the LED light-up and the ADC read

        for (uint8_t adcIdx = optoIdx / 8; adcIdx < cfg::NUM_SENSORS / 8; adcIdx += 2) {
            const uint8_t absPos = adcIdx * 8 + (optoIdx % 8);

            if (micro::isBtw(absPos, scanRange.first, scanRange.second)) {
                const gpio_t& adcEnPin = this->adcEnPins_[adcIdx];

                gpio_write(adcEnPin, gpioPinState_t::RESET);
                measurements[absPos] = this->readAdc(optoIdx);
                gpio_write(adcEnPin, gpioPinState_t::SET);
            }
        }

        gpio_write(this->OE_opto_, gpioPinState_t::SET);
    }
}

void SensorHandler::writeLeds(const Leds& leds) {
    uint8_t outBuffer[cfg::NUM_SENSORS / 8] = { 0, 0, 0, 0, 0, 0 };

    for (uint32_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        if (leds[i]) {
            outBuffer[i / 8] |= (1 << (8 - (i % 8) - 1));
        }
    }

    this->exchangeData(outBuffer, nullptr, ARRAY_SIZE(outBuffer));

    gpio_write(this->OE_ind_, gpioPinState_t::SET);
    gpio_write(this->LE_ind_, gpioPinState_t::SET);
    gpio_write(this->LE_ind_, gpioPinState_t::RESET);
    gpio_write(this->OE_ind_, gpioPinState_t::RESET);
}

void SensorHandler::onTxFinished() {
    this->semaphore_.give();
}

uint8_t SensorHandler::readAdc(const uint8_t channel) {
    uint8_t adcBuffer[3] = { 0, 0, 0 };

    // Control byte: | START | SEL2 | SEL1 | SEL0 | UNI/BIP | SGL/DIF | PD1 | PD0 |
    // Select bits (according to the datasheet):
    //      SEL2    -   channel's 1st bit (LSB)
    //      SEL1    -   channel's 3rd bit
    //      SEL0    -   channel's 2nd bit
    //
    // @see MAX1110CAP+ datasheet for details
    adcBuffer[0] =  0b10001111 | ((channel & 0b00000001) << 6) | ((channel & 0b00000010) << 3) | ((channel & 0b00000100) << 3);
    this->exchangeData(adcBuffer, adcBuffer, ARRAY_SIZE(adcBuffer));
    return (adcBuffer[1] << 2) | (adcBuffer[2] >> 6); // ADC value format: 00000000 00XXXXXX XX000000
}

void SensorHandler::exchangeData(const uint8_t *txBuf, uint8_t *rxBuf, const uint32_t size) {
    micro::spi_exchange(this->spi_, txBuf, rxBuf, size);
    this->semaphore_.take(millisecond_t(2));
}
