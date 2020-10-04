#include <micro/math/numeric.hpp>

#include <cfg_sensor.hpp>
#include <SensorHandler.hpp>

#include <utility>

using namespace micro;

namespace {

constexpr uint8_t OPTO_BUFFERS[8][cfg::NUM_SENSORS / 8] = { // selects every 8th optical sensor
    { 1,   1,   1,   1,   1,   1   },
    { 2,   2,   2,   2,   2,   2   },
    { 4,   4,   4,   4,   4,   4   },
    { 8,   8,   8,   8,   8,   8   },
    { 16,  16,  16,  16,  16,  16  },
    { 32,  32,  32,  32,  32,  32  },
    { 64,  64,  64,  64,  64,  64  },
    { 128, 128, 128, 128, 128, 128 }
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
    for (uint8_t optoIdx = 0; optoIdx < 8; ++optoIdx) {

        this->exchangeData((uint8_t*)OPTO_BUFFERS[optoIdx], nullptr, cfg::NUM_SENSORS / 8);

        gpio_write(this->OE_opto_, gpioPinState_t::SET);
        gpio_write(this->LE_opto_, gpioPinState_t::SET);
        gpio_write(this->LE_opto_, gpioPinState_t::RESET);
        gpio_write(this->OE_opto_, gpioPinState_t::RESET);

        for (uint8_t adcIdx = 0; adcIdx < cfg::NUM_SENSORS / 8; ++adcIdx) {
            const uint8_t pos = adcIdx * 8 + optoIdx;

            if (micro::isBtw(pos, scanRange.first, scanRange.second)) {
                const gpio_t& adcEnPin = this->adcEnPins_[adcIdx];

                gpio_write(adcEnPin, gpioPinState_t::RESET);
                measurements[pos] = this->readAdc(optoIdx);
                gpio_write(adcEnPin, gpioPinState_t::SET);
            }
        }
    }
}

void SensorHandler::writeLeds(const Leds& leds) {
    uint8_t outBuffer[2 * cfg::NUM_SENSORS / 8] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    for (uint32_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        if (leds.get(i)) {
            outBuffer[(cfg::NUM_SENSORS + i) / 8] |= (1 << (i % 8));
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
