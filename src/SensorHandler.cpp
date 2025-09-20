#include <SensorHandler.hpp>
#include <cfg_sensor.hpp>
#include <utility>

#include <micro/container/vector.hpp>
#include <micro/math/numeric.hpp>

using namespace micro;

/**
 * Sensor Lighting Mode Configuration
 *
 * 6 sensor mode:
 * - Lights up 6 sensors at once (every 8th sensor across the 48-sensor array)
 * - Creates 8 groups of 6 sensors each:
 *   Group 0: sensors 0, 8, 16, 24, 32, 40
 *   Group 1: sensors 1, 9, 17, 25, 33, 41
 *   ...and so on
 * - Requires 8 iterations to read all 48 sensors
 *
 * 3 sensor mode:
 * - Lights up 3 sensors at once (every 16th sensor across the 48-sensor array)
 * - Creates 16 groups of 3 sensors each:
 *   Group 0: sensors 0, 16, 32
 *   Group 1: sensors 1, 17, 33
 *   ...and so on
 * - Requires 16 iterations to read all 48 sensors
 */
#define NUM_PARALLEL_SENSORS 6

constexpr uint8_t NUM_ITERATIONS = cfg::NUM_SENSORS / NUM_PARALLEL_SENSORS;

namespace {
#if NUM_PARALLEL_SENSORS == 6

// Optimized for highest minimum distance between two measurements: 3
constexpr uint8_t SENSOR_GROUPS[NUM_ITERATIONS] = {0, 3, 6, 1, 4, 7, 2, 5};

constexpr uint8_t SENSOR_SELECTORS[NUM_ITERATIONS][cfg::NUM_SENSORS / 8] = {
    {1, 1, 1, 1, 1, 1},
    {2, 2, 2, 2, 2, 2},
    {4, 4, 4, 4, 4, 4},
    {8, 8, 8, 8, 8, 8},
    {16, 16, 16, 16, 16, 16},
    {32, 32, 32, 32, 32, 32},
    {64, 64, 64, 64, 64, 64},
    {128, 128, 128, 128, 128, 128}
};

#elif NUM_PARALLEL_SENSORS == 3

// Optimized for highest minimum distance between two measurements: 5
constexpr uint8_t SENSOR_GROUPS[NUM_ITERATIONS] = {0, 5, 10, 15, 4, 9, 14, 3, 8, 13, 2, 7, 12, 1, 6, 11};

constexpr uint8_t SENSOR_SELECTORS[NUM_ITERATIONS][cfg::NUM_SENSORS / 8] = {
    {0, 1, 0, 1, 0, 1},    {0, 2, 0, 2, 0, 2},    {0, 4, 0, 4, 0, 4},    {0, 8, 0, 8, 0, 8},
    {0, 16, 0, 16, 0, 16}, {0, 32, 0, 32, 0, 32}, {0, 64, 0, 64, 0, 64}, {0, 128, 0, 128, 0, 128},
    {1, 0, 1, 0, 1, 0},    {2, 0, 2, 0, 2, 0},    {4, 0, 4, 0, 4, 0},    {8, 0, 8, 0, 8, 0},
    {16, 0, 16, 0, 16, 0}, {32, 0, 32, 0, 32, 0}, {64, 0, 64, 0, 64, 0}, {128, 0, 128, 0, 128, 0}};


#endif // NUM_PARALLEL_SENSORS

} // namespace

SensorHandler::SensorHandler(const spi_t& spi,
                             const micro::vector<micro::gpio_t, cfg::NUM_SENSORS / 8>& adcEnPins,
                             const micro::gpio_t& LE_opto, const micro::gpio_t& OE_opto,
                             const micro::gpio_t& LE_ind, const micro::gpio_t& OE_ind)
    : spi_(spi), adcEnPins_(adcEnPins), LE_opto_(LE_opto), OE_opto_(OE_opto), LE_ind_(LE_ind),
      OE_ind_(OE_ind) {
}

void SensorHandler::initialize() {
    gpio_write(this->LE_opto_, gpioPinState_t::RESET);
    gpio_write(this->OE_opto_, gpioPinState_t::SET);

    gpio_write(this->LE_ind_, gpioPinState_t::RESET);
    gpio_write(this->OE_ind_, gpioPinState_t::SET);

    for (const gpio_t& adcEnPin : this->adcEnPins_) {
        gpio_write(adcEnPin, gpioPinState_t::SET);
    }
}

void SensorHandler::readSensors(Measurements& OUT measurements,
                                const std::pair<uint8_t, uint8_t>& scanRange) {
    for (uint8_t i = 0; i < NUM_ITERATIONS; ++i) {
        const uint8_t groupIdx = SENSOR_GROUPS[i];
        this->exchangeData(SENSOR_SELECTORS[groupIdx], nullptr, cfg::NUM_SENSORS / 8);

        gpio_write(this->LE_opto_, gpioPinState_t::SET);
        gpio_write(this->LE_opto_, gpioPinState_t::RESET);
        gpio_write(this->OE_opto_, gpioPinState_t::RESET);

        for (volatile uint32_t t = 0; t < 800; ++t) {
        } // waits between the LED light-up and the ADC read

        for (uint8_t adcIdx = groupIdx / 8; adcIdx < cfg::NUM_SENSORS / 8; adcIdx += NUM_ITERATIONS / 8) {
            const uint8_t absPos = (adcIdx * 8) + (groupIdx % 8);

            if (micro::isBtw(absPos, scanRange.first, scanRange.second)) {
                const gpio_t& adcEnPin = this->adcEnPins_[adcIdx];

                gpio_write(adcEnPin, gpioPinState_t::RESET);
                measurements[absPos] = this->readAdc(groupIdx);
                gpio_write(adcEnPin, gpioPinState_t::SET);
            }
        }

        gpio_write(this->OE_opto_, gpioPinState_t::SET);
    }
}

void SensorHandler::writeLeds(const Leds& leds) {
    uint8_t outBuffer[cfg::NUM_SENSORS / 8] = {0, 0, 0, 0, 0, 0};

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
    uint8_t adcBuffer[3] = {0, 0, 0};

    // Control byte: | START | SEL2 | SEL1 | SEL0 | UNI/BIP | SGL/DIF | PD1 | PD0 |
    // Select bits (according to the datasheet):
    //      SEL2    -   channel's 1st bit (LSB)
    //      SEL1    -   channel's 3rd bit
    //      SEL0    -   channel's 2nd bit
    //
    // @see MAX1110CAP+ datasheet for details
    adcBuffer[0] = 0b10001111 | ((channel & 0b00000001) << 6) | ((channel & 0b00000010) << 3) |
                   ((channel & 0b00000100) << 3);
    this->exchangeData(adcBuffer, adcBuffer, ARRAY_SIZE(adcBuffer));
    return (adcBuffer[1] << 2) |
           (adcBuffer[2] >> 6); // ADC value format: 00000000 00XXXXXX XX000000
}

void SensorHandler::exchangeData(const uint8_t* txBuf, uint8_t* rxBuf, const uint32_t size) {
    micro::spi_exchange(this->spi_, txBuf, rxBuf, size);
    this->semaphore_.take(millisecond_t(2));
}
