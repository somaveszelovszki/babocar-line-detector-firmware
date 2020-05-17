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

SensorHandler::SensorHandler(SPI_HandleTypeDef *hspi,
    const micro::vec<micro::gpio_t, cfg::NUM_SENSORS / 8>& adcEnPins,
    const micro::gpio_t& LE_opto,
    const micro::gpio_t& OE_opto,
    const micro::gpio_t& LE_ind,
    const micro::gpio_t& OE_ind)
    : hspi_(hspi)
    , adcEnPins_(adcEnPins)
    , LE_opto_(LE_opto)
    , OE_opto_(OE_opto)
    , LE_ind_(LE_ind)
    , OE_ind_(OE_ind) {}

void SensorHandler::initialize() {
    HAL_GPIO_WritePin(this->LE_opto_.instance, this->LE_opto_.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(this->OE_opto_.instance, this->OE_opto_.pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(this->LE_ind_.instance, this->LE_ind_.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(this->OE_ind_.instance, this->OE_ind_.pin, GPIO_PIN_SET);

    for (const gpio_t& adcEnPin : this->adcEnPins_) {
        HAL_GPIO_WritePin(adcEnPin.instance, adcEnPin.pin, GPIO_PIN_SET);
    }
}

void SensorHandler::readSensors(measurements_t& OUT measurements, const uint8_t first, const uint8_t last) {
    for (uint8_t optoIdx = 0; optoIdx < 8; ++optoIdx) {

        HAL_SPI_Transmit_DMA(this->hspi_, (uint8_t*)OPTO_BUFFERS[optoIdx], cfg::NUM_SENSORS / 8);

        HAL_GPIO_WritePin(this->OE_opto_.instance, this->OE_opto_.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(this->LE_opto_.instance, this->LE_opto_.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(this->LE_opto_.instance, this->LE_opto_.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(this->OE_opto_.instance, this->OE_opto_.pin, GPIO_PIN_RESET);

        for (uint8_t adcIdx = 0; adcIdx < cfg::NUM_SENSORS / 8; ++adcIdx) {
            const uint8_t pos = adcIdx * 8 + optoIdx;

            if (micro::isBtw(pos, first, last)) {
                const gpio_t& adcEnPin = this->adcEnPins_[adcIdx];

                HAL_GPIO_WritePin(adcEnPin.instance, adcEnPin.pin, GPIO_PIN_RESET);
                measurements[pos] = this->readAdc(optoIdx);
                HAL_GPIO_WritePin(adcEnPin.instance, adcEnPin.pin, GPIO_PIN_SET);
            }
        }
    }
}

void SensorHandler::writeLeds(const leds_t& leds) {
    uint8_t outBuffer[2 * cfg::NUM_SENSORS / 8] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    for (uint32_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        if (leds.get(i)) {
            outBuffer[(cfg::NUM_SENSORS + i) / 8] |= (1 << (i % 8));
        }
    }

    HAL_SPI_Transmit_DMA(this->hspi_, outBuffer, ARRAY_SIZE(outBuffer));

    HAL_GPIO_WritePin(this->OE_ind_.instance, this->OE_ind_.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(this->LE_ind_.instance, this->LE_ind_.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(this->LE_ind_.instance, this->LE_ind_.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(this->OE_ind_.instance, this->OE_ind_.pin, GPIO_PIN_RESET);
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
    HAL_SPI_TransmitReceive_DMA(this->hspi_, adcBuffer, adcBuffer, ARRAY_SIZE(adcBuffer));
    return (adcBuffer[1] << 2) | (adcBuffer[2] >> 6); // ADC value format: 00000000 00XXXXXX XX000000
}
