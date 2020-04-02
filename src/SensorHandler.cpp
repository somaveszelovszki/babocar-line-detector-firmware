#include <cfg_board.h>
#include <cfg_sensor.hpp>
#include <SensorHandler.hpp>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"

#include <utility>

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

const std::pair<GPIO_TypeDef*, uint16_t> ADC_ENABLE_PINS[cfg::NUM_SENSORS / 8] = {
    { GPIO_SS_ADC0, GPIO_PIN_SS_ADC0 },
    { GPIO_SS_ADC1, GPIO_PIN_SS_ADC1 },
    { GPIO_SS_ADC2, GPIO_PIN_SS_ADC2 },
    { GPIO_SS_ADC3, GPIO_PIN_SS_ADC3 },
    { GPIO_SS_ADC4, GPIO_PIN_SS_ADC4 },
    { GPIO_SS_ADC5, GPIO_PIN_SS_ADC5 }
};

} // namespace

void SensorHandler::initialize() {
    this->semaphore_ = xSemaphoreCreateBinaryStatic(&this->semaphoreBuffer_);
    xSemaphoreGive(this->semaphore_);

    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_OPTO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_OPTO, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_IND, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_IND, GPIO_PIN_SET);

    for (const std::pair<GPIO_TypeDef*, uint16_t>& adcEnPin : ADC_ENABLE_PINS) {
        HAL_GPIO_WritePin(adcEnPin.first, adcEnPin.second, GPIO_PIN_SET);
    }
}

void SensorHandler::readSensors(measurements_t& OUT measurements, const uint8_t first, const uint8_t last) {
    for (uint8_t optoIdx = 0; optoIdx < 8; ++optoIdx) {

        HAL_SPI_Transmit_DMA(spi_Sensor, (uint8_t*)OPTO_BUFFERS[optoIdx], cfg::NUM_SENSORS / 8);

        HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_OPTO, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_OPTO, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_OPTO, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_OPTO, GPIO_PIN_RESET);

        for (uint8_t adcIdx = 0; adcIdx < cfg::NUM_SENSORS / 8; ++adcIdx) {
            const std::pair<GPIO_TypeDef*, uint16_t>& adcEnPin = ADC_ENABLE_PINS[adcIdx];

            HAL_GPIO_WritePin(adcEnPin.first, adcEnPin.second, GPIO_PIN_RESET);
            measurements[adcIdx * 8 + optoIdx] = this->readAdc(optoIdx);
            HAL_GPIO_WritePin(adcEnPin.first, adcEnPin.second, GPIO_PIN_SET);
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

    HAL_SPI_Transmit_DMA(spi_Sensor, outBuffer, ARRAY_SIZE(outBuffer));

    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_IND, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_IND, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_IND, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_IND, GPIO_PIN_RESET);
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
    HAL_SPI_TransmitReceive_DMA(spi_Sensor, adcBuffer, adcBuffer, ARRAY_SIZE(adcBuffer));
    return (adcBuffer[1] << 2) | (adcBuffer[2] >> 6); // ADC value format: 00000000 00XXXXXX XX000000
}
