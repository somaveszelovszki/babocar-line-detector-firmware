#include "linepanel.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_spi.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_tim.h"

#include <string.h>

extern SPI_HandleTypeDef hspi1;
static SPI_HandleTypeDef * const SPI_INSTANCE = &hspi1;

#define GPIO_SS_ADC         GPIOA
#define GPIO_PIN_ADC0_7     GPIO_PIN_8
#define GPIO_PIN_ADC8_15    GPIO_PIN_9
#define GPIO_PIN_ADC16_23   GPIO_PIN_10
#define GPIO_PIN_ADC24_31   GPIO_PIN_11

#define GPIO_LED_DRIVERS    GPIOB
#define GPIO_PIN_LE_OPTO    GPIO_PIN_1
#define GPIO_PIN_OE_OPTO    GPIO_PIN_3
#define GPIO_PIN_LE_IND     GPIO_PIN_4
#define GPIO_PIN_OE_IND     GPIO_PIN_5

static const uint8_t OPTO_BUFFERS[8][NUM_OPTOS / 8] = { // selects every 8th optical sensor
    { 1, 1, 1, 1 },
    { 2, 2, 2, 2 },
    { 4, 4, 4, 4 },
    { 8, 8, 8, 8 },
    { 16, 16, 16, 16 },
    { 32, 32, 32, 32 },
    { 64, 64, 64, 64 },
    { 128, 128, 128, 128 }
};

static inline void select_ADC(uint8_t idx) {
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC0_7,   idx == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC8_15,  idx == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC16_23, idx == 2 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC24_31, idx == 3 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline HAL_StatusTypeDef write_OPTO(uint8_t optoIdx) {
    return HAL_SPI_Transmit(SPI_INSTANCE, (uint8_t*)OPTO_BUFFERS[optoIdx], NUM_OPTOS / 8, 2);  // selects every 8th optical sensor
}

static inline HAL_StatusTypeDef write_IND(const uint8_t *leds) {
    uint8_t ind_buffer[NUM_OPTOS / 8 * 2] = { 0, 0, 0, 0, 0, 0, 0, 0 };

    // appends led bits after the first 4 bytes of the indicator buffer
    // (the LED drivers are connected in a series, values have to be shifted through the optical sensor LED drivers)
    memcpy(ind_buffer, leds, NUM_OPTOS / 8);

    //return HAL_SPI_Transmit(SPI_INSTANCE, ind_buffer, NUM_OPTOS / 8 * 2, 2);
    return HAL_SPI_Transmit(SPI_INSTANCE, ind_buffer, 6, 2);    // TODO why?
}

static inline uint8_t read_ADC(uint8_t channel) {
    uint8_t adc_buffer[3];

    // Control byte: | START | SEL2 | SEL1 | SEL0 | UNI/BIP | SGL/DIF | PD1 | PD0 |
    // Select bits (according to the datasheet):
    //      SEL2    -   channel's 1st bit (LSB)
    //      SEL1    -   channel's 3rd bit
    //      SEL0    -   channel's 2nd bit
    //
    // @see MAX1110CAP+ datasheet for details
    adc_buffer[0] =  0b10001111 | ((channel & 0b00000001) << 6) | ((channel & 0b00000010) << 3) | ((channel & 0b00000100) << 3);
    adc_buffer[1] = adc_buffer[2] = 0x00;
    HAL_SPI_TransmitReceive(SPI_INSTANCE, adc_buffer, adc_buffer, 3, 2);

    return (adc_buffer[1] << 2) | (adc_buffer[2] >> 6); // ADC value format: 00000000 00XXXXXX XX000000
}

static inline void end_comm_OPTO() {
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_OPTO, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_OPTO, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_OPTO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_OPTO, GPIO_PIN_RESET);
}

static inline void end_comm_IND() {
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_IND, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_IND, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_IND, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_IND, GPIO_PIN_RESET);
}

static inline void end_comm_ADC() {
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC0_7,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC8_15,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC16_23, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC24_31, GPIO_PIN_SET);
}

HAL_StatusTypeDef linepanel_initialize() {

    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_OPTO, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_OPTO, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_LE_IND, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_LED_DRIVERS, GPIO_PIN_OE_IND, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC0_7,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC8_15,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC16_23, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_SS_ADC, GPIO_PIN_ADC24_31, GPIO_PIN_SET);

    return HAL_OK;
}

HAL_StatusTypeDef linepanel_read_optos(uint8_t *result) {
    HAL_StatusTypeDef status = HAL_OK;
    for (uint8_t optoIdx = 0; optoIdx < 8; ++optoIdx) {
        start_comm_OPTO();
        status = write_OPTO(optoIdx);
        end_comm_OPTO();

        if (status == HAL_OK) {
            // TODO delay if needed

            for (uint8_t adcIdx = 0; adcIdx < NUM_OPTOS / 8; ++adcIdx) {
                select_ADC(adcIdx);

                const uint8_t ADC_value = read_ADC(optoIdx);
                const uint8_t idx = adcIdx * 8 + optoIdx;
                result[idx] = ADC_value;

                end_comm_ADC(adcIdx);
            }
        } else {
            break;
        }
    }

    return status;
}

HAL_StatusTypeDef linepanel_write_leds(const uint8_t *leds) {
    HAL_StatusTypeDef status = write_IND(leds);
    end_comm_IND();
    return status;
}
