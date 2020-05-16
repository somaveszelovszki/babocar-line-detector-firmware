#ifndef CFG_BOARD_H
#define CFG_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

extern CAN_HandleTypeDef  hcan1;
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart2;

#define can_Vehicle             (&hcan1)
#define canRxFifo_Vehicle       CAN_RX_FIFO0

#define gpio_SS_ADC0            GPIOC
#define gpioPin_SS_ADC0         GPIO_PIN_6

#define gpio_SS_ADC1            GPIOB
#define gpioPin_SS_ADC1         GPIO_PIN_3

#define gpio_SS_ADC2            GPIOA
#define gpioPin_SS_ADC2         GPIO_PIN_15

#define gpio_SS_ADC3            GPIOC
#define gpioPin_SS_ADC3         GPIO_PIN_13

#define gpio_SS_ADC4            GPIOC
#define gpioPin_SS_ADC4         GPIO_PIN_14

#define gpio_SS_ADC5            GPIOC
#define gpioPin_SS_ADC5         GPIO_PIN_15

#define gpio_LedDrivers         GPIOC
#define gpioPin_OE_OPTO         GPIO_PIN_2
#define gpioPin_LE_OPTO         GPIO_PIN_3
#define gpioPin_OE_IND          GPIO_PIN_0
#define gpioPin_LE_IND          GPIO_PIN_1

#define spi_Sensor              (&hspi1)

#define tim_System              (&htim2)

#define uart_Debug              (&huart2)

#define PANEL_VERSION_FRONT     0x01
#define PANEL_VERSION_REAR      0x00

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CFG_BOARD_H
