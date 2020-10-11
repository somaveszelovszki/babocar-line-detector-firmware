#pragma once

#include <micro/port/can.hpp>
#include <micro/port/gpio.hpp>
#include <micro/port/spi.hpp>
#include <micro/port/timer.hpp>
#include <micro/port/uart.hpp>

extern CAN_HandleTypeDef  hcan1;
extern SPI_HandleTypeDef  hspi1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart2;

#define can_Vehicle             micro::can_t{ &hcan1 }

#define gpio_SS_ADC0            micro::gpio_t{ GPIOC, GPIO_PIN_6 }
#define gpio_SS_ADC1            micro::gpio_t{ GPIOB, GPIO_PIN_3 }
#define gpio_SS_ADC2            micro::gpio_t{ GPIOA, GPIO_PIN_15 }
#define gpio_SS_ADC3            micro::gpio_t{ GPIOC, GPIO_PIN_15 }
#define gpio_SS_ADC4            micro::gpio_t{ GPIOC, GPIO_PIN_14 }
#define gpio_SS_ADC5            micro::gpio_t{ GPIOC, GPIO_PIN_13 }

#define gpio_OE_OPTO            micro::gpio_t{ GPIOC, GPIO_PIN_2 }
#define gpio_LE_OPTO            micro::gpio_t{ GPIOC, GPIO_PIN_3 }
#define gpio_OE_IND             micro::gpio_t{ GPIOC, GPIO_PIN_0 }
#define gpio_LE_IND             micro::gpio_t{ GPIOC, GPIO_PIN_1 }

#define spi_Sensor              micro::spi_t{ &hspi1 }

#define tim_System              micro::timer_t{ &htim2 }

#define uart_Debug              micro::uart_t{ &huart2 }

#define PANEL_VERSION_FRONT     0x01
#define PANEL_VERSION_REAR      0x00

#define QUARTZ_FREQ             megahertz_t(20)
