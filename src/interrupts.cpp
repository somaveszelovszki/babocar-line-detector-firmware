#include <cfg_board.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void uart_Command_RxCpltCallback();
extern void spi_SensorTxCpltCallback();
extern void spi_SensorTxRxCpltCallback();

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_Command) {
        uart_Command_RxCpltCallback();
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == spi_Sensor) {
        spi_SensorTxCpltCallback();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == spi_Sensor) {
        spi_SensorTxRxCpltCallback();
    }
}

extern "C" void onHardFault() {
    while(1) {}
}
