#include <cfg_board.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void micro_Command_Uart_RxCpltCallback();

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_Command) {
        micro_Command_Uart_RxCpltCallback();
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {}

extern "C" void onHardFault() {
    while(1) {}
}
