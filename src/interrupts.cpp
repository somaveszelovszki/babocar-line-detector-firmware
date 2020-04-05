#include <cfg_board.h>

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void spi_SensorTxCpltCallback();
extern void spi_SensorTxRxCpltCallback();

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
