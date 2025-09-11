#include <cfg_board.hpp>

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void spi_SensorTxCpltCallback();
extern void spi_SensorTxRxCpltCallback();
extern void micro_Vehicle_Can_RxFifoMsgPendingCallback();

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == spi_Sensor.handle) {
        spi_SensorTxCpltCallback();
    }
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == spi_Sensor.handle) {
        spi_SensorTxRxCpltCallback();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == can_Vehicle.handle) {
        micro_Vehicle_Can_RxFifoMsgPendingCallback();
    }
}
