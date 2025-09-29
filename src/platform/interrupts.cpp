#include <cfg_board.hpp>

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void micro_Vehicle_Can_RxFifoMsgPendingCallback();

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == can_Vehicle.handle) {
        micro_Vehicle_Can_RxFifoMsgPendingCallback();
    }
}
