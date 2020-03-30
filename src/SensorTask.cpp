#include <micro/utils/log.hpp>
#include <micro/task/common.hpp>

#include <cfg_board.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern QueueHandle_t measurementsQueue;

extern "C" void runSensorTask(void) {

    while (true) {

        vTaskDelay(1);
    }

    vTaskDelete(nullptr);
}
