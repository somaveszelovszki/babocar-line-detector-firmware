#include <cfg_board.hpp>
#include <micro/container/ring_buffer.hpp>
#include <micro/debug/params.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/port/semaphore.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/str_utils.hpp>
#include <micro/utils/timer.hpp>

using namespace micro;

queue_t<uint8_t, 1> numFailingTasksQueue;

namespace {

void monitorTasks() {
    static Timer failureCheckTimer(millisecond_t(200));

    if (failureCheckTimer.checkTimeout()) {
        numFailingTasksQueue.overwrite(static_cast<uint8_t>(SystemManager::instance().failingTasks().size()));
    }
}

} // namespace

extern "C" void runDebugTask(void) {
    SystemManager::instance().registerTask();

    while (true) {
        monitorTasks();
        SystemManager::instance().notify(true);
    }
}
