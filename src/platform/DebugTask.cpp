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
    static Timer failureLogTimer(millisecond_t(1000));

    if (failureCheckTimer.checkTimeout()) {
        const SystemManager::TaskStates failingTasks = SystemManager::instance().failingTasks();

        if (failingTasks.size() && failureLogTimer.checkTimeout()) {
            char msg[LOG_MSG_MAX_SIZE];
            uint32_t idx = 0;
            for (SystemManager::TaskStates::const_iterator it = failingTasks.begin(); it != failingTasks.end(); ++it) {
                idx += strncpy_until(&msg[idx], it->details.pcTaskName, min(static_cast<uint32_t>(configMAX_TASK_NAME_LEN), LOG_MSG_MAX_SIZE - idx));
                if (it != failingTasks.back()) {
                    idx += strncpy_until(&msg[idx], ", ", sizeof(", "), LOG_MSG_MAX_SIZE - idx);
                }
            }
            msg[idx] = '\0';
            LOG_ERROR("Failing tasks: %s", msg);
        }

        numFailingTasksQueue.overwrite(static_cast<uint8_t>(failingTasks.size()));
    }
}

} // namespace

extern "C" void runDebugTask(void) {
    SystemManager::instance().registerTask();

    while (true) {
        monitorTasks();
        SystemManager::instance().notify(false);
    }
}
