#include <cfg_board.hpp>
#include <micro/panel/panelVersion.hpp>
#include <micro/utils/timer.hpp>

#include <system_init.h>

using namespace micro;

extern "C" void Error_Handler(void);

extern "C" void system_init(void) {
    if (PANEL_VERSION_FRONT != getPanelVersion() && PANEL_VERSION_REAR != getPanelVersion()) {
        Error_Handler();
    }

    time_init({ tim_System });
}
