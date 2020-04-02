#include <micro/utils/algorithm.hpp>
#include <micro/utils/timer.hpp>
#include <micro/task/common.hpp>
#include <micro/panel/PanelLink.hpp>
#include <micro/panel/LineDetectPanelLinkData.hpp>

#include <cfg_board.h>
#include <globals.hpp>
#include <SensorHandler.hpp>
#include <LinePosCalculator.hpp>
#include <LineFilter.hpp>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

using namespace micro;

extern QueueHandle_t ledsQueue;
extern SemaphoreHandle_t lineCalcSemaphore;

#define MEASUREMENTS_QUEUE_LENGTH 1
QueueHandle_t measurementsQueue;
static uint8_t measurementsQueueStorageBuffer[MEASUREMENTS_QUEUE_LENGTH * sizeof(measurements_t)];
static StaticQueue_t measurementsQueueBuffer;

namespace {

PanelLink<LineDetectInPanelLinkData, LineDetectOutPanelLinkData> panelLink(panelLinkRole_t::Slave, uart_Command);
LinePosCalculator linePosCalc;
LineFilter lineFilter;

void parseRxData(const LineDetectInPanelLinkData& rxData) {
    vTaskSuspendAll();
    globals::indicatorLedsEnabled = rxData.indicatorLedsEnabled;
    globals::scanRangeRadius = rxData.scanRangeRadius;
    xTaskResumeAll();
}

void fillTxData(LineDetectOutPanelLinkData& txData, const trackedLines_t& trackedLines, microsecond_t timeDiff) {

    static constexpr uint16_t MAX_D_TIME_US = std::numeric_limits<uint16_t>::max();

    uint32_t i = 0;

    for (; i < trackedLines.size(); ++i) {
        txData.lines[i].pos_mm_per16 = static_cast<int16_t>(trackedLines[i].pos.get() * 16);
        txData.lines[i].idx = trackedLines[i].id;
    }

    for (; i < ARRAY_SIZE(txData.lines); ++i) {
        txData.lines[i].pos_mm_per16 = 0;
        txData.lines[i].idx = 0;
    }

    txData.d_time_us = timeDiff < microsecond_t(MAX_D_TIME_US) ? static_cast<uint16_t>(timeDiff.get()) : MAX_D_TIME_US;
}

void sendLeds(const trackedLines_t& trackedLines) {

    static constexpr uint8_t LED_RADIUS = 1;

    static leds_t leds;
    static bool prevIsConnected = false;
    static Timer blinkTimer = [] () {
        Timer timer;
        timer.start(millisecond_t(250));
        return timer;
    }();

    bool send = true;

    if (globals::isConnected) {
        leds.reset();

        if (globals::indicatorLedsEnabled) {
            for (const trackedLine_t& l : trackedLines) {
                const uint8_t centerIdx = static_cast<uint8_t>(round(LinePosCalculator::linePosToOptoPos(l.pos)));

                const uint8_t startIdx = max<uint8_t>(centerIdx, LED_RADIUS) - LED_RADIUS;
                const uint8_t endIdx = min<uint8_t>(centerIdx + LED_RADIUS + 1, cfg::NUM_SENSORS);

                for (uint8_t i = startIdx; i < endIdx; ++i) {
                    leds.set(i, true);
                }
            }
        } else if (!prevIsConnected) {
            prevIsConnected = true;
        } else {
            send = false;
        }
    } else if (blinkTimer.checkTimeout()) {
        const bool blinkState = leds.get(0);
        leds.reset();
        leds.set(0, !blinkState);
    }

    if (send) {
        xQueueOverwrite(ledsQueue, &leds);
    }
}

} // namespace

extern "C" void runLineCalcTask(void) {

    measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_LENGTH, sizeof(measurements_t), measurementsQueueStorageBuffer, &measurementsQueueBuffer);

    measurements_t measurements;

    LineDetectInPanelLinkData rxData;
    LineDetectOutPanelLinkData txData;

    microsecond_t prevLineCalcTime = getExactTime();

    while (true) {
        panelLink.update();
        globals::isConnected = panelLink.isConnected();

        if (panelLink.readAvailable(rxData)) {
            parseRxData(rxData);
        }

        if (xQueueReceive(measurementsQueue, &measurements, 0)) {
            xSemaphoreGive(lineCalcSemaphore);

            const microsecond_t now = getExactTime();
            const microsecond_t timeDiff = now - prevLineCalcTime;
            prevLineCalcTime = now;

            const linePositions_t linePositions = linePosCalc.calculate(measurements);
            const trackedLines_t trackedLines = lineFilter.update(linePositions);

            if (trackedLines.size()) {
                const millimeter_t avgLinePos = micro::accumulate(trackedLines.begin(), trackedLines.end(), millimeter_t(0),
                    [] (const millimeter_t& sum, const trackedLine_t& line) { return sum + line.pos; });

                globals::scanRangeCenter = round(LinePosCalculator::linePosToOptoPos(avgLinePos));
            }

            if (panelLink.shouldSend()) {
                fillTxData(txData, trackedLines, timeDiff);
                panelLink.send(txData);
            }

            sendLeds(trackedLines);
        }
    }

    vTaskDelete(nullptr);
}

void uart_Command_RxCpltCallback(void) {
    panelLink.onNewRxData();
}
