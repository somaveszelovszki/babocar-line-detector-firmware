#include <micro/utils/algorithm.hpp>
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

void fillTxData(LineDetectOutPanelLinkData& txData, const trackedLines_t& trackedLines) {

    uint32_t i = 0;

    for (; i < trackedLines.size(); ++i) {
        txData.lines[i].pos_mm_per16 = static_cast<int16_t>(trackedLines[i].pos.get() * 16);
        txData.lines[i].idx = trackedLines[i].id;
    }

    for (; i < ARRAY_SIZE(txData.lines); ++i) {
        txData.lines[i].pos_mm_per16 = 0;
        txData.lines[i].idx = 0;
    }
}

} // namespace

extern "C" void runLineCalcTask(void) {

    measurementsQueue = xQueueCreateStatic(MEASUREMENTS_QUEUE_LENGTH, sizeof(measurements_t), measurementsQueueStorageBuffer, &measurementsQueueBuffer);

    measurements_t measurements;

    LineDetectInPanelLinkData rxData;
    LineDetectOutPanelLinkData txData;

    while (true) {
        panelLink.update();
        globals::isConnected = panelLink.isConnected();

        if (panelLink.readAvailable(rxData)) {
            parseRxData(rxData);
        }

        if (xQueueReceive(measurementsQueue, &measurements, 0)) {
            xSemaphoreGive(lineCalcSemaphore);

            const linePositions_t linePositions = linePosCalc.calculate(measurements);
            const trackedLines_t trackedLines = lineFilter.update(linePositions);

            if (trackedLines.size()) {
                const millimeter_t avgLinePos = micro::accumulate(trackedLines.begin(), trackedLines.end(), millimeter_t(0),
                    [] (const millimeter_t& sum, const trackedLine_t& line) { return sum + line.pos; });

                globals::scanRangeCenter = round(LinePosCalculator::linePosToOptoPos(avgLinePos));
            }

            if (panelLink.shouldSend()) {
                fillTxData(txData, trackedLines);
                panelLink.send(txData);
            }
        }
    }

    vTaskDelete(nullptr);
}

void uart_Command_RxCpltCallback(void) {
    panelLink.onNewRxData();
}
