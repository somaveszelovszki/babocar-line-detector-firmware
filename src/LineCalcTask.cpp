#include <micro/panel/PanelLink.hpp>
#include <micro/panel/LineDetectPanelLinkData.hpp>
#include <micro/utils/algorithm.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <globals.hpp>
#include <LineFilter.hpp>
#include <LinePatternCalculator.hpp>
#include <LinePosCalculator.hpp>
#include <SensorHandler.hpp>

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
LinePatternCalculator linePatternCalc;

linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
millimeter_t distance;

void parseRxData(const LineDetectInPanelLinkData& rxData) {
    globals::indicatorLedsEnabled = rxData.indicatorLedsEnabled;
    globals::scanRangeRadius      = rxData.scanRangeRadius;
    domain                        = static_cast<linePatternDomain_t>(rxData.domain);
    distance                      = millimeter_t(rxData.distance_mm);
}

void fillTxData(LineDetectOutPanelLinkData& txData, const Lines& lines, const LinePattern& pattern) {

    uint32_t i = 0;

    for (; i < lines.size(); ++i) {
        txData.lines[i].pos_mm_per16 = static_cast<int16_t>(lines[i].pos.get() * 16);
        txData.lines[i].id = lines[i].id;
    }

    for (; i < ARRAY_SIZE(txData.lines); ++i) {
        txData.lines[i].pos_mm_per16 = 0;
        txData.lines[i].id = 0;
    }

    txData.pattern.type         = static_cast<uint8_t>(pattern.type);
    txData.pattern.dir          = static_cast<int8_t>(pattern.dir);
    txData.pattern.side         = static_cast<int8_t>(pattern.side);
    txData.pattern.startDist_mm = static_cast<uint32_t>(pattern.startDist);
}

void sendLeds(const Lines& lines) {

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
            for (const Line& l : lines) {
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

    while (true) {
        panelLink.update();
        globals::isConnected = panelLink.isConnected();

        if (panelLink.readAvailable(rxData)) {
            parseRxData(rxData);
        }

        if (xQueueReceive(measurementsQueue, &measurements, 0)) {
            xSemaphoreGive(lineCalcSemaphore);

            const linePositions_t linePositions = linePosCalc.calculate(measurements);
            const Lines lines = lineFilter.update(linePositions);
            linePatternCalc.update(domain, lines, distance);

            if (lines.size()) {
                const millimeter_t avgLinePos = micro::accumulate(lines.begin(), lines.end(), millimeter_t(0),
                    [] (const millimeter_t& sum, const Line& line) { return sum + line.pos; });

                globals::scanRangeCenter = round(LinePosCalculator::linePosToOptoPos(avgLinePos));
            }

            if (panelLink.shouldSend()) {
                fillTxData(txData, lines, linePatternCalc.pattern());
                panelLink.send(txData);
            }

            sendLeds(lines);
        }
    }

    vTaskDelete(nullptr);
}

void uart_Command_RxCpltCallback(void) {
    panelLink.onNewRxData();
}
