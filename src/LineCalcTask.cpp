#include <micro/panel/CanManager.hpp>
#include <micro/panel/panelVersion.h>
#include <micro/utils/algorithm.hpp>
#include <micro/utils/task.hpp>
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
QueueHandle_t measurementsQueue = nullptr;
static uint8_t measurementsQueueStorageBuffer[MEASUREMENTS_QUEUE_LENGTH * sizeof(measurements_t)];
static StaticQueue_t measurementsQueueBuffer;

namespace {

void sendLedStates(const Lines& lines) {

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

    micro::waitReady(ledsQueue);
    micro::waitReady(lineCalcSemaphore);

    measurements_t measurements;
    LinePosCalculator linePosCalc;
    LineFilter lineFilter;
    LinePatternCalculator linePatternCalc;

    linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
    m_per_sec_t speed;
    meter_t distance;

    CanManager canManager(can_Vehicle, canRxFifo_Vehicle, millisecond_t(15));

    canManager.registerHandler(can::LongitudinalState::id(), [&speed, &distance] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(speed, distance);
    });

    canManager.registerHandler(can::LineDetectControl::id(), [&domain] (const uint8_t * const data) {
        reinterpret_cast<const can::LineDetectControl*>(data)->acquire(globals::indicatorLedsEnabled, globals::scanRangeRadius, domain);
    });

    while (true) {

        globals::isConnected = !canManager.hasRxTimedOut();

        canManager.handleIncomingFrames();

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

            if (PANEL_VERSION_FRONT == panelVersion_get()) {
                canManager.send(can::FrontLines(lines));
            } else if (PANEL_VERSION_REAR == panelVersion_get()) {
                canManager.send(can::RearLines(lines));
            }

            sendLedStates(lines);
        }
    }

    vTaskDelete(nullptr);
}
