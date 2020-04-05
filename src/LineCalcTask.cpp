#include <micro/panel/vehicleCanTypes.hpp>
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

LinePosCalculator linePosCalc;
LineFilter lineFilter;
LinePatternCalculator linePatternCalc;

linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
m_per_sec_t speed;
meter_t distance;

void parseVehicleCanData(const uint32_t id, const uint8_t * const data) {

    switch (id) {
    case can::LongitudinalState::id():
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(speed, distance);
        break;

    case can::LineDetectControl::id():
        reinterpret_cast<const can::LineDetectControl*>(data)->acquire(globals::indicatorLedsEnabled, globals::scanRangeRadius, domain);
        break;
    }
}

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

    measurements_t measurements;

    WatchdogTimer vehicleCanWatchdog;
    vehicleCanWatchdog.start(millisecond_t(15));

    CAN_RxHeaderTypeDef rxHeader;
    alignas(8) uint8_t rxData[8];
    uint32_t txMailbox = 0;

    while (true) {

        globals::isConnected = !vehicleCanWatchdog.hasTimedOut();

        if (HAL_CAN_GetRxFifoFillLevel(can_Vehicle, canRxFifo_Vehicle)) {
            if (HAL_OK == HAL_CAN_GetRxMessage(can_Vehicle, canRxFifo_Vehicle, &rxHeader, rxData)) {
                parseVehicleCanData(rxHeader.StdId, rxData);
                vehicleCanWatchdog.reset();
            }
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

            if (PANEL_ID_FRONT_LINE_DETECT == globals::panelId) {
                CAN_TxHeaderTypeDef txHeader;
                txHeader.StdId = can::FrontLines::id();
                txHeader.ExtId = 0;
                txHeader.IDE   = CAN_ID_STD;
                txHeader.RTR   = CAN_RTR_DATA;
                txHeader.DLC   = sizeof(can::FrontLines);
                txHeader.TransmitGlobalTime = DISABLE;

                can::FrontLines frontLines(lines);
                HAL_CAN_AddTxMessage(can_Vehicle, &txHeader, reinterpret_cast<uint8_t*>(&frontLines), &txMailbox);
            }

            sendLedStates(lines);
        }
    }

    vTaskDelete(nullptr);
}
