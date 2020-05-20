#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/panel/panelVersion.h>
#include <micro/utils/algorithm.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.h>
#include <LineFilter.hpp>
#include <LinePatternCalculator.hpp>
#include <LinePosCalculator.hpp>
#include <SensorHandlerData.hpp>

#include <numeric>

using namespace micro;

extern queue_t<measurements_t, 1> measurementsQueue;

CanManager vehicleCanManager(can_Vehicle, canRxFifo_Vehicle, millisecond_t(50));
queue_t<SensorHandlerData, 1> sensorHandlerDataQueue;

namespace {

bool indicatorLedsEnabled = false;
uint8_t scanRangeCenter = 0;

void sendSensorHandlerData(const Lines& lines) {

    static constexpr uint8_t LED_RADIUS = 1;
    leds_t leds;

    if (indicatorLedsEnabled) {
        for (const Line& l : lines) {
            const uint8_t centerIdx = static_cast<uint8_t>(round(LinePosCalculator::linePosToOptoPos(l.pos)));

            const uint8_t startIdx = max<uint8_t>(centerIdx, LED_RADIUS) - LED_RADIUS;
            const uint8_t endIdx = min<uint8_t>(centerIdx + LED_RADIUS + 1, cfg::NUM_SENSORS);

            for (uint8_t i = startIdx; i < endIdx; ++i) {
                leds.set(i, true);
            }
        }
    }

    sensorHandlerDataQueue.overwrite({ leds, scanRangeCenter });
}

} // namespace

extern "C" void runLineCalcTask(void) {

    SystemManager::instance().registerTask();

    measurements_t measurements;
    LinePosCalculator linePosCalc;
    LineFilter lineFilter;
    LinePatternCalculator linePatternCalc;

    linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
    m_per_sec_t speed;
    meter_t distance;

    canFrame_t rxCanFrame;
    CanFrameHandler vehicleCanFrameHandler;

    vehicleCanFrameHandler.registerHandler(can::LongitudinalState::id(), [&speed, &distance] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(speed, distance);
    });

    vehicleCanFrameHandler.registerHandler(can::LineDetectControl::id(), [&domain] (const uint8_t * const data) {
        uint8_t scanRangeRadius;
        reinterpret_cast<const can::LineDetectControl*>(data)->acquire(indicatorLedsEnabled, scanRangeRadius, domain);
    });

    const CanManager::subscriberId_t vehicleCanSubsciberId = vehicleCanManager.registerSubscriber(vehicleCanFrameHandler.identifiers());

    while (true) {
        while (vehicleCanManager.read(vehicleCanSubsciberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        if (measurementsQueue.receive(measurements, millisecond_t(5))) {
            const linePositions_t linePositions = linePosCalc.calculate(measurements);
            const Lines lines = lineFilter.update(linePositions);
            linePatternCalc.update(domain, lines, distance);

            if (lines.size()) {
                const millimeter_t avgLinePos = std::accumulate(lines.begin(), lines.end(), millimeter_t(0),
                    [] (const millimeter_t& sum, const Line& line) { return sum + line.pos; });

                scanRangeCenter = round(LinePosCalculator::linePosToOptoPos(avgLinePos));
            }

            if (PANEL_VERSION_FRONT == panelVersion_get()) {
                vehicleCanManager.send(can::FrontLines(lines));
            } else if (PANEL_VERSION_REAR == panelVersion_get()) {
                vehicleCanManager.send(can::RearLines(lines));
            }

            sendSensorHandlerData(lines);

            SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut());
        }
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
