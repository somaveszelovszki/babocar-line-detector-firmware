#include <cfg_board.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/panel/panelVersion.hpp>
#include <micro/utils/algorithm.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/timer.hpp>

#include <LineFilter.hpp>
#include <LinePatternCalculator.hpp>
#include <LinePosCalculator.hpp>
#include <SensorData.hpp>

#include <numeric>

using namespace micro;

extern queue_t<Measurements, 1> measurementsQueue;

CanManager vehicleCanManager(can_Vehicle, millisecond_t(50));
queue_t<SensorControlData, 1> sensorControlDataQueue;

namespace {

LinePosCalculator linePosCalc;
LineFilter lineFilter;
LinePatternCalculator linePatternCalc;

linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
m_per_sec_t speed;
meter_t distance;
bool indicatorLedsEnabled = false;

Measurements measurements;
SensorControlData sensorControl;

canFrame_t rxCanFrame;
CanFrameHandler vehicleCanFrameHandler;

const Leds& updateFailureLeds() {

    static Leds leds;
    static Timer animationTimer(millisecond_t(25));
    static uint8_t pos = 0;
    static Sign dir = Sign::POSITIVE;

    if (animationTimer.checkTimeout()) {
        leds.reset();

        pos += dir * 1;
        if (0 == pos || cfg::NUM_SENSORS - 1 == pos) {
            dir = -dir;
        }

        leds.set(pos, true);
    }

    return leds;
}

void updateSensorControl(const Lines& lines) {
    static constexpr uint8_t LED_RADIUS = 1;

    if (vehicleCanManager.hasRxTimedOut()) {
        sensorControl.leds = updateFailureLeds();
    } else {
        sensorControl.leds.reset();

        if (indicatorLedsEnabled) {
            for (const Line& l : lines) {
                const uint8_t centerIdx = static_cast<uint8_t>(round(LinePosCalculator::linePosToOptoPos(l.pos)));

                const uint8_t startIdx = max<uint8_t>(centerIdx, LED_RADIUS) - LED_RADIUS;
                const uint8_t endIdx = min<uint8_t>(centerIdx + LED_RADIUS + 1, cfg::NUM_SENSORS);

                for (uint8_t i = startIdx; i < endIdx; ++i) {
                    sensorControl.leds.set(i, true);
                }
            }
        }
    }

    if (lines.size()) {
        const millimeter_t avgLinePos = std::accumulate(lines.begin(), lines.end(), millimeter_t(0),
            [] (const millimeter_t& sum, const Line& line) { return sum + line.pos; }) / lines.size();

        sensorControl.scanRangeCenter = round(LinePosCalculator::linePosToOptoPos(avgLinePos));
    }
}

} // namespace

extern "C" void runLineCalcTask(void) {

    vehicleCanFrameHandler.registerHandler(can::LongitudinalState::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(speed, distance);
    });

    vehicleCanFrameHandler.registerHandler(can::LineDetectControl::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LineDetectControl*>(data)->acquire(indicatorLedsEnabled, sensorControl.scanRangeRadius, domain);
    });

    const CanManager::subscriberId_t vehicleCanSubsciberId = vehicleCanManager.registerSubscriber(vehicleCanFrameHandler.identifiers());

    while (true) {
        measurementsQueue.receive(measurements);

        const LinePositions linePositions = linePosCalc.calculate(measurements);
        const Lines lines = lineFilter.update(linePositions);
        linePatternCalc.update(domain, lines, distance);

        if (PANEL_VERSION_FRONT == getPanelVersion()) {
            vehicleCanManager.send(can::FrontLines(lines));
        } else if (PANEL_VERSION_REAR == getPanelVersion()) {
            vehicleCanManager.send(can::RearLines(lines));
        }

        while (vehicleCanManager.read(vehicleCanSubsciberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        updateSensorControl(lines);
        sensorControlDataQueue.send(sensorControl);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
