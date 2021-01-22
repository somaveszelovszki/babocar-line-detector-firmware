#include <micro/panel/CanManager.hpp>
#include <micro/panel/panelVersion.hpp>
#include <micro/utils/algorithm.hpp>
#include <micro/port/queue.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>
#include <LineFilter.hpp>
#include <LinePatternCalculator.hpp>
#include <LinePosCalculator.hpp>
#include <SensorData.hpp>

#include <numeric>

using namespace micro;

extern queue_t<Measurements, 1> measurementsQueue;

CanManager vehicleCanManager(can_Vehicle);
queue_t<SensorControlData, 1> sensorControlDataQueue;

namespace {

LinePosCalculator linePosCalc;
LineFilter lineFilter;
LinePatternCalculator linePatternCalc;

linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
m_per_sec_t speed;
meter_t distance;
bool indicatorLedsEnabled = true;

Measurements measurements;
SensorControlData sensorControl;

canFrame_t rxCanFrame;
CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::id_t vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

const Leds& updateFailureLeds() {

    static constexpr float SENSOR_OFFSET = cfg::NUM_SENSORS / 2.0f - 0.5f;

    static Leds leds;
    static Timer animationTimer(millisecond_t(1200));
    static radian_t angle = radian_t(0);

    animationTimer.checkTimeout();
    angle = map(getTime(), animationTimer.startTime(), animationTimer.startTime() + animationTimer.period(), radian_t(0), 2 * PI);

    leds.fill(false);
    leds[static_cast<uint32_t>(micro::round(SENSOR_OFFSET + micro::cos(angle) * SENSOR_OFFSET))] = true;

    return leds;
}

void updateSensorControl(const Lines& lines, const bool isOk) {
    static constexpr uint8_t LED_RADIUS = 1;

    if (isOk) {
        sensorControl.leds.fill(false);

        if (indicatorLedsEnabled) {
            for (const Line& l : lines) {
                const uint8_t centerIdx = static_cast<uint8_t>(round(LinePosCalculator::linePosToOptoPos(l.pos)));

                const uint8_t startIdx = max<uint8_t>(centerIdx, LED_RADIUS) - LED_RADIUS;
                const uint8_t endIdx = min<uint8_t>(centerIdx + LED_RADIUS + 1, cfg::NUM_SENSORS);

                for (uint8_t i = startIdx; i < endIdx; ++i) {
                    sensorControl.leds[i] = true;
                }
            }
        }
    } else {
        sensorControl.leds = updateFailureLeds();
    }

    sensorControl.scanEnabled = true;

    if (lines.size()) {
        const millimeter_t avgLinePos = std::accumulate(lines.begin(), lines.end(), millimeter_t(0),
            [] (const millimeter_t& sum, const Line& line) { return sum + line.pos; }) / lines.size();

        sensorControl.scanRangeCenter = round(LinePosCalculator::linePosToOptoPos(avgLinePos));
    }
}

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::LongitudinalState::id(), [] (const uint8_t * const data) {
        bool isRemoteControlled;
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(speed, isRemoteControlled, distance);
    });

    vehicleCanFrameHandler.registerHandler(can::LineDetectControl::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LineDetectControl*>(data)->acquire(indicatorLedsEnabled, sensorControl.scanRangeRadius, domain);
    });

    const CanFrameIds rxFilter = vehicleCanFrameHandler.identifiers();
    const CanFrameIds txFilter = {
        PANEL_VERSION_FRONT == getPanelVersion() ? can::FrontLines::id() : can::RearLines::id(),
        PANEL_VERSION_FRONT == getPanelVersion() ? can::FrontLinePattern::id() : can::RearLinePattern::id()
    };
    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runLineCalcTask(void) {

    initializeVehicleCan();

    for (uint8_t i = 0; i < cfg::NUM_SENSORS; ++i) {
        measurements[i] = 0;
    }

    while (true) {
        measurementsQueue.receive(measurements);

        const LinePositions linePositions = linePosCalc.calculate(measurements);
        const Lines lines = lineFilter.update(linePositions);
        linePatternCalc.update(domain, lines, distance, PANEL_VERSION_FRONT == getPanelVersion() ? sgn(speed) : -sgn(speed));

        if (PANEL_VERSION_FRONT == getPanelVersion()) {
            vehicleCanManager.periodicSend<can::FrontLines>(vehicleCanSubscriberId, lines);
            vehicleCanManager.periodicSend<can::FrontLinePattern>(vehicleCanSubscriberId, linePatternCalc.pattern());
        } else if (PANEL_VERSION_REAR == getPanelVersion()) {
            vehicleCanManager.periodicSend<can::RearLines>(vehicleCanSubscriberId, lines);
            vehicleCanManager.periodicSend<can::RearLinePattern>(vehicleCanSubscriberId, linePatternCalc.pattern());
        }

        while (vehicleCanManager.read(vehicleCanSubscriberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        const bool isOk = !vehicleCanManager.hasTimedOut(vehicleCanSubscriberId);
        updateSensorControl(lines, isOk);
        sensorControlDataQueue.send(sensorControl);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
