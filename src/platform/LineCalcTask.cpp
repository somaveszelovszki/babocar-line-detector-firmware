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
extern queue_t<uint8_t, 1> numFailingTasksQueue;

CanManager vehicleCanManager(can_Vehicle, millisecond_t(50));
queue_t<SensorControlData, 1> sensorControlDataQueue;

namespace {

constexpr std::pair<uint8_t, uint8_t> FRONT_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 13,  255 }, // 0
    { 14,  217 }, // 1
    { 14,  212 }, // 2
    { 11,  222 }, // 3
    { 12,  215 }, // 4
    { 13,  226 }, // 5
    { 13,  209 }, // 6
    { 14,  229 }, // 7
    { 12,  244 }, // 8
    { 15,  236 }, // 9
    { 14,  179 }, // 10
    { 14,  189 }, // 11
    { 13,  202 }, // 12
    { 14,  212 }, // 13
    { 13,  187 }, // 14
    { 13,  200 }, // 15
    { 13,  232 }, // 16
    { 14,  198 }, // 17
    { 14,  209 }, // 18
    { 14,  224 }, // 19
    { 14,  227 }, // 20
    { 15,  233 }, // 21
    { 15,  238 }, // 22
    { 15,  242 }, // 23
    { 13,  247 }, // 24
    { 15,  232 }, // 25
    { 14,  210 }, // 26
    { 14,  184 }, // 27
    { 14,  191 }, // 28
    { 15,  211 }, // 29
    { 14,  211 }, // 30
    { 15,  218 }, // 31
    { 8,   219 }, // 32
    { 14,  198 }, // 33
    { 14,  180 }, // 34
    { 14,  201 }, // 35
    { 14,  179 }, // 36
    { 15,  203 }, // 37
    { 21,  240 }, // 38
    { 50,  240 }, // 39
    { 42,  232 }, // 40
    { 19,  230 }, // 41
    { 23,  233 }, // 42
    { 20,  226 }, // 43
    { 21,  232 }, // 44
    { 20,  227 }, // 45
    { 23,  227 }, // 46
    { 21,  218 }  // 47
};

constexpr std::pair<uint8_t, uint8_t> REAR_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 13,  255 }, // 0
    { 14,  217 }, // 1
    { 14,  212 }, // 2
    { 11,  222 }, // 3
    { 12,  215 }, // 4
    { 13,  226 }, // 5
    { 13,  209 }, // 6
    { 14,  229 }, // 7
    { 12,  244 }, // 8
    { 15,  236 }, // 9
    { 14,  179 }, // 10
    { 14,  189 }, // 11
    { 13,  202 }, // 12
    { 14,  212 }, // 13
    { 13,  187 }, // 14
    { 13,  200 }, // 15
    { 13,  232 }, // 16
    { 14,  198 }, // 17
    { 14,  209 }, // 18
    { 14,  224 }, // 19
    { 14,  227 }, // 20
    { 15,  233 }, // 21
    { 15,  238 }, // 22
    { 15,  242 }, // 23
    { 13,  247 }, // 24
    { 15,  232 }, // 25
    { 14,  210 }, // 26
    { 14,  184 }, // 27
    { 14,  191 }, // 28
    { 15,  211 }, // 29
    { 14,  211 }, // 30
    { 15,  218 }, // 31
    { 8,   219 }, // 32
    { 14,  198 }, // 33
    { 14,  180 }, // 34
    { 14,  201 }, // 35
    { 14,  179 }, // 36
    { 15,  203 }, // 37
    { 21,  240 }, // 38
    { 50,  240 }, // 39
    { 42,  232 }, // 40
    { 19,  230 }, // 41
    { 23,  233 }, // 42
    { 20,  226 }, // 43
    { 21,  232 }, // 44
    { 20,  227 }, // 45
    { 23,  227 }, // 46
    { 21,  218 }  // 47
};

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

    static Leds leds;
    static Timer animationTimer(millisecond_t(15));
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

    uint8_t numFailingTasks = 0;
    if (false && (!numFailingTasksQueue.peek(numFailingTasks, millisecond_t(0)) || numFailingTasks > 0)) {
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

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(can::LongitudinalState::id(), [] (const uint8_t * const data) {
        reinterpret_cast<const can::LongitudinalState*>(data)->acquire(speed, distance);
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
    SystemManager::instance().registerTask();

    LinePosCalculator linePosCalc(PANEL_VERSION_FRONT == getPanelVersion() ? FRONT_SENSOR_LIMITS : REAR_SENSOR_LIMITS);

    initializeVehicleCan();

    while (true) {
        measurementsQueue.receive(measurements);

        measurements[0]                    = measurements[1];
        measurements[cfg::NUM_SENSORS - 1] = measurements[cfg::NUM_SENSORS - 2];

        const LinePositions linePositions = linePosCalc.calculate(measurements);
        const Lines lines = lineFilter.update(linePositions);
        linePatternCalc.update(domain, lines, distance);

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

        SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut());

        updateSensorControl(lines);
        sensorControlDataQueue.send(sensorControl);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
