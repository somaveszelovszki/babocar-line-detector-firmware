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

CanManager vehicleCanManager(can_Vehicle);
queue_t<SensorControlData, 1> sensorControlDataQueue;

namespace {

constexpr std::pair<uint8_t, uint8_t> FRONT_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 80,  231 }, // 0
    { 10,  186 }, // 1
    { 11,  185 }, // 2
    { 9,   203 }, // 3
    { 9,   193 }, // 4
    { 21,  207 }, // 5
    { 11,  153 }, // 6
    { 12,  198 }, // 7
    { 22,  217 }, // 8
    { 96,  226 }, // 9
    { 15,  142 }, // 10
    { 15,  163 }, // 11
    { 14,  169 }, // 12
    { 15,  191 }, // 13
    { 14,  169 }, // 14
    { 14,  153 }, // 15
    { 14,  210 }, // 16
    { 13,  176 }, // 17
    { 13,  164 }, // 18
    { 13,  169 }, // 19
    { 13,  172 }, // 20
    { 15,  195 }, // 21
    { 15,  198 }, // 22
    { 15,  207 }, // 23
    { 10,  192 }, // 24
    { 11,  182 }, // 25
    { 11,  168 }, // 26
    { 12,  186 }, // 27
    { 12,  170 }, // 28
    { 12,  178 }, // 29
    { 12,  179 }, // 30
    { 12,  187 }, // 31
    { 16,  186 }, // 32
    { 16,  173 }, // 33
    { 15,  170 }, // 34
    { 15,  157 }, // 35
    { 15,  155 }, // 36
    { 15,  192 }, // 37
    { 115, 235 }, // 38
    { 138, 232 }, // 39
    { 135, 224 }, // 40
    { 100, 219 }, // 41
    { 124, 229 }, // 42
    { 104, 221 }, // 43
    { 114, 231 }, // 44
    { 101, 219 }, // 45
    { 124, 227 }, // 46
    { 113, 217 }  // 47
};

constexpr std::pair<uint8_t, uint8_t> REAR_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 52,  220 }, // 0
    { 130, 232 }, // 1
    { 28,  199 }, // 2
    { 140, 236 }, // 3
    { 13,  210 }, // 4
    { 119, 231 }, // 5
    { 28,  216 }, // 6
    { 163, 240 }, // 7
    { 55,  227 }, // 8
    { 119, 235 }, // 9
    { 14,  208 }, // 10
    { 13,  175 }, // 11
    { 14,  219 }, // 12
    { 116, 242 }, // 13
    { 12,  200 }, // 14
    { 120, 237 }, // 15
    { 12,  198 }, // 16
    { 13,  140 }, // 17
    { 130, 234 }, // 18
    { 13,  203 }, // 19
    { 97,  227 }, // 20
    { 12,  160 }, // 21
    { 105, 231 }, // 22
    { 13,  189 }, // 23
    { 93,  231 }, // 24
    { 11,  184 }, // 25
    { 123, 239 }, // 26
    { 13,  199 }, // 27
    { 110, 227 }, // 28
    { 37,  223 }, // 29
    { 108, 230 }, // 30
    { 13,  202 }, // 31
    { 105, 233 }, // 32
    { 13,  175 }, // 33
    { 92,  230 }, // 34
    { 72,  225 }, // 35
    { 99,  230 }, // 36
    { 86,  229 }, // 37
    { 94,  235 }, // 38
    { 127, 232 }, // 39
    { 75,  225 }, // 40
    { 103, 232 }, // 41
    { 12,  172 }, // 42
    { 142, 235 }, // 43
    { 75,  227 }, // 44
    { 113, 232 }, // 45
    { 67,  220 }, // 46
    { 11,  124 }  // 47
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

    static constexpr float SENSOR_OFFSET = cfg::NUM_SENSORS / 2.0f - 0.5f;

    static Leds leds;
    static Timer animationTimer(millisecond_t(1200));

    static radian_t angle = radian_t(0);

    animationTimer.checkTimeout();
    angle = map(getTime(), animationTimer.startTime(), animationTimer.startTime() + animationTimer.period(), radian_t(0), 2 * PI);

    leds.reset();
    leds.set(static_cast<uint32_t>(micro::round(SENSOR_OFFSET + micro::cos(angle) * SENSOR_OFFSET)), true);

    return leds;
}

void updateSensorControl(const Lines& lines) {
    static constexpr uint8_t LED_RADIUS = 1;

    uint8_t numFailingTasks = 0;
    if (!numFailingTasksQueue.peek(numFailingTasks, millisecond_t(0)) || numFailingTasks > 0) {
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
    SystemManager::instance().registerTask();

    LinePosCalculator linePosCalc(PANEL_VERSION_FRONT == getPanelVersion() ? FRONT_SENSOR_LIMITS : REAR_SENSOR_LIMITS);

    initializeVehicleCan();

    while (true) {
        measurementsQueue.receive(measurements);

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

        SystemManager::instance().notify(!vehicleCanManager.hasTimedOut(vehicleCanSubscriberId));

        updateSensorControl(lines);
        sensorControlDataQueue.send(sensorControl);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
