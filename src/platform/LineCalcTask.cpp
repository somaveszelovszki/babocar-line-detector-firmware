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
    { 24,  219 }, // 0
    { 24,  225 }, // 1
    { 31,  219 }, // 2
    { 8,   217 }, // 3
    { 18,  221 }, // 4
    { 32,  222 }, // 5
    { 12,  205 }, // 6
    { 18,  216 }, // 7
    { 16,  231 }, // 8
    { 107, 236 }, // 9
    { 8,   226 }, // 10
    { 31,  215 }, // 11
    { 31,  221 }, // 12
    { 27,  213 }, // 13
    { 20,  208 }, // 14
    { 23,  204 }, // 15
    { 8,   222 }, // 16
    { 15,  217 }, // 17
    { 28,  196 }, // 18
    { 27,  214 }, // 19
    { 4,   213 }, // 20
    { 29,  227 }, // 21
    { 21,  216 }, // 22
    { 20,  221 }, // 23
    { 27,  219 }, // 24
    { 8,   222 }, // 25
    { 8,   213 }, // 26
    { 31,  198 }, // 27
    { 31,  215 }, // 28
    { 8,   206 }, // 29
    { 8,   217 }, // 30
    { 31,  215 }, // 31
    { 8,   189 }, // 32
    { 31,  186 }, // 33
    { 8,   198 }, // 34
    { 8,   206 }, // 35
    { 4,   166 }, // 36
    { 16,  212 }, // 37
    { 106, 239 }, // 38
    { 139, 243 }, // 39
    { 116, 232 }, // 40
    { 85,  231 }, // 41
    { 127, 231 }, // 42
    { 92,  234 }, // 43
    { 104, 230 }, // 44
    { 92,  236 }, // 45
    { 116, 227 }, // 46
    { 100, 221 }  // 47
};

constexpr std::pair<uint8_t, uint8_t> REAR_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 11,  204 }, // 0
    { 135, 240 }, // 1
    { 97,  242 }, // 2
    { 108, 242 }, // 3
    { 9,   218 }, // 4
    { 119, 243 }, // 5
    { 54,  238 }, // 6
    { 158, 238 }, // 7
    { 58,  232 }, // 8
    { 120, 235 }, // 9
    { 54,  219 }, // 10
    { 31,  229 }, // 11
    { 56,  230 }, // 12
    { 128, 242 }, // 13
    { 40,  204 }, // 14
    { 120, 249 }, // 15
    { 16,  197 }, // 16
    { 8,   170 }, // 17
    { 143, 237 }, // 18
    { 4,   209 }, // 19
    { 107, 225 }, // 20
    { 8,   184 }, // 21
    { 100, 241 }, // 22
    { 8,   200 }, // 23
    { 109, 229 }, // 24
    { 15,  217 }, // 25
    { 143, 230 }, // 26
    { 8,   223 }, // 27
    { 114, 231 }, // 28
    { 55,  231 }, // 29
    { 116, 216 }, // 30
    { 19,  223 }, // 31
    { 108, 228 }, // 32
    { 9,   185 }, // 33
    { 111, 218 }, // 34
    { 87,  224 }, // 35
    { 96,  229 }, // 36
    { 88,  220 }, // 37
    { 102, 237 }, // 38
    { 137, 229 }, // 39
    { 80,  226 }, // 40
    { 104, 228 }, // 41
    { 15,  181 }, // 42
    { 155, 246 }, // 43
    { 84,  220 }, // 44
    { 127, 239 }, // 45
    { 68,  208 }, // 46
    { 8,   151 }  // 47
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
    SystemManager::instance().registerTask();

    LinePosCalculator linePosCalc(PANEL_VERSION_FRONT == getPanelVersion() ? FRONT_SENSOR_LIMITS : REAR_SENSOR_LIMITS);

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

        SystemManager::instance().notify(!vehicleCanManager.hasTimedOut(vehicleCanSubscriberId));

        updateSensorControl(lines);
        sensorControlDataQueue.send(sensorControl);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
