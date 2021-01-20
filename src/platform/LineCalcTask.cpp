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
    { 23, 219 }, // 0
    { 12, 225 }, // 1
    { 17, 219 }, // 2
    { 18, 217 }, // 3
    { 2,  221 }, // 4
    { 23, 222 }, // 5
    { 4,  205 }, // 6
    { 23, 216 }, // 7
    { 4,  231 }, // 8
    { 43, 236 }, // 9
    { 15, 226 }, // 10
    { 23, 215 }, // 11
    { 16, 221 }, // 12
    { 12, 213 }, // 13
    { 23, 208 }, // 14
    { 8,  204 }, // 15
    { 4,  222 }, // 16
    { 8,  217 }, // 17
    { 4,  196 }, // 18
    { 4,  214 }, // 19
    { 19, 213 }, // 20
    { 27, 227 }, // 21
    { 8,  216 }, // 22
    { 4,  221 }, // 23
    { 15, 219 }, // 24
    { 8,  222 }, // 25
    { 23, 213 }, // 26
    { 4,  198 }, // 27
    { 4,  215 }, // 28
    { 4,  206 }, // 29
    { 10, 217 }, // 30
    { 5,  215 }, // 31
    { 11, 189 }, // 32
    { 17, 186 }, // 33
    { 21, 198 }, // 34
    { 4,  206 }, // 35
    { 8,  166 }, // 36
    { 8,  212 }, // 37
    { 23, 239 }, // 38
    { 55, 243 }, // 39
    { 40, 232 }, // 40
    { 10, 231 }, // 41
    { 24, 231 }, // 42
    { 12, 234 }, // 43
    { 19, 230 }, // 44
    { 15, 236 }, // 45
    { 32, 227 }, // 46
    { 39, 221 }  // 47
};

constexpr std::pair<uint8_t, uint8_t> REAR_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 0,  204 }, // 0
    { 34, 240 }, // 1
    { 8,  242 }, // 2
    { 28, 242 }, // 3
    { 18, 218 }, // 4
    { 16, 243 }, // 5
    { 5,  238 }, // 6
    { 97, 238 }, // 7
    { 8,  232 }, // 8
    { 56, 235 }, // 9
    { 29, 219 }, // 10
    { 4,  229 }, // 11
    { 23, 230 }, // 12
    { 55, 242 }, // 13
    { 4,  204 }, // 14
    { 63, 249 }, // 15
    { 4,  197 }, // 16
    { 4,  170 }, // 17
    { 50, 237 }, // 18
    { 4,  209 }, // 19
    { 20, 225 }, // 20
    { 4,  184 }, // 21
    { 23, 241 }, // 22
    { 9,  200 }, // 23
    { 12, 229 }, // 24
    { 16, 217 }, // 25
    { 58, 230 }, // 26
    { 8,  223 }, // 27
    { 47, 231 }, // 28
    { 31, 231 }, // 29
    { 26, 216 }, // 30
    { 17, 223 }, // 31
    { 16, 228 }, // 32
    { 4,  185 }, // 33
    { 12, 218 }, // 34
    { 8,  224 }, // 35
    { 12, 229 }, // 36
    { 8,  220 }, // 37
    { 15, 237 }, // 38
    { 63, 229 }, // 39
    { 8,  226 }, // 40
    { 16, 228 }, // 41
    { 4,  181 }, // 42
    { 79, 246 }, // 43
    { 15, 220 }, // 44
    { 43, 239 }, // 45
    { 8,  208 }, // 46
    { 2,  151 }  // 47
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
