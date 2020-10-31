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
    { 13,  231 }, // 0
    { 11,  186 }, // 1
    { 11,  185 }, // 2
    { 9,   203 }, // 3
    { 9,   193 }, // 4
    { 10,  207 }, // 5
    { 11,  153 }, // 6
    { 12,  198 }, // 7
    { 11,  217 }, // 8
    { 14,  226 }, // 9
    { 11,  142 }, // 10
    { 11,  163 }, // 11
    { 11,  169 }, // 12
    { 11,  191 }, // 13
    { 11,  169 }, // 14
    { 11,  153 }, // 15
    { 11,  210 }, // 16
    { 11,  176 }, // 17
    { 11,  164 }, // 18
    { 11,  169 }, // 19
    { 11,  172 }, // 20
    { 12,  195 }, // 21
    { 12,  198 }, // 22
    { 12,  207 }, // 23
    { 11,  192 }, // 24
    { 11,  182 }, // 25
    { 11,  168 }, // 26
    { 11,  186 }, // 27
    { 11,  170 }, // 28
    { 11,  178 }, // 29
    { 11,  179 }, // 30
    { 11,  187 }, // 31
    { 11,  186 }, // 32
    { 11,  173 }, // 33
    { 11,  170 }, // 34
    { 11,  157 }, // 35
    { 11,  155 }, // 36
    { 11,  192 }, // 37
    { 17,  235 }, // 38
    { 44,  232 }, // 39
    { 43,  224 }, // 40
    { 18,  219 }, // 41
    { 28,  229 }, // 42
    { 18,  221 }, // 43
    { 21,  231 }, // 44
    { 18,  219 }, // 45
    { 32,  227 }, // 46
    { 22,  217 }  // 47
};

constexpr std::pair<uint8_t, uint8_t> REAR_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 13,  220 }, // 0
    { 84,  232 }, // 1
    { 16,  199 }, // 2
    { 98,  236 }, // 3
    { 14,  210 }, // 4
    { 70,  231 }, // 5
    { 14,  216 }, // 6
    { 125, 240 }, // 7
    { 14,  227 }, // 8
    { 70,  235 }, // 9
    { 11,  208 }, // 10
    { 13,  175 }, // 11
    { 12,  219 }, // 12
    { 70,  242 }, // 13
    { 11,  200 }, // 14
    { 67,  237 }, // 15
    { 13,  198 }, // 16
    { 12,  140 }, // 17
    { 91,  234 }, // 18
    { 11,  203 }, // 19
    { 46,  227 }, // 20
    { 12,  160 }, // 21
    { 57,  231 }, // 22
    { 13,  189 }, // 23
    { 46,  231 }, // 24
    { 13,  184 }, // 25
    { 82,  239 }, // 26
    { 13,  199 }, // 27
    { 65,  227 }, // 28
    { 17,  223 }, // 29
    { 63,  230 }, // 30
    { 12,  202 }, // 31
    { 55,  233 }, // 32
    { 11,  175 }, // 33
    { 42,  230 }, // 34
    { 24,  225 }, // 35
    { 54,  230 }, // 36
    { 40,  229 }, // 37
    { 44,  235 }, // 38
    { 94,  232 }, // 39
    { 37,  225 }, // 40
    { 69,  232 }, // 41
    { 13,  172 }, // 42
    { 119, 235 }, // 43
    { 42,  227 }, // 44
    { 78,  232 }, // 45
    { 28,  220 }, // 46
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

        SystemManager::instance().notify(true || !vehicleCanManager.hasRxTimedOut());

        updateSensorControl(lines);
        sensorControlDataQueue.send(sensorControl);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
