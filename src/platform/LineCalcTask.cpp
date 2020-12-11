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
    { 114, 240 }, // 0
    { 15,  198 }, // 1
    { 16,  189 }, // 2
    { 15,  208 }, // 3
    { 15,  193 }, // 4
    { 30,  217 }, // 5
    { 15,  177 }, // 6
    { 18,  188 }, // 7
    { 28,  186 }, // 8
    { 104, 219 }, // 9
    { 10,  115 }, // 10
    { 10,  162 }, // 11
    { 10,  152 }, // 12
    { 10,  151 }, // 13
    { 10,  144 }, // 14
    { 10,  147 }, // 15
    { 15,  201 }, // 16
    { 16,  142 }, // 17
    { 15,  130 }, // 18
    { 14,  136 }, // 19
    { 14,  124 }, // 20
    { 17,  160 }, // 21
    { 15,  172 }, // 22
    { 16,  175 }, // 23
    { 11,  158 }, // 24
    { 14,  160 }, // 25
    { 15,  183 }, // 26
    { 15,  162 }, // 27
    { 15,  166 }, // 28
    { 15,  156 }, // 29
    { 15,  153 }, // 30
    { 16,  190 }, // 31
    { 13,  195 }, // 32
    { 11,  165 }, // 33
    { 11,  145 }, // 34
    { 12,  145 }, // 35
    { 12,  149 }, // 36
    { 12,  197 }, // 37
    { 102, 232 }, // 38
    { 125, 232 }, // 39
    { 123, 234 }, // 40
    { 85,  220 }, // 41
    { 110, 228 }, // 42
    { 88,  227 }, // 43
    { 97,  231 }, // 44
    { 82,  217 }, // 45
    { 108, 224 }, // 46
    { 92,  221 }  // 47
};

constexpr std::pair<uint8_t, uint8_t> REAR_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
    { 59,  239 }, // 0
    { 133, 240 }, // 1
    { 30,  208 }, // 2
    { 143, 242 }, // 3
    { 11,  196 }, // 4
    { 124, 236 }, // 5
    { 34,  216 }, // 6
    { 165, 243 }, // 7
    { 61,  228 }, // 8
    { 123, 236 }, // 9
    { 14,  205 }, // 10
    { 15,  184 }, // 11
    { 16,  212 }, // 12
    { 121, 234 }, // 13
    { 14,  207 }, // 14
    { 124, 236 }, // 15
    { 12,  209 }, // 16
    { 14,  170 }, // 17
    { 136, 237 }, // 18
    { 13,  202 }, // 19
    { 101, 229 }, // 20
    { 14,  180 }, // 21
    { 111, 232 }, // 22
    { 15,  189 }, // 23
    { 99,  233 }, // 24
    { 12,  185 }, // 25
    { 127, 241 }, // 26
    { 12,  196 }, // 27
    { 114, 233 }, // 28
    { 43,  223 }, // 29
    { 110, 232 }, // 30
    { 10,  198 }, // 31
    { 109, 232 }, // 32
    { 11,  184 }, // 33
    { 97,  234 }, // 34
    { 78,  230 }, // 35
    { 103, 230 }, // 36
    { 91,  231 }, // 37
    { 94,  234 }, // 38
    { 130, 236 }, // 39
    { 82,  230 }, // 40
    { 108, 228 }, // 41
    { 15,  180 }, // 42
    { 149, 235 }, // 43
    { 85,  222 }, // 44
    { 117, 227 }, // 45
    { 73,  216 }, // 46
    { 13,  83  }  // 47
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
