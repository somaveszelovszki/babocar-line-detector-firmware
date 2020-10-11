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
//       0             1             2             3             4             5             6             7
   { 139, 252 }, { 9,   146 }, { 14,  121 }, { 13,  170 }, { 13,  93  }, { 13,  161 }, { 13,  68  }, { 13,  133 }, // 0-7
   { 8,   208 }, { 8,   205 }, { 4,   49  }, { 5,   104 }, { 6,   117 }, { 6,   130 }, { 7,   90  }, { 7,   116 }, // 8-15
   { 13,  183 }, { 14,  117 }, { 13,  109 }, { 12,  118 }, { 11,  102 }, { 11,  108 }, { 11,  149 }, { 11,  170 }, // 16-23
   { 6,   148 }, { 7,   128 }, { 10,  100 }, { 11,  120 }, { 11,  115 }, { 11,  137 }, { 11,  119 }, { 11,  124 }, // 24-31
   { 12,  139 }, { 8,   105 }, { 6,   120 }, { 6,   104 }, { 7,   101 }, { 7,   129 }, { 9,   211 }, { 10,  213 }, // 32-39
   { 15,  207 }, { 18,  197 }, { 18,  212 }, { 16,  197 }, { 16,  198 }, { 15,  197 }, { 15,  204 }, { 14,  200 }  // 40-47
};

constexpr std::pair<uint8_t, uint8_t> REAR_SENSOR_LIMITS[cfg::NUM_SENSORS] = {
//       0             1             2             3             4             5             6             7
   { 139, 252 }, { 9,   146 }, { 14,  121 }, { 13,  170 }, { 13,  93  }, { 13,  161 }, { 13,  68  }, { 13,  133 }, // 0-7
   { 8,   208 }, { 8,   205 }, { 4,   49  }, { 5,   104 }, { 6,   117 }, { 6,   130 }, { 7,   90  }, { 7,   116 }, // 8-15
   { 13,  183 }, { 14,  117 }, { 13,  109 }, { 12,  118 }, { 11,  102 }, { 11,  108 }, { 11,  149 }, { 11,  170 }, // 16-23
   { 6,   148 }, { 7,   128 }, { 10,  100 }, { 11,  120 }, { 11,  115 }, { 11,  137 }, { 11,  119 }, { 11,  124 }, // 24-31
   { 12,  139 }, { 8,   105 }, { 6,   120 }, { 6,   104 }, { 7,   101 }, { 7,   129 }, { 9,   211 }, { 10,  213 }, // 32-39
   { 15,  207 }, { 18,  197 }, { 18,  212 }, { 16,  197 }, { 16,  198 }, { 15,  197 }, { 15,  204 }, { 14,  200 }  // 40-47
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
    const CanFrameIds txFilter = {};
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
        } else if (PANEL_VERSION_REAR == getPanelVersion()) {
            vehicleCanManager.periodicSend<can::RearLines>(vehicleCanSubscriberId, lines);
        }

        while (vehicleCanManager.read(vehicleCanSubscriberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut());

//        sensorControl.leds.reset();
//        sensorControl.leds.set(1, true);
//        sensorControl.leds.set(3, true);
//        sensorControl.leds.set(micro::round(speed.get()) % 48, true);

        updateSensorControl(lines);
        sensorControlDataQueue.send(sensorControl);
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
