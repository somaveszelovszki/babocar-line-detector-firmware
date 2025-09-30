#include <LineFilter.hpp>
#include <LinePatternCalculator.hpp>
#include <LinePosCalculator.hpp>
#include <SensorData.hpp>
#include <SensorHandler.hpp>
#include <cfg_board.hpp>
#include <numeric>

#include <micro/panel/CanManager.hpp>
#include <micro/panel/panelVersion.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/algorithm.hpp>
#include <micro/utils/timer.hpp>

#define REPORT_STATISTICS false

using namespace micro;

CanManager vehicleCanManager(can_Vehicle);

namespace {

SensorHandler sensorHandler(spi_Sensor,
                            {gpio_SS_ADC0, gpio_SS_ADC1, gpio_SS_ADC2, gpio_SS_ADC3, gpio_SS_ADC4,
                             gpio_SS_ADC5},
                            gpio_LE_OPTO, gpio_OE_OPTO, gpio_LE_IND, gpio_LE_IND);

Leds ledsControl;
std::pair<uint8_t, uint8_t> scanRange{0, cfg::NUM_SENSORS};

LinePosCalculator linePosCalc(true);
LineFilter lineFilter;
LinePatternCalculator linePatternCalc;

linePatternDomain_t domain = linePatternDomain_t::Labyrinth;
m_per_sec_t speed;
meter_t distance;
bool indicatorLedsEnabled = true;

CanFrameHandler vehicleCanFrameHandler;
CanSubscriber::Id vehicleCanSubscriberId = CanSubscriber::INVALID_ID;

#if REPORT_STATISTICS
// Statistics tracking variables
uint32_t statisticsCounter                    = 0;
millisecond_t statisticsStartTime             = millisecond_t(0);
constexpr uint16_t STATISTICS_ITERATION_COUNT = 1000;
#endif

Leds getFailureLedsControl() {
    constexpr float SENSOR_OFFSET = cfg::NUM_SENSORS / 2.0f - 0.5f;

    static Leds leds;
    static Timer animationTimer(millisecond_t(1200));
    static radian_t angle = radian_t(0);

    animationTimer.checkTimeout();
    angle = micro::lerp(getTime(), animationTimer.startTime(),
                        animationTimer.startTime() + animationTimer.period(), radian_t(0), 2 * PI);

    leds.fill(false);
    leds[static_cast<uint32_t>(std::lround(SENSOR_OFFSET + micro::cos(angle) * SENSOR_OFFSET))] =
        true;

    return leds;
}

Leds getLedsControl(const Lines& lines, const bool isOk) {
    if (!isOk) {
        return getFailureLedsControl();
    }

    Leds leds;
    leds.fill(false);

    if (!indicatorLedsEnabled) {
        return leds;
    }

    for (const Line& l : lines) {
        const uint8_t centerIdx =
            static_cast<uint8_t>(std::lround(LinePosCalculator::linePosToOptoPos(l.pos)));

        const uint8_t startIdx =
            max<uint8_t>(centerIdx, cfg::INDICATOR_LED_RADIUS) - cfg::INDICATOR_LED_RADIUS;
        const uint8_t endIdx =
            min<uint8_t>(centerIdx + cfg::INDICATOR_LED_RADIUS + 1, cfg::NUM_SENSORS);

        for (uint8_t i = startIdx; i < endIdx; ++i) {
            leds[i] = true;
        }
    }

    return leds;
}

std::pair<uint8_t, uint8_t> getScanRange(const Lines& lines, const LinePattern& linePattern) {
    if (lines.size() != 1 || linePattern.type != LinePattern::SINGLE_LINE) {
        return {0, cfg::NUM_SENSORS};
    }

    const uint8_t center =
        static_cast<uint8_t>(std::lround(LinePosCalculator::linePosToOptoPos(lines.begin()->pos)));

    if (center < cfg::REDUCED_SCAN_RANGE / 2) {
        return {0, cfg::REDUCED_SCAN_RANGE};
    } else if (center > cfg::NUM_SENSORS - cfg::REDUCED_SCAN_RANGE / 2) {
        return {cfg::NUM_SENSORS - cfg::REDUCED_SCAN_RANGE, cfg::NUM_SENSORS};
    } else {
        return {center - cfg::REDUCED_SCAN_RANGE / 2, center + cfg::REDUCED_SCAN_RANGE / 2};
    }
}

void initializeVehicleCan() {
    vehicleCanFrameHandler.registerHandler(
        can::LongitudinalState::id(), [](const uint8_t* const data) {
            bool isRemoteControlledPlaceholder;
            reinterpret_cast<const can::LongitudinalState*>(data)->acquire(
                speed, isRemoteControlledPlaceholder, distance);
        });

    vehicleCanFrameHandler.registerHandler(
        can::LineDetectControl::id(), [](const uint8_t* const data) {
            uint8_t scanRangeRadiusPlaceholder;
            reinterpret_cast<const can::LineDetectControl*>(data)->acquire(
                indicatorLedsEnabled, scanRangeRadiusPlaceholder, domain);
        });

    const CanFrameIds rxFilter = vehicleCanFrameHandler.identifiers();
    CanFrameIds txFilter       = {PANEL_VERSION_FRONT == getPanelVersion() ? can::FrontLines::id()
                                                                           : can::RearLines::id(),
                            PANEL_VERSION_FRONT == getPanelVersion() ? can::FrontLinePattern::id()
                                                                           : can::RearLinePattern::id()};
#if REPORT_STATISTICS
    if (PANEL_VERSION_FRONT == getPanelVersion()) {
        txFilter.insert(can::FrontLineStatistics::id());
    } else if (PANEL_VERSION_REAR == getPanelVersion()) {
        txFilter.insert(can::RearLineStatistics::id());
    }
#endif // REPORT_STATISTICS

    vehicleCanSubscriberId = vehicleCanManager.registerSubscriber(rxFilter, txFilter);
}

} // namespace

extern "C" void runLineCalcTask(void) {
    initializeVehicleCan();
    sensorHandler.initialize();

#if REPORT_STATISTICS
    statisticsStartTime = getTime();
#endif

    while (true) {
        sensorHandler.writeLeds(ledsControl);

        const auto measurements           = sensorHandler.readSensors(scanRange);
        const auto maxLines               = domain == linePatternDomain_t::Labyrinth ? 4 : 3;
        const LinePositions linePositions = linePosCalc.calculate(measurements, maxLines);
        const Lines lines                 = lineFilter.update(linePositions, maxLines);
        linePatternCalc.update(domain, lines, distance,
                               PANEL_VERSION_FRONT == getPanelVersion() ? sgn(speed) : -sgn(speed));

        if (PANEL_VERSION_FRONT == getPanelVersion()) {
            vehicleCanManager.send<can::FrontLines>(vehicleCanSubscriberId, lines);
            vehicleCanManager.send<can::FrontLinePattern>(vehicleCanSubscriberId,
                                                          linePatternCalc.pattern());
        } else if (PANEL_VERSION_REAR == getPanelVersion()) {
            vehicleCanManager.send<can::RearLines>(vehicleCanSubscriberId, lines);
            vehicleCanManager.send<can::RearLinePattern>(vehicleCanSubscriberId,
                                                         linePatternCalc.pattern());
        }

#if REPORT_STATISTICS
        statisticsCounter++;
        if (statisticsCounter == STATISTICS_ITERATION_COUNT) {
            const millisecond_t endTime = getTime();
            const uint32_t processingTime_ms =
                static_cast<uint32_t>((endTime - statisticsStartTime).get());

            if (PANEL_VERSION_FRONT == getPanelVersion()) {
                vehicleCanManager.send<can::FrontLineStatistics>(
                    vehicleCanSubscriberId, processingTime_ms, STATISTICS_ITERATION_COUNT);
            } else if (PANEL_VERSION_REAR == getPanelVersion()) {
                vehicleCanManager.send<can::RearLineStatistics>(
                    vehicleCanSubscriberId, processingTime_ms, STATISTICS_ITERATION_COUNT);
            }

            statisticsCounter   = 0;
            statisticsStartTime = endTime;
        }
#endif

        while (const auto frame = vehicleCanManager.read(vehicleCanSubscriberId)) {
            vehicleCanFrameHandler.handleFrame(*frame);
        }

        const bool isOk = !vehicleCanManager.hasTimedOut(vehicleCanSubscriberId);

        ledsControl = getLedsControl(lines, isOk);
        scanRange   = getScanRange(lines, linePatternCalc.pattern());
    }
}

void micro_Vehicle_Can_RxFifoMsgPendingCallback() {
    vehicleCanManager.onFrameReceived();
}
