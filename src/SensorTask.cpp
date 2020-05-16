#include <micro/debug/DebugLed.hpp>
#include <micro/debug/SystemManager.hpp>
#include <micro/panel/CanManager.hpp>
#include <micro/panel/vehicleCanTypes.hpp>
#include <micro/port/task.hpp>
#include <micro/utils/log.hpp>
#include <micro/utils/str_utils.hpp>

#include <cfg_board.h>
#include <SensorHandler.hpp>

using namespace micro;

extern CanManager vehicleCanManager;
extern queue_t<SensorHandlerData, 1> sensorHandlerDataQueue;

queue_t<measurements_t, 1> measurementsQueue;

namespace {

SensorHandler sensorHandler;
uint8_t scanRangeRadius = 0;

const leds_t& updateFailureLeds() {

    static leds_t leds;
    static Timer blinkTimer(DebugLed::period_NOK());

    if (blinkTimer.checkTimeout()) {
        for (uint8_t i = 0; i < cfg::NUM_SENSORS; i += 8) {
            leds.set(i, !leds.get(i));
        }
    }
    return leds;
}

} // namespace

extern "C" void runSensorTask(void) {

    SystemManager::instance().registerTask();

    measurements_t measurements;
    SensorHandlerData sensorHandlerData;

    canFrame_t rxCanFrame;
    CanFrameHandler vehicleCanFrameHandler;

    vehicleCanFrameHandler.registerHandler(can::LineDetectControl::id(), [] (const uint8_t * const data) {
        bool indicatorLedsEnabled;
        linePatternDomain_t domain;
        reinterpret_cast<const can::LineDetectControl*>(data)->acquire(indicatorLedsEnabled, scanRangeRadius, domain);
    });

    const CanManager::subscriberId_t vehicleCanSubsciberId = vehicleCanManager.registerSubscriber(vehicleCanFrameHandler.identifiers());

    sensorHandler.initialize();

    while (true) {
        if (vehicleCanManager.read(vehicleCanSubsciberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        const uint8_t first = scanRangeRadius > 0 ? sensorHandlerData.scanRangeCenter - scanRangeRadius : 0;
        const uint8_t last  = scanRangeRadius > 0 ? sensorHandlerData.scanRangeCenter + scanRangeRadius : ARRAY_SIZE(measurements) - 1;

        sensorHandler.readSensors(measurements, first, last);
        measurementsQueue.send(measurements, millisecond_t(5));

        sensorHandlerDataQueue.receive(sensorHandlerData, millisecond_t(0));

        sensorHandler.writeLeds(SystemManager::instance().failingTasks().size() > 0 ? sensorHandlerData.leds : updateFailureLeds());

        SystemManager::instance().notify(!vehicleCanManager.hasRxTimedOut());
    }
}

extern void spi_SensorTxCpltCallback() {
    sensorHandler.onTxFinished();
}

extern void spi_SensorTxRxCpltCallback() {
    sensorHandler.onTxFinished();
}
