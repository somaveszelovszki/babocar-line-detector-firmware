#include <micro/panel/CanManager.hpp>
#include <micro/panel/vehicleCanTypes.hpp>
#include <micro/port/task.hpp>

#include <cfg_board.h>
#include <globals.hpp>
#include <SensorHandler.hpp>

using namespace micro;

extern CanManager vehicleCanManager;
extern queue_t<leds_t, 1> ledsQueue;

queue_t<measurements_t, 1> measurementsQueue;

namespace {

SensorHandler sensorHandler;

} // namespace

extern "C" void runSensorTask(void) {

    measurements_t measurements;
    leds_t leds;

    canFrame_t rxCanFrame;
    CanFrameHandler vehicleCanFrameHandler;

    vehicleCanFrameHandler.registerHandler(can::LineDetectControl::id(), [] (const uint8_t * const data) {
        linePatternDomain_t domain;
        reinterpret_cast<const can::LineDetectControl*>(data)->acquire(globals::indicatorLedsEnabled, globals::scanRangeRadius, domain);
    });

    const CanManager::subscriberId_t vehicleCanSubsciberId = vehicleCanManager.registerSubscriber(vehicleCanFrameHandler.identifiers());

    sensorHandler.initialize();

    while (true) {
        if (vehicleCanManager.read(vehicleCanSubsciberId, rxCanFrame)) {
            vehicleCanFrameHandler.handleFrame(rxCanFrame);
        }

        vTaskSuspendAll();
        const uint8_t scanRangeRadius = globals::scanRangeRadius;
        const uint8_t scanRangeCenter = globals::scanRangeCenter;
        xTaskResumeAll();

        const uint8_t first = scanRangeRadius > 0 ? scanRangeCenter - scanRangeRadius : 0;
        const uint8_t last  = scanRangeRadius > 0 ? scanRangeCenter + scanRangeRadius : ARRAY_SIZE(measurements) - 1;

        sensorHandler.readSensors(measurements, first, last);
        measurementsQueue.send(measurements, millisecond_t(5));

        if (ledsQueue.receive(leds, millisecond_t(0))) {
            sensorHandler.writeLeds(leds);
        }
    }
}

extern void spi_SensorTxCpltCallback() {
    sensorHandler.onTxFinished();
}

extern void spi_SensorTxRxCpltCallback() {
    sensorHandler.onTxFinished();
}
