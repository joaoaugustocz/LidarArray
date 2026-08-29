#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20};
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p0, TOF_VL53L0X, 0),
    LIDAR_SLOT(0, pcf_p1, TOF_VL53L0X, 1),
    LIDAR_SLOT(0, pcf_p2, TOF_VL53L0X, 2),
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L0X, 3)
};

LidarArray lidar(TOF_VL53L0X);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Even when the array is simple, the sparse slot map makes the boot order,
    // the physical pins, and the future migration path explicit.
    lidar.setLayout(1, 4, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setMeasurementTimingBudget(20000);
    lidar.setVcselPulsePeriod(0, 14);
    lidar.setVcselPulsePeriod(1, 10);

    // Expected boot flow:
    // 1. scan the bus before the library changes sensor state
    // 2. wait so the serial monitor can attach
    // 3. shut down all ToF sensors
    // 4. wake one slot at a time and rescan after every step
    lidar.setDebugOutput(&Serial);
    lidar.setDebugLevel(LidarDebugLevel::Verbose);
    lidar.setDebugScanBeforeInit(true);
    lidar.setDebugScanEachStep(true);
    lidar.setDebugBootDelay(1500);
    lidar.setDebugStepDelay(500);

    const bool ok = lidar.begin();
    Serial.print("begin() -> ");
    Serial.println(ok ? "OK" : "PARTIAL/FAILED");
}

void loop()
{
    for (uint8_t index = 0; index < lidar.getSensorCount(); ++index)
    {
        const LidarSensorSlot *slot = lidar.getSensorSlot(index);
        const LidarReading reading = lidar.readReading(index, false);

        Serial.print("idx=");
        Serial.print(index);
        Serial.print(" pin=");
        Serial.print(slot != nullptr ? slot->pin : 255);
        Serial.print(" ready=");
        Serial.print(lidar.isSensorReady(index) ? "yes" : "no");
        Serial.print(" status=");
        Serial.print(reading.status);
        Serial.print(" dist=");
        Serial.println(reading.distanceMm);
    }

    Serial.println();
    delay(150);
}
