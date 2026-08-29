#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20};

// The physical wiring did not change. Only the order of the slots changed.
// Because the array order defines the internal logical order, the first entry
// becomes index 0, the second entry becomes index 1, and so on.
//
// Physical wiring:
// - pin 3 should be treated as logical sensor 0
// - pin 1 should be treated as logical sensor 1
// - pin 2 should be treated as logical sensor 2
// - pin 0 should be treated as logical sensor 3
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p1, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p2, TOF_VL53L4CD, 2),
    LIDAR_SLOT(0, pcf_p0, TOF_VL53L4CD, 3)
};

LidarArray lidar(TOF_VL53L4CD);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(1, 4, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setVL53L4CDTiming(50, 0);

    Serial.println("Logical remap example.");
    Serial.println("The order of sensorMap defines the logical order.");

    if (!lidar.begin()) {
        Serial.println("Partial initialization detected.");
    }
}

void loop()
{
    for (uint8_t index = 0; index < lidar.getSensorCount(); ++index)
    {
        const LidarSensorSlot *slot = lidar.getSensorSlot(index);
        const LidarReading reading = lidar.readReading(index, true);

        Serial.print("idx=");
        Serial.print(index);
        Serial.print(" physical pin=");
        Serial.print(slot != nullptr ? slot->pin : 255);
        Serial.print(" dist=");
        Serial.print(reading.distanceMm);
        Serial.println(" mm");
    }

    Serial.println();
    delay(140);
}
