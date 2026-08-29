#include <LidarArray.h>

// Minimal recommended setup:
// - one PCF8574
// - four VL53L4CD sensors
// - sparse slot map, even though the wiring is simple
//
// Each slot explicitly tells the library:
// - which PCF controls XSHUT
// - which physical PCF pin is used
// - which model is connected there
// - which public ID the application should use
const uint8_t pcf8574Addresses[] = {0x20};
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p0, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p1, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p2, TOF_VL53L4CD, 2),
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 3)
};

LidarArray lidar(TOF_VL53L4CD);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(1, 4, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setVL53L4CDTiming(50, 0);

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
        Serial.print(" id=");
        Serial.print(reading.sensorId);
        Serial.print(" pcf=");
        Serial.print(slot != nullptr ? slot->pcfIndex : 255);
        Serial.print(" pin=");
        Serial.print(slot != nullptr ? slot->pin : 255);
        Serial.print(" addr=0x");
        if (reading.address < 0x10) {
            Serial.print('0');
        }
        Serial.print(reading.address, HEX);
        Serial.print(" dist=");
        Serial.print(reading.distanceMm);
        Serial.println(" mm");
    }

    Serial.println();
    delay(120);
}
