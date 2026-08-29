#include <LidarArray.h>

// This example keeps the same model on every slot, but spreads the array
// across two PCF8574 expanders. The sensor order follows the order of this
// table, not the numeric order of the pins.
const uint8_t pcf8574Addresses[] = {0x20, 0x21};
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p0, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p2, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p5, TOF_VL53L4CD, 2),
    LIDAR_SLOT(0, pcf_p7, TOF_VL53L4CD, 3),
    LIDAR_SLOT(1, pcf_p1, TOF_VL53L4CD, 4),
    LIDAR_SLOT(1, pcf_p3, TOF_VL53L4CD, 5),
    LIDAR_SLOT(1, pcf_p4, TOF_VL53L4CD, 6),
    LIDAR_SLOT(1, pcf_p6, TOF_VL53L4CD, 7)
};

LidarArray lidar(TOF_VL53L4CD);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(2, 8, pcf8574Addresses, sensorMap);
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
        Serial.print(" dist=");
        Serial.print(reading.distanceMm);
        Serial.println(" mm");
    }

    Serial.println();
    delay(140);
}
