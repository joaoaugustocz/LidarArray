#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20, 0x21};

// This example shows the full recommended flow:
// - sparse slot map
// - explicit model per slot
// - optional manual address on selected sensors
// - one shared LidarArray instance for both models
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p4, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p7, TOF_VL53L4CD, 2),
    LIDAR_SLOT(1, pcf_p0, TOF_VL53L0X, 10),
    LIDAR_SLOT(1, pcf_p2, TOF_VL53L0X, 11),
    LIDAR_SLOT(1, pcf_p4, TOF_VL53L0X, 12),
    LIDAR_SLOT_ADDR(1, pcf_p5, 0x3A, TOF_VL53L0X, 13),
    LIDAR_SLOT(1, pcf_p7, TOF_VL53L0X, 14)
};

LidarArray lidar(TOF_VL53L4CD);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(2, 8, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);

    // These settings only affect VL53L4CD slots.
    lidar.setVL53L4CDTiming(50, 0);

    // These settings only affect VL53L0X slots.
    lidar.setMeasurementTimingBudget(20000);
    lidar.setVcselPulsePeriod(0, 14);
    lidar.setVcselPulsePeriod(1, 10);

    if (!lidar.begin()) {
        Serial.println("Partial initialization or at least one sensor failed.");
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
        Serial.print(" model=");
        Serial.print(reading.model == TOF_VL53L0X ? "VL53L0X" : "VL53L4CD");
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
    delay(150);
}
