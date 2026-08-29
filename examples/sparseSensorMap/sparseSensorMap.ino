#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20, 0x21};

// In the newer API, each item describes:
// - which PCF8574 controls the sensor
// - which physical PCF pin is wired to XSHUT
// - which final address the sensor should receive (0 = automatic)
// - which public ID it should expose (-1 = reuse the internal index)
// - which model this slot uses (or TOF_INHERIT_DEFAULT to reuse config.model)
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p4, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p7, TOF_VL53L4CD, 2),
    LIDAR_SLOT(1, pcf_p0, TOF_VL53L4CD, 10),
    LIDAR_SLOT(1, pcf_p2, TOF_VL53L4CD, 11),
    LIDAR_SLOT(1, pcf_p4, TOF_VL53L4CD, 12),

    // The legacy 4-field form still works and inherits lidar.config().model.
    {1, pcf_p5, 0, 13},
    {1, pcf_p7, 0, 14}
};

LidarArray lidar(TOF_VL53L4CD);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // config().model remains the array default used by 4-field sparse slots.
    lidar.config().model = TOF_VL53L4CD;

    // The array order remains the internal logical order 0..N-1.
    // The sensorId field lets you expose different public names, such as 10..14.
    lidar.setLayout(2, 8, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setVL53L4CDTiming(50, 0);

    if (!lidar.begin()) {
        Serial.println("Partial initialization or at least one sensor failed.");
    }
}

void loop()
{
    // Here we read by public sensor ID, not by internal index.
    for (int16_t sensorId = 0; sensorId <= 14; ++sensorId)
    {
        const int16_t index = lidar.indexOfSensorId(sensorId);
        if (index < 0) {
            continue;
        }

        const LidarSensorSlot *slot = lidar.getSensorSlot(static_cast<uint8_t>(index));
        LidarReading reading = lidar.readReadingById(sensorId, true);

        Serial.print("ID ");
        Serial.print(sensorId);
        Serial.print(" -> idx=");
        Serial.print(index);
        Serial.print(", pcf=");
        Serial.print(slot != nullptr ? slot->pcfIndex : 255);
        Serial.print(", pin=");
        Serial.print(slot != nullptr ? slot->pin : 255);
        Serial.print(", model=");
        Serial.print(lidar.getSensorModelById(sensorId) == TOF_VL53L0X ? "VL53L0X" : "VL53L4CD");
        Serial.print(", addr=0x");
        if (reading.address < 0x10) {
            Serial.print('0');
        }
        Serial.print(reading.address, HEX);
        Serial.print(", dist=");
        Serial.print(reading.distanceMm);
        Serial.println(" mm");
    }

    Serial.println();
    delay(150);
}
