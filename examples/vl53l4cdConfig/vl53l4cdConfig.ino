#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20};

// Automatic addressing uses 0x30 + internal index.
const LidarSensorSlot automaticSensorMap[] = {
    LIDAR_SLOT(0, pcf_p0, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p1, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p2, TOF_VL53L4CD, 2),
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 3)
};

// Manual addressing is declared directly in the slot map.
const LidarSensorSlot manualAddressSensorMap[] = {
    LIDAR_SLOT_ADDR(0, pcf_p0, 0x30, TOF_VL53L4CD, 0),
    LIDAR_SLOT_ADDR(0, pcf_p1, 0x32, TOF_VL53L4CD, 1),
    LIDAR_SLOT_ADDR(0, pcf_p2, 0x34, TOF_VL53L4CD, 2),
    LIDAR_SLOT_ADDR(0, pcf_p3, 0x36, TOF_VL53L4CD, 3)
};

const bool useCustomAddresses = false;

LidarArray lidar(TOF_VL53L4CD);

uint8_t expectedAddressForIndex(uint8_t index)
{
    const LidarSensorSlot *slot = lidar.getSensorSlot(index);
    if (slot != nullptr && slot->address != 0) {
        return slot->address;
    }
    return 0x30 + index;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(
        1,
        4,
        pcf8574Addresses,
        useCustomAddresses ? manualAddressSensorMap : automaticSensorMap
    );

    lidar.setTimeout(100);
    lidar.setVL53L4CDTiming(50, 0);
    lidar.setDebugOutput(&Serial);
    lidar.setDebugLevel(LidarDebugLevel::Info);
    lidar.setDebugScanBeforeInit(true);

    if (!lidar.begin()) {
        Serial.println("Partial initialization or at least one sensor failed.");
    }
}

void loop()
{
    for (uint8_t index = 0; index < lidar.getSensorCount(); ++index) {
        const LidarSensorSlot *slot = lidar.getSensorSlot(index);
        const LidarReading reading = lidar.readReading(index, true);

        Serial.print("idx=");
        Serial.print(index);
        Serial.print(" pin=");
        Serial.print(slot != nullptr ? slot->pin : 255);
        Serial.print(" expected=0x");
        if (expectedAddressForIndex(index) < 0x10) {
            Serial.print('0');
        }
        Serial.print(expectedAddressForIndex(index), HEX);
        Serial.print(" actual=0x");
        if (reading.address < 0x10) {
            Serial.print('0');
        }
        Serial.print(reading.address, HEX);
        Serial.print(" status=");
        Serial.print(reading.status);
        Serial.print(" dist=");
        Serial.print(reading.distanceMm);
        Serial.println(" mm");
    }

    Serial.println();
    delay(140);
}
