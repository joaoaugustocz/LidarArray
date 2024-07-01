#ifndef LidarArray_h
#define LidarArray_h

#include <Wire.h>
#include <VL53L0X.h>
#include <vector>

class LidarArray {
public:
    LidarArray(uint8_t numPCF, uint8_t numSensors, const uint8_t pcf8574Addresses[], const uint8_t xshutPins[][8]);
    bool initSensors();
    bool initSensors(int measurementT);
    bool initSensors(int measurementT, uint8_t preRange, uint8_t FinalRange);
    bool initSensors(int measurementT, uint8_t preRange, uint8_t FinalRange, int timeout);
    uint16_t readSensor(uint8_t sensorIndex);
    uint16_t readSensorNB(uint8_t sensorIndex);
    VL53L0X& getSensor(uint8_t sensorIndex);
    void setMeasurementTimingBudget(uint32_t timingBudget);
    void setVcselPulsePeriod(uint8_t type, uint8_t period);
    uint8_t getSensorCount();

private:
    uint8_t numPCF;
    uint8_t numSensors;
    std::vector<VL53L0X> sensors;
    std::vector<std::vector<uint8_t>> xshutPins;
    std::vector<uint8_t> pcf8574Addresses;
    std::vector<uint8_t> pcf8574States;

    int LidarMeasurementTimingBudget = 20000;
    int LidarPulsePeriodPreRange = 8;
    int LidarPulsePeriodFinalRange = 6;
    int LidarTimeout = 0;

    void pcf8574Write(uint8_t pcf8574Index, uint8_t pin, bool state);
    void pcf8574PinMode(uint8_t pcf8574Index, uint8_t pin, bool mode);
    uint16_t readSensorNonBlocking(VL53L0X &sensor);
};

#endif