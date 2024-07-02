#include "LidarArray.h"

LidarArray::LidarArray(uint8_t numPCF, uint8_t numSensors, const uint8_t pcf8574Addresses[], const uint8_t xshutPins[][8]) {
    this->numPCF = numPCF;
    this->numSensors = numSensors;

    this->pcf8574Addresses.assign(pcf8574Addresses, pcf8574Addresses + numPCF);

    for (uint8_t i = 0; i < numPCF; i++) {
        this->pcf8574States.push_back(0xFF); // Inicialmente, todos os pinos são entradas

        std::vector<uint8_t> pins(xshutPins[i], xshutPins[i] + 8);
        this->xshutPins.push_back(pins);
    }

    sensors.resize(numSensors);
}

bool LidarArray::initSensors() {
    for (uint8_t i = 0; i < numPCF; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            pcf8574Write(i, xshutPins[i][j], 0);
        }
    }
    delay(100);
    for (uint8_t i = 0; i < numPCF; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            pcf8574Write(i, xshutPins[i][j], 1);
            delay(10);
            sensors[i * 8 + j].setAddress(0x30 + i * 8 + j);
        }
        for (uint8_t j = 0; j < 8; j++) {
            if (!sensors[i * 8 + j].init()) {
                Serial.print("Falha ao inicializar o sensor ");
                Serial.println(i * 8 + j + 1);
            } else {
                sensors[i * 8 + j].setMeasurementTimingBudget(LidarMeasurementTimingBudget);
                sensors[i * 8 + j].startContinuous();
                Serial.print("Iniciou o sensor ");
                Serial.println(i * 8 + j + 1);
            }
        }
    }
    return true;
}

bool LidarArray::initSensors(int measurementT) {
    LidarMeasurementTimingBudget = measurementT;
    return initSensors();
}

bool LidarArray::initSensors(int measurementT, uint8_t preRange, uint8_t FinalRange) {
    LidarMeasurementTimingBudget = measurementT;
    LidarPulsePeriodPreRange = preRange;
    LidarPulsePeriodFinalRange = FinalRange;
    return initSensors();
}

bool LidarArray::initSensors(int measurementT, uint8_t preRange, uint8_t FinalRange, int timeout) {
    LidarMeasurementTimingBudget = measurementT;
    LidarPulsePeriodPreRange = preRange;
    LidarPulsePeriodFinalRange = FinalRange;
    LidarTimeout = timeout;
    return initSensors();
}

uint16_t LidarArray::readSensorNB(uint8_t sensorIndex) {
    if (sensorIndex < sensors.size()) {
        uint16_t dist = readSensorNonBlocking(sensors[sensorIndex]);
        return dist != 0 ? constrain(dist, 10, 700) : 700;
    }
    return 0;
}

uint16_t LidarArray::readSensor(uint8_t sensorIndex) 
{
    if (sensorIndex < sensors.size()) 
    {
        uint16_t dist = sensors[sensorIndex].readRangeContinuousMillimeters();
        return dist != 0 ? constrain(dist, 10, 700) : 700;
    }
    return 0;
}

void LidarArray::setMeasurementTimingBudget(uint32_t timingBudget) {
    for (auto &sensor : sensors) {
        sensor.setMeasurementTimingBudget(timingBudget);
    }
}

void LidarArray::setVcselPulsePeriod(uint8_t type, uint8_t period) {
    for (auto &sensor : sensors) {
        if (type == 0) {
            sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, period);
        } else if (type == 1) {
            sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, period);
        }
    }
}

uint16_t LidarArray::readSensorNonBlocking(VL53L0X &sensor) {
    if ((sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        return 0;
    }

    uint16_t range = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range;
}

void LidarArray::pcf8574Write(uint8_t pcf8574Index, uint8_t pin, bool state) {
    if (state) {
        pcf8574States[pcf8574Index] |= (1 << pin);
    } else {
        pcf8574States[pcf8574Index] &= ~(1 << pin);
    }
    Wire.beginTransmission(pcf8574Addresses[pcf8574Index]);
    Wire.write(pcf8574States[pcf8574Index]);
    Wire.endTransmission();
}

VL53L0X& LidarArray::getSensor(uint8_t sensorIndex) {
    return sensors[sensorIndex];
}

uint8_t LidarArray::getSensorCount() {
    return sensors.size();
}