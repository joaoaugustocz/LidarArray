#include <lidarArray.h>

uint8_t pcf8574Addresses[] = {0x20, 0x21}; // Endereços I2C dos PCF8574
uint8_t xshutPins[2][8] = {
    {0, 1, 2, 3, 4, 5, 6, 8}, // Pinos XSHUT no PCF8574 no endereço 0x20
    {0, 1, 2, 3, 4, 5, 6, 8} // Pinos XSHUT no PCF8574 no endereço 0x21
};

LidarArray lidar(2, 16, pcf8574Addresses, xshutPins);

void setup() 
{
    Serial.begin(9600);
    Wire.begin();
    lidar.initSensors();
}

void loop() 
{
    for (uint8_t i = 0; i < lidar.getSensorCount(); i++) 
    {
        //uint16_t distancia = lidar.readSensorNB(i);//Lê o sensor com a função No Breaking, criada para evitar travaemntos 
        uint16_t distancia = lidar.readSensor(i);//Lê o sensor com a função Convencional. 

        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(distancia);
        Serial.print(" mm\t");
    }
    Serial.println();
    delay(100);
}