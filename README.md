# LidarArray Library Documentation
## Overview
LidarArray is an Arduino library designed to manage multiple VL53L0X LiDAR sensors using PCF8574 I/O expanders. This library simplifies the integration and control of LiDAR sensors in Arduino projects, offering flexible configuration options and easy data acquisition.

## Installation
To use the LidarArray library in your Arduino projects:

1. Download the LidarArray library from the [GitHub repository](https://github.com/joaoaugustocz/LidarArray) as a ZIP file.
2. Install the library in the Arduino IDE by navigating to Sketch -> Include Library -> Add .ZIP Library... and selecting the downloaded ZIP file.
3. Open the LidarArray example sketches from File -> Examples -> LidarArray to get started with using the library.

## Class LidarArray
### Constructor

`LidarArray(uint8_t numPCF, uint8_t numSensors, const uint8_t pcf8574Addresses[], const uint8_t xshutPins[][8]);`

### Parameters:

- **numPCF**: Number of PCF8574 I/O expanders used.

- **numSensors**: Total number of VL53L0X sensors.

- **pcf8574Addresses[]**: Array of I2C addresses for PCF8574 devices.

- **xshutPins[][8]**: 2D array of XSHUT pin configurations for each PCF8574 device.


### Methods

`initSensors()`
Initializes all sensors with default measurement timing.

`initSensors(int measurementT)`
Initializes all sensors with a custom measurement timing budget.

`initSensors(int measurementT, uint8_t preRange, uint8_t finalRange)`
Initializes all sensors with custom timing and Vcsel pulse periods.

`initSensors(int measurementT, uint8_t preRange, uint8_t finalRange, int timeout)`
Initializes sensors with custom timing, Vcsel pulse periods, and timeout.

`readSensor(uint8_t sensorIndex)`
Reads distance from the specified sensor index in millimeters.

`readSensorNB(uint8_t sensorIndex)`
Non-blocking read of distance from the specified sensor index.

`getSensor(uint8_t sensorIndex)`
Returns a reference to the VL53L0X sensor object at the specified index.

`setMeasurementTimingBudget(uint32_t timingBudget)`
Sets the measurement timing budget for all sensors.

`setVcselPulsePeriod(uint8_t type, uint8_t period)`
Sets the Vcsel pulse period for all sensors.

`getSensorCount()`
Returns the total number of sensors initialized.

`pcf8574Write(uint8_t pcf8574Index, uint8_t pin, bool state)`
Writes a state (HIGH or LOW) to a specific pin on the PCF8574 device.

`readSensorNonBlocking(VL53L0X &sensor)`
Non-blocking read of distance from a VL53L0X sensor object.



## Class Vector
### Methods

`Vector()`
Default constructor that creates an empty vector.

`Vector(size_t size)`
Constructor that creates a vector with the specified size.

`Vector(size_t size, const T &value)`
Constructor that creates a vector with the specified size and initializes all elements with the specified value.

`~Vector()`
Destructor that frees the allocated memory for the vector.

`size_t size() const`
Returns the number of elements in the vector.

`bool empty() const`
Checks if the vector is empty.

`void resize(size_t newSize)`
Changes the size of the vector to the new specified size.

`void reserve(size_t newCapacity)`
Reserves space for at least a certain number of elements without modifying the size of the vector.

`void clear()`
Removes all elements from the vector and frees the allocated memory.

`T &operator[](size_t index)`
Returns a reference to the element at the specified index.

`const T &operator[](size_t index) const`
Returns a constant reference to the element at the specified index.

`void push_back(const T &value)`
Adds an element to the end of the vector.

`void pop_back()`
Removes the last element from the vector.

`T &front()`
Returns a reference to the first element of the vector.

`const T &front() const`
Returns a constant reference to the first element of the vector.

`T &back()`
Returns a reference to the last element of the vector.

`const T &back() const`
Returns a constant reference to the last element of the vector.

`T *data()`
Returns a pointer to the elements of the vector.

`const T *data() const`
Returns a constant pointer to the elements of the vector.

`iterator begin() noexcept`
Returns an iterator to the first element of the vector.

`const_iterator begin() const noexcept`
Returns a constant iterator to the first element of the vector.

`iterator end() noexcept`
Returns an iterator to the element past the last element of the vector.

`const_iterator end() const noexcept`
Returns a constant iterator to the element past the last element of the vector.



## Example Usage

```cpp
#include <LidarArray.h>

uint8_t pcf8574Addresses[] = {0x20}; // I2C addresses of the PCF8574
uint8_t xshutPins[1][8] = {
    {0, 1, 2, 3, 4, 5, 6, 8} // XSHUT pins on PCF8574 at address 0x20
};

LidarArray lidar(1, 8, pcf8574Addresses, xshutPins);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(1000);
    lidar.initSensors();
}

void loop() {
    for (uint8_t i = 0; i < lidar.getSensorCount(); i++) {
        uint16_t distance = lidar.readSensor(i);
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(distance);
        Serial.println(" mm");
    }
    delay(100);
}
```
## License
This library is released under the MIT License. See the LICENSE file for more details.



# Documentação da Biblioteca LidarArray
## Visão Geral
LidarArray é uma biblioteca para Arduino projetada para gerenciar múltiplos sensores LiDAR VL53L0X utilizando expansores de I/O PCF8574. Esta biblioteca simplifica a integração e controle de sensores LiDAR em projetos Arduino, oferecendo opções de configuração flexíveis e aquisição de dados simplificada.

## Instalação
Para utilizar a biblioteca LidarArray nos seus projetos Arduino:

1. Baixe a biblioteca LidarArray do [repositório GitHub](https://github.com/joaoaugustocz/LidarArray) como um arquivo ZIP.
2. Instale a biblioteca no Arduino IDE navegando em Sketch -> Incluir Biblioteca -> Adicionar Biblioteca .ZIP... e selecione o arquivo ZIP baixado.
3. Abra os exemplos da LidarArray em Arquivo -> Exemplos -> LidarArray para começar a utilizar a biblioteca.

## Classe LidarArray
### Construtor

`LidarArray(uint8_t numPCF, uint8_t numSensors, const uint8_t pcf8574Addresses[], const uint8_t xshutPins[][8]);`

### Parâmetros:

- **numPCF**: Número de expansores de I/O PCF8574 utilizados.

- **numSensors**: Número total de sensores VL53L0X.

- **pcf8574Addresses[]**: Array de endereços I2C para os dispositivos PCF8574.

- **xshutPins[][8]**: Array 2D de configurações de pinos XSHUT para cada dispositivo PCF8574.


### Métodos

`initSensors()`
Inicializa todos os sensores com o tempo de medição padrão.

`initSensors(int measurementT)`
Inicializa todos os sensores com um orçamento de tempo de medição personalizado.

`initSensors(int measurementT, uint8_t preRange, uint8_t finalRange)`
Inicializa todos os sensores com períodos de pulso Vcsel e tempo personalizados.

`initSensors(int measurementT, uint8_t preRange, uint8_t finalRange, int timeout)`
Inicializa sensores com tempo, períodos de pulso Vcsel e timeout personalizados.

`readSensor(uint8_t sensorIndex)`
Lê a distância do sensor especificado pelo índice em milímetros.

`readSensorNB(uint8_t sensorIndex)`
Leitura não bloqueante da distância do sensor especificado.

`getSensor(uint8_t sensorIndex)`
Retorna uma referência ao objeto do sensor VL53L0X no índice especificado.

`setMeasurementTimingBudget(uint32_t timingBudget)`
Define o orçamento de tempo de medição para todos os sensores.

`setVcselPulsePeriod(uint8_t type, uint8_t period)`
Define o período de pulso Vcsel para todos os sensores.

`getSensorCount()`
Retorna o número total de sensores inicializados.

`pcf8574Write(uint8_t pcf8574Index, uint8_t pin, bool state)`
Escreve um estado (HIGH ou LOW) para um pino específico no dispositivo PCF8574.

`readSensorNonBlocking(VL53L0X &sensor)`
Leitura não bloqueante da distância de um objeto sensor VL53L0X.


## Classe Vector
### Métodos

`Vector()`
Construtor padrão que cria um vetor vazio.

`Vector(size_t size)`
Construtor que cria um vetor com o tamanho especificado.

`Vector(size_t size, const T &value)`
Construtor que cria um vetor com o tamanho especificado e inicializa todos os elementos com o valor especificado.

`~Vector()`
Destrutor que libera a memória alocada para o vetor.

`size_t size() const`
Retorna o número de elementos no vetor.

`bool empty() const`
Verifica se o vetor está vazio.

`void resize(size_t newSize)`
Altera o tamanho do vetor para o novo tamanho especificado.

`void reserve(size_t newCapacity)`
Reserva espaço para pelo menos uma certa quantidade de elementos sem modificar o tamanho do vetor.

`void clear()`
Remove todos os elementos do vetor e libera a memória alocada.

`T &operator[](size_t index)`
Retorna uma referência ao elemento no índice especificado.

`const T &operator[](size_t index) const`
Retorna uma referência constante ao elemento no índice especificado.

`void push_back(const T &value)`
Adiciona um elemento ao final do vetor.

`void pop_back()`
Remove o último elemento do vetor.

`T &front()`
Retorna uma referência ao primeiro elemento do vetor.

`const T &front() const`
Retorna uma referência constante ao primeiro elemento do vetor.

`T &back()`
Retorna uma referência ao último elemento do vetor.

`const T &back() const`
Retorna uma referência constante ao último elemento do vetor.

`T *data()`
Retorna um ponteiro para os elementos do vetor.

`const T *data() const`
Retorna um ponteiro constante para os elementos do vetor.

`iterator begin() noexcept`
Retorna um iterador para o primeiro elemento do vetor.

`const_iterator begin() const noexcept`
Retorna um iterador constante para o primeiro elemento do vetor.

`iterator end() noexcept`
Retorna um iterador para o elemento após o último elemento do vetor.

`const_iterator end() const noexcept`
Retorna um iterador constante para o elemento após o último elemento do vetor.


## Exemplo de Uso

```cpp
#include <LidarArray.h>

uint8_t pcf8574Addresses[] = {0x20}; // Endereços I2C dos PCF8574
uint8_t xshutPins[1][8] = {
    {0, 1, 2, 3, 4, 5, 6, 8} // Pinos XSHUT no PCF8574 no endereço 0x20
};

LidarArray lidar(1, 8, pcf8574Addresses, xshutPins);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(1000);
    lidar.initSensors();
}

void loop() {
    for (uint8_t i = 0; i < lidar.getSensorCount(); i++) {
        uint16_t distancia = lidar.readSensor(i);
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(distancia);
        Serial.println(" mm");
    }
    delay(100);
}
```
## Licença
Esta biblioteca é lançada sob a Licença MIT. Consulte o arquivo LICENSE para mais detalhes.
