# LidarArray

## English

`LidarArray` is an Arduino library for managing `VL53L0X` or `VL53L4CD` ToF sensor arrays through `PCF8574` expanders that control the `XSHUT` lines.

Current codebase includes:

- support for `VL53L0X` and `VL53L4CD`
- mixed-model sparse maps in a single `LidarArray`
- preserved legacy dense `VL53L0X` flow
- sparse physical mapping through `LidarSensorSlot`
- public logical sensor IDs and ID-based reads
- automatic or manual final I2C addressing
- optional internal filtered reads with median, EMA, hold-last-valid, and jump rejection
- native I2C scan and step-by-step initialization debug
- configurable `TwoWire` bus selection
- English hover documentation in `LidarArray.h`

### Dependencies

- Pololu `VL53L0X`
- Pololu `VL53L4CD`

Your sketch must still call `Wire.begin()` or `Wire.begin(SDA, SCL)` before using the library.

### Recommended Include

```cpp
#include <LidarArray.h>
```

The root wrapper `lidarArray.h` is still available for backward compatibility.

### Layout Modes

`LidarArray` supports two public layout styles:

- legacy dense layout with `xshutPins`
- sparse layout with `LidarSensorSlot`

Use the dense layout when sensors follow the classic sequential mapping by PCF block. Use the sparse layout when sensors are spread across arbitrary `PCF8574` pins and you want explicit physical mapping.
All shipped examples now use the sparse layout and the short `LIDAR_SLOT(...)` syntax. The dense layout remains supported for backward compatibility.

### LidarSensorSlot Cheat Sheet

The sparse mapping API uses the following slot order:

```cpp
{pcfIndex, pin, address, sensorId, model}
```

Example:

```cpp
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p4, TOF_VL53L4CD, 1),
    LIDAR_SLOT(1, pcf_p5, TOF_VL53L0X, 12),
    LIDAR_SLOT_ADDR(1, pcf_p7, 0x36, TOF_VL53L0X, 14),
    {1, pcf_p6, 0, 11} // legacy 4-field form still works and inherits config.model
};
```

Field meaning:

- `pcfIndex`: which `PCF8574` controls the sensor `XSHUT`
- `pin`: physical `PCF8574` pin, always `0..7`
- `address`: final sensor address, `0` for automatic assignment
- `sensorId`: public logical ID, `-1` to reuse the internal index
- `model`: slot model, or `TOF_INHERIT_DEFAULT` to reuse `config.model`

Short syntax tokens:

- `pcf_p0..pcf_p7`: short `PCF8574` pin names
- `TOF_VL53L0X` and `TOF_VL53L4CD`: short model aliases
- `LIDAR_SLOT(...)`: helper macro that omits the common automatic address `0`

### Internal Index vs Public ID vs Physical Position

The library separates three concepts:

- internal index: always `0..N-1`
- public `sensorId`: optional user-facing label such as `10`, `11`, `12`
- physical position: the pair `pcfIndex + pin`

This means you can keep a compact internal array while exposing sensor names that match your robot geometry or application naming.

### Addressing Rules

Default behavior:

- sensor boot address: `0x29`
- final address: `0x30 + internal index`

Manual addressing is also supported:

- with dense legacy layout, use `setSensorAddresses(...)` or `config.sensorAddresses`
- with sparse layout, define the final address in each `LidarSensorSlot`

Rules:

- every final address must be unique
- final addresses must not collide with any `PCF8574`
- `0x29` must not be used as a final address
- do not combine `sensorMap` with `setSensorAddresses()`

### Quick Start

#### Legacy VL53L0X

```cpp
#include <LidarArray.h>

uint8_t pcf8574Addresses[] = {0x20};
uint8_t xshutPins[1][8] = {
    {0, 1, 2, 3, 4, 5, 6, 7}
};

LidarArray lidar(1, 4, pcf8574Addresses, xshutPins);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    lidar.initSensors(20000, 14, 10, 100);
}

void loop() {
    for (uint8_t i = 0; i < lidar.getSensorCount(); ++i) {
        Serial.println(lidar.readSensor(i));
    }
}
```

Use this path when you want the original dense `VL53L0X` flow and legacy `readSensor()` behavior.
The shipped examples no longer use this style, but it remains fully supported.

#### Recommended Current Setup

```cpp
#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20};
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p0, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p1, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p2, TOF_VL53L4CD, 2),
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 3)
};

LidarArray lidar(TOF_VL53L4CD);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(1, 4, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setVL53L4CDTiming(50, 0);
    lidar.begin();
}
```

Minimum required information:

- sensor model
- number of `PCF8574` expanders
- number of active sensors
- `PCF8574` addresses
- `sensorMap` for the recommended path, or `xshutPins` for the legacy dense path

Everything else can stay at the defaults.

#### Sparse Layout with Mixed Models and Public IDs

```cpp
#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20, 0x21};
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p4, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p7, TOF_VL53L4CD, 2),
    LIDAR_SLOT(1, pcf_p0, TOF_VL53L0X, 10),
    LIDAR_SLOT(1, pcf_p2, TOF_VL53L0X, 11),
    LIDAR_SLOT(1, pcf_p4, TOF_VL53L0X, 12),
    LIDAR_SLOT(1, pcf_p5, TOF_VL53L0X, 13),
    LIDAR_SLOT(1, pcf_p7, TOF_VL53L0X, 14)
};

LidarArray lidar(LidarSensorModel::VL53L4CD); // default model for 4-field slots

void setup() {
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(2, 8, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setMeasurementTimingBudget(20000);
    lidar.setVL53L4CDTiming(50, 0);
    lidar.begin();
}

void loop() {
    LidarReading reading = lidar.readReadingById(10, true);
    Serial.println(reading.distanceMm);
}
```

This is the recommended API for all new sketches, especially non-dense layouts, mixed sensor models, and applications that need public sensor IDs.

### Initialization Debug

The library can trace the full initialization sequence through any `Print`, including `Serial`.

```cpp
LidarArrayDebugConfig debugConfig =
    LidarArrayDebugConfig::verbose(&Serial, true, true, true, 500, 1500);

lidar.setDebugConfig(debugConfig);
lidar.begin();
```

Main debug options:

- `scanBeforeInit`
- `scanEachStep`
- `bootDelayMs`
- `animationDelayMs`

Expected sequence:

1. scan the bus before the library changes sensor state
2. shut down all ToF devices
3. bring sensors back one by one
4. assign final addresses and keep scanning if enabled

### Header Hover Docs

The public header `LidarArray.h` includes English Doxygen-style comments for the main public types, fields, and methods. In Arduino IDE and PlatformIO, these comments should appear as hover help for the current API.

### Optional Internal Filters

The raw reading path stays unchanged:

- `readSensor()`
- `readSensorNB()`
- `readReading()`
- `readReadingById()`

Filtered reads are opt-in and live in a separate API:

```cpp
LidarFilterConfig filterConfig = LidarFilterConfig::recommended();
filterConfig.maxJumpMm = 80;

lidar.setFilterConfig(filterConfig);

LidarFilteredReading filtered = lidar.readFilteredReading(0, true);
Serial.print("raw=");
Serial.print(filtered.raw.distanceMm);
Serial.print(" filtered=");
Serial.println(filtered.distanceMm);
```

Main filter features:

- median window `1`, `3`, or `5`
- EMA smoothing through `emaAlphaPercent`
- hold-last-valid for invalid reads
- jump rejection through `maxJumpMm`
- bounded jump rejection through `maxJumpRejections`
- optional per-sensor overrides with `setSensorFilterConfig(...)`

Recommended starting point for a moving robot:

- `LidarFilterConfig::recommended()`
- `medianWindow = 3`
- `emaAlphaPercent = 50`
- `holdLastValid = true`
- `maxHeldReads = 2`
- `maxJumpRejections = 2`

When jump rejection is active, `maxJumpRejections` bounds how many consecutive samples the
rule may reject. Once that budget is spent, the filter treats the new distance as real,
re-acquires it, and reports `jumpResynced = true` on that reading. This is what keeps a real
distance change, such as a robot turning to face a further wall, from silently disabling a
sensor. A value of `0` is coerced to `1` while `maxJumpMm > 0`, so the filter can never stay
locked on a stale value. Keep `maxJumpRejections` less than or equal to `maxHeldReads` to
avoid invalid outputs during the transition.

Filtered APIs:

- `readFilteredSensor(index)`
- `readFilteredSensorNB(index)`
- `readFilteredSensorById(sensorId)`
- `readFilteredSensorNBById(sensorId)`
- `readFilteredReading(index, blocking)`
- `readFilteredReadingById(sensorId, blocking)`

### Shipped Examples

- `examples/basic`: smallest recommended `sensorMap` setup for one `PCF8574`
- `examples/usingTwoPCF`: sparse slot mapping across two `PCF8574` expanders
- `examples/vl53l4cdConfig`: minimal `VL53L4CD` configuration with optional manual slot addresses
- `examples/logicalRemap`: reorder logical indices by reordering the slot array
- `examples/guidedDebug`: walk through boot, scans, and debug timing with sparse slots
- `examples/addressOverview`: inspect automatic and manual final addresses per slot
- `examples/sparseSensorMap`: sparse mapping with public IDs and 4-field compatibility
- `examples/mixedSensorMap`: mix `VL53L0X` and `VL53L4CD` in one array

### Notes and Common Pitfalls

- All shipped examples use sparse `sensorMap` layouts. The dense `xshutPins` path remains supported, but it is now a compatibility path rather than the recommended style.
- One `LidarArray` instance can mix `VL53L0X` and `VL53L4CD` when `sensorMap` defines the model per slot.
- Sensors above `numSensors` remain in shutdown.
- `config.model` is now the array default model used by legacy layouts and by 4-field sparse slots.
- `getSensor()` is only for `VL53L0X` compatibility and only makes sense for `VL53L0X` slots.
- `readSensor()` and `readSensorNB()` may apply legacy fallback and clamp behavior when legacy mode is active.
- Filtered reads are optional and do not change the raw behavior of `readSensor()`, `readSensorNB()`, or `readReading()`.
- `timeoutMs` defaults to `100` for both models. A value of `0` would make the underlying driver block forever on an unresponsive sensor, so the library replaces `0` with a timeout derived from the configured measurement period, bounded to `100..1000` ms.
- Seeing `0x29` before initialization is normal if a ToF sensor is already awake.
- `status = 254` indicates a library-side not-ready or unavailable condition.
- Partial initialization does not prevent ready sensors from being used.

### Public API Reference

#### Public Types

- `LidarSensorModel`: selects the array default model or a slot model override
- `LidarPcfPin`: short `PCF8574` pin tokens for sparse maps
- `LidarDebugLevel`: selects the debug verbosity
- `LidarSensorSlot`: sparse physical slot description
- `LidarReading`: structured measurement result
- `LidarFilterConfig`: filter pipeline configuration
- `LidarFilteredReading`: filtered result with raw and final output values
- `LidarArrayConfig`: full array configuration
- `LidarArrayDebugConfig`: initialization and scan debug configuration

#### Config Builders

- `LidarArrayConfig::defaults(model)`
- `LidarArrayConfig::forModel(..., xshutPins, wire, sensorAddresses)`
- `LidarArrayConfig::forModel(..., sensorMap, wire)`
- `LidarArrayConfig::forVL53L0X(..., xshutPins, wire, sensorAddresses)`
- `LidarArrayConfig::forVL53L0X(..., sensorMap, wire)`
- `LidarArrayConfig::forVL53L4CD(..., xshutPins, wire, sensorAddresses)`
- `LidarArrayConfig::forVL53L4CD(..., sensorMap, wire)`
- `LidarArrayDebugConfig::verbose(...)`
- `LidarFilterConfig::disabled()`
- `LidarFilterConfig::recommended()`

#### Constructors

- `LidarArray(LidarSensorModel model = LidarSensorModel::VL53L0X)`
- `LidarArray(const LidarArrayConfig &config)`
- `LidarArray(uint8_t numPCF, uint8_t numSensors, const uint8_t pcf8574Addresses[], const uint8_t xshutPins[][8])`

#### Initialization

- `begin()`
- `initSensors()`
- `initSensors(int measurementT)`
- `initSensors(int measurementT, uint8_t preRange, uint8_t finalRange)`
- `initSensors(int measurementT, uint8_t preRange, uint8_t finalRange, int timeout)`

#### Configuration Access

- `config()`
- `config() const`
- `debug()`
- `debug() const`
- `filter()`
- `filter() const`

#### Configuration Setters

- `setLayout(..., xshutPins)`
- `setLayout(..., sensorMap)`
- `setSensorAddresses(...)`
- `setWire(...)`
- `setTimeout(...)`
- `setVL53L4CDTiming(...)`
- `setFilterConfig(...)`
- `setSensorFilterConfig(...)`
- `clearSensorFilterConfig(...)`
- `clearSensorFilterConfigs()`
- `hasSensorFilterConfig(index)`
- `getEffectiveFilterConfig(index)`
- `resetFilter(index)`
- `resetFilters()`
- `setMeasurementTimingBudget(...)`
- `setVcselPulsePeriod(...)`

#### Reading API

- `readSensor(index)`
- `readSensorNB(index)`
- `readSensorById(sensorId)`
- `readSensorNBById(sensorId)`
- `readReading(index, blocking)`
- `readReadingById(sensorId, blocking)`
- `readFilteredSensor(index)`
- `readFilteredSensorNB(index)`
- `readFilteredSensorById(sensorId)`
- `readFilteredSensorNBById(sensorId)`
- `readFilteredReading(index, blocking)`
- `readFilteredReadingById(sensorId, blocking)`

#### Direct Driver Access

- `getSensor(index)`
- `getVL53L0XSensor(index)`
- `getVL53L4CDSensor(index)`

#### Debug API

- `setDebugConfig(...)`
- `setDebugOutput(...)`
- `setDebugLevel(...)`
- `setDebugScanBeforeInit(...)`
- `setDebugScanEachStep(...)`
- `setDebugBootDelay(...)`
- `setDebugStepDelay(...)`
- `scanI2C()`

#### Diagnostics

- `getSensorCount()`
- `getInitializedSensorCount()`
- `isSensorReady(index)`
- `getSensorId(index)`
- `getSensorModel(index)`
- `getSensorModelById(sensorId)`
- `indexOfSensorId(sensorId)`
- `getSensorSlot(index)`

---

## PT-BR

`LidarArray` e uma biblioteca Arduino para gerenciar arrays de sensores ToF `VL53L0X` ou `VL53L4CD` atraves de expansores `PCF8574` que controlam os pinos `XSHUT`.

O codigo atual inclui:

- suporte a `VL53L0X` e `VL53L4CD`
- mapeamento esparso misto com os dois modelos na mesma instancia
- preservacao do fluxo legado denso para `VL53L0X`
- mapeamento fisico esparso com `LidarSensorSlot`
- IDs logicos publicos e leituras por ID
- enderecamento final automatico ou manual
- leituras filtradas opcionais com mediana, EMA, hold-last-valid e rejeicao de salto
- scan I2C nativo e debug passo a passo da inicializacao
- selecao configuravel de `TwoWire`
- documentacao de hover em ingles dentro de `LidarArray.h`

### Dependencias

- Pololu `VL53L0X`
- Pololu `VL53L4CD`

O sketch continua responsavel por chamar `Wire.begin()` ou `Wire.begin(SDA, SCL)` antes de usar a biblioteca.

### Include Recomendado

```cpp
#include <LidarArray.h>
```

O wrapper raiz `lidarArray.h` continua disponivel por compatibilidade.

### Modos de Layout

`LidarArray` suporta dois estilos publicos de layout:

- layout denso legado com `xshutPins`
- layout esparso com `LidarSensorSlot`

Use o layout denso quando os sensores seguem o mapeamento sequencial classico por bloco de PCF. Use o layout esparso quando os sensores estiverem distribuidos em pinos arbitrarios de um ou mais `PCF8574` e voce quiser declarar o mapeamento fisico explicitamente.
Todos os exemplos publicos agora usam o layout esparso com a sintaxe curta `LIDAR_SLOT(...)`. O layout denso continua suportado apenas como compatibilidade.

### Cola do LidarSensorSlot

A API de mapeamento esparso usa a seguinte ordem de campos:

```cpp
{pcfIndex, pin, address, sensorId, model}
```

Exemplo:

```cpp
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p4, TOF_VL53L4CD, 1),
    LIDAR_SLOT(1, pcf_p5, TOF_VL53L0X, 12),
    LIDAR_SLOT_ADDR(1, pcf_p7, 0x36, TOF_VL53L0X, 14),
    {1, pcf_p6, 0, 11}
};
```

Significado dos campos:

- `pcfIndex`: qual `PCF8574` controla o `XSHUT`
- `pin`: pino fisico do `PCF8574`, sempre `0..7`
- `address`: endereco final do sensor, `0` para modo automatico
- `sensorId`: ID logico publico, `-1` para reaproveitar o indice interno
- `model`: modelo do slot, ou `TOF_INHERIT_DEFAULT` para herdar `config.model`

Atalhos recomendados:

- `pcf_p0..pcf_p7`: nomes curtos para os pinos do `PCF8574`
- `TOF_VL53L0X` e `TOF_VL53L4CD`: aliases curtos dos modelos
- `LIDAR_SLOT(...)`: macro para omitir o endereco automatico `0`

### Indice Interno vs sensorId vs Posicao Fisica

A biblioteca separa tres conceitos:

- indice interno: sempre `0..N-1`
- `sensorId` publico: rotulo opcional como `10`, `11`, `12`
- posicao fisica: o par `pcfIndex + pin`

Assim, a biblioteca pode manter arrays internos compactos enquanto voce expoe nomes de sensores que fazem sentido para o robo ou para a aplicacao.

### Regras de Enderecamento

Comportamento padrao:

- endereco de boot do sensor: `0x29`
- endereco final: `0x30 + indice interno`

Tambem existe enderecamento manual:

- no layout denso legado, use `setSensorAddresses(...)` ou `config.sensorAddresses`
- no layout esparso, defina o endereco final dentro de cada `LidarSensorSlot`

Regras:

- todo endereco final precisa ser unico
- enderecos finais nao podem colidir com nenhum `PCF8574`
- `0x29` nao pode ser usado como endereco final
- nao combine `sensorMap` com `setSensorAddresses()`

### Inicio Rapido

#### VL53L0X Legado

```cpp
#include <LidarArray.h>

uint8_t pcf8574Addresses[] = {0x20};
uint8_t xshutPins[1][8] = {
    {0, 1, 2, 3, 4, 5, 6, 7}
};

LidarArray lidar(1, 4, pcf8574Addresses, xshutPins);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    lidar.initSensors(20000, 14, 10, 100);
}

void loop() {
    for (uint8_t i = 0; i < lidar.getSensorCount(); ++i) {
        Serial.println(lidar.readSensor(i));
    }
}
```

Use esse caminho quando quiser o fluxo denso original de `VL53L0X` e o comportamento legado de `readSensor()`.
Os exemplos publicos nao usam mais esse estilo, mas ele continua totalmente suportado.

#### Configuracao Recomendada Atual

```cpp
#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20};
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p0, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p1, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p2, TOF_VL53L4CD, 2),
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 3)
};

LidarArray lidar(TOF_VL53L4CD);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(1, 4, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setVL53L4CDTiming(50, 0);
    lidar.begin();
}
```

Informacoes minimas necessarias:

- modelo do sensor
- quantidade de `PCF8574`
- quantidade de sensores ativos
- enderecos dos `PCF8574`
- `sensorMap` no caminho recomendado, ou `xshutPins` no caminho denso legado

Todo o resto pode ficar nos defaults.

#### Layout Esparso Misto com IDs Publicos

```cpp
#include <LidarArray.h>

const uint8_t pcf8574Addresses[] = {0x20, 0x21};
const LidarSensorSlot sensorMap[] = {
    LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 0),
    LIDAR_SLOT(0, pcf_p4, TOF_VL53L4CD, 1),
    LIDAR_SLOT(0, pcf_p7, TOF_VL53L4CD, 2),
    LIDAR_SLOT(1, pcf_p0, TOF_VL53L0X, 10),
    LIDAR_SLOT(1, pcf_p2, TOF_VL53L0X, 11),
    LIDAR_SLOT(1, pcf_p4, TOF_VL53L0X, 12),
    LIDAR_SLOT(1, pcf_p5, TOF_VL53L0X, 13),
    LIDAR_SLOT(1, pcf_p7, TOF_VL53L0X, 14)
};

LidarArray lidar(LidarSensorModel::VL53L4CD);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    lidar.setLayout(2, 8, pcf8574Addresses, sensorMap);
    lidar.setTimeout(100);
    lidar.setMeasurementTimingBudget(20000);
    lidar.setVL53L4CDTiming(50, 0);
    lidar.begin();
}

void loop() {
    LidarReading reading = lidar.readReadingById(10, true);
    Serial.println(reading.distanceMm);
}
```

Essa e a API recomendada para todos os sketches novos, especialmente layouts nao densos, mistura de modelos e aplicacoes que precisam de IDs publicos.

### Debug da Inicializacao

A biblioteca consegue mostrar a sequencia completa de inicializacao em qualquer `Print`, incluindo `Serial`.

```cpp
LidarArrayDebugConfig debugConfig =
    LidarArrayDebugConfig::verbose(&Serial, true, true, true, 500, 1500);

lidar.setDebugConfig(debugConfig);
lidar.begin();
```

Principais opcoes de debug:

- `scanBeforeInit`
- `scanEachStep`
- `bootDelayMs`
- `animationDelayMs`

Sequencia esperada:

1. scan do barramento antes da biblioteca mudar o estado dos sensores
2. shutdown de todos os ToF
3. retorno dos sensores um por vez
4. atribuicao dos enderecos finais e novos scans, se habilitados

### Documentacao de Hover no Header

O header publico `LidarArray.h` inclui comentarios em ingles no estilo Doxygen para os principais tipos, campos e metodos publicos. No Arduino IDE e no PlatformIO, esses comentarios devem aparecer como ajuda de hover para a API atual.

### Filtros Internos Opcionais

O caminho cru continua igual:

- `readSensor()`
- `readSensorNB()`
- `readReading()`
- `readReadingById()`

As leituras filtradas sao opcionais e vivem em uma API separada:

```cpp
LidarFilterConfig filterConfig = LidarFilterConfig::recommended();
filterConfig.maxJumpMm = 80;

lidar.setFilterConfig(filterConfig);

LidarFilteredReading filtered = lidar.readFilteredReading(0, true);
Serial.print("raw=");
Serial.print(filtered.raw.distanceMm);
Serial.print(" filtered=");
Serial.println(filtered.distanceMm);
```

Principais recursos do filtro:

- janela de mediana `1`, `3` ou `5`
- suavizacao EMA por `emaAlphaPercent`
- hold-last-valid para leituras invalidas
- rejeicao de salto com `maxJumpMm`
- rejeicao de salto limitada por `maxJumpRejections`
- override por sensor com `setSensorFilterConfig(...)`

Ponto de partida recomendado para um robo em movimento:

- `LidarFilterConfig::recommended()`
- `medianWindow = 3`
- `emaAlphaPercent = 50`
- `holdLastValid = true`
- `maxHeldReads = 2`
- `maxJumpRejections = 2`

Com a rejeicao de salto ativa, `maxJumpRejections` limita quantas amostras consecutivas a regra
pode rejeitar. Quando esse limite se esgota, o filtro assume que a nova distancia e real,
readquire esse nivel e marca `jumpResynced = true` naquela leitura. E isso que impede que uma
mudanca real de distancia, como o robo girando para uma parede mais distante, desative o sensor
em silencio. O valor `0` e convertido para `1` enquanto `maxJumpMm > 0`, entao o filtro nunca
fica preso em um valor antigo. Mantenha `maxJumpRejections` menor ou igual a `maxHeldReads` para
evitar saidas invalidas durante a transicao.

APIs filtradas:

- `readFilteredSensor(index)`
- `readFilteredSensorNB(index)`
- `readFilteredSensorById(sensorId)`
- `readFilteredSensorNBById(sensorId)`
- `readFilteredReading(index, blocking)`
- `readFilteredReadingById(sensorId, blocking)`

### Exemplos Publicos

- `examples/basic`: menor setup recomendado com `sensorMap` e um `PCF8574`
- `examples/usingTwoPCF`: mapeamento esparso em dois expansores `PCF8574`
- `examples/vl53l4cdConfig`: configuracao minima de `VL53L4CD` com enderecos opcionais por slot
- `examples/logicalRemap`: remapeio da ordem logica reordenando o array de slots
- `examples/guidedDebug`: boot guiado com scans e tempos de debug em slots esparsos
- `examples/addressOverview`: inspecao de enderecos finais automaticos e manuais por slot
- `examples/sparseSensorMap`: mapeamento esparso com IDs publicos e compatibilidade de 4 campos
- `examples/mixedSensorMap`: mistura `VL53L0X` e `VL53L4CD` no mesmo array

### Observacoes e Armadilhas Comuns

- Todos os exemplos publicos usam `sensorMap`. O caminho denso com `xshutPins` continua suportado, mas agora fica como trilha de compatibilidade, nao como estilo recomendado.
- Uma instancia de `LidarArray` pode misturar `VL53L0X` e `VL53L4CD` quando o `sensorMap` define o modelo por slot.
- Sensores acima de `numSensors` permanecem em shutdown.
- `config.model` agora e o modelo padrao do array para layouts legados e slots esparsos com 4 campos.
- `getSensor()` existe apenas para compatibilidade com `VL53L0X` e so faz sentido em slots `VL53L0X`.
- `readSensor()` e `readSensorNB()` podem aplicar fallback e clamp quando o modo legado estiver ativo.
- As leituras filtradas sao opcionais e nao alteram o comportamento cru de `readSensor()`, `readSensorNB()` ou `readReading()`.
- `timeoutMs` agora vale `100` por padrao nos dois modelos. O valor `0` faria o driver bloquear para sempre em um sensor que nao responde, entao a biblioteca troca `0` por um timeout derivado do periodo de medicao configurado, limitado a `100..1000` ms.
- Ver `0x29` antes da inicializacao e normal se um ToF ja estiver acordado.
- `status = 254` indica um estado interno de nao pronto ou indisponivel.
- Inicializacao parcial nao impede o uso dos sensores que ficaram prontos.

### Referencia da API Publica

#### Tipos Publicos

- `LidarSensorModel`: seleciona o modelo padrao do array ou o override de um slot
- `LidarPcfPin`: tokens curtos para pinos do `PCF8574`
- `LidarDebugLevel`: seleciona a verbosidade do debug
- `LidarSensorSlot`: descricao de um slot fisico esparso
- `LidarReading`: resultado estruturado de leitura
- `LidarFilterConfig`: configuracao do pipeline de filtros
- `LidarFilteredReading`: resultado filtrado com valores bruto e final
- `LidarArrayConfig`: configuracao completa do array
- `LidarArrayDebugConfig`: configuracao de debug da inicializacao e dos scans

#### Builders de Configuracao

- `LidarArrayConfig::defaults(model)`
- `LidarArrayConfig::forModel(..., xshutPins, wire, sensorAddresses)`
- `LidarArrayConfig::forModel(..., sensorMap, wire)`
- `LidarArrayConfig::forVL53L0X(..., xshutPins, wire, sensorAddresses)`
- `LidarArrayConfig::forVL53L0X(..., sensorMap, wire)`
- `LidarArrayConfig::forVL53L4CD(..., xshutPins, wire, sensorAddresses)`
- `LidarArrayConfig::forVL53L4CD(..., sensorMap, wire)`
- `LidarArrayDebugConfig::verbose(...)`
- `LidarFilterConfig::disabled()`
- `LidarFilterConfig::recommended()`

#### Construtores

- `LidarArray(LidarSensorModel model = LidarSensorModel::VL53L0X)`
- `LidarArray(const LidarArrayConfig &config)`
- `LidarArray(uint8_t numPCF, uint8_t numSensors, const uint8_t pcf8574Addresses[], const uint8_t xshutPins[][8])`

#### Inicializacao

- `begin()`
- `initSensors()`
- `initSensors(int measurementT)`
- `initSensors(int measurementT, uint8_t preRange, uint8_t finalRange)`
- `initSensors(int measurementT, uint8_t preRange, uint8_t finalRange, int timeout)`

#### Acesso a Configuracao

- `config()`
- `config() const`
- `debug()`
- `debug() const`
- `filter()`
- `filter() const`

#### Setters de Configuracao

- `setLayout(..., xshutPins)`
- `setLayout(..., sensorMap)`
- `setSensorAddresses(...)`
- `setWire(...)`
- `setTimeout(...)`
- `setVL53L4CDTiming(...)`
- `setFilterConfig(...)`
- `setSensorFilterConfig(...)`
- `clearSensorFilterConfig(...)`
- `clearSensorFilterConfigs()`
- `hasSensorFilterConfig(index)`
- `getEffectiveFilterConfig(index)`
- `resetFilter(index)`
- `resetFilters()`
- `setMeasurementTimingBudget(...)`
- `setVcselPulsePeriod(...)`

#### API de Leitura

- `readSensor(index)`
- `readSensorNB(index)`
- `readSensorById(sensorId)`
- `readSensorNBById(sensorId)`
- `readReading(index, blocking)`
- `readReadingById(sensorId, blocking)`
- `readFilteredSensor(index)`
- `readFilteredSensorNB(index)`
- `readFilteredSensorById(sensorId)`
- `readFilteredSensorNBById(sensorId)`
- `readFilteredReading(index, blocking)`
- `readFilteredReadingById(sensorId, blocking)`

#### Acesso Direto aos Drivers

- `getSensor(index)`
- `getVL53L0XSensor(index)`
- `getVL53L4CDSensor(index)`

#### API de Debug

- `setDebugConfig(...)`
- `setDebugOutput(...)`
- `setDebugLevel(...)`
- `setDebugScanBeforeInit(...)`
- `setDebugScanEachStep(...)`
- `setDebugBootDelay(...)`
- `setDebugStepDelay(...)`
- `scanI2C()`

#### Diagnostico

- `getSensorCount()`
- `getInitializedSensorCount()`
- `isSensorReady(index)`
- `getSensorId(index)`
- `getSensorModel(index)`
- `getSensorModelById(sensorId)`
- `indexOfSensorId(sensorId)`
- `getSensorSlot(index)`

## License

MIT.
