#include "LidarArray.h"

#include <new>

namespace
{
bool isConcreteModel(LidarSensorModel model)
{
    return model == LidarSensorModel::VL53L0X || model == LidarSensorModel::VL53L4CD;
}

LidarSensorModel resolveSlotModel(const LidarSensorSlot &slot, LidarSensorModel defaultModel)
{
    return slot.model == LidarSensorModel::InheritArrayDefault ? defaultModel : slot.model;
}

bool resolveModelCounts(const LidarArrayConfig &config, uint8_t &vl53l0xCount, uint8_t &vl53l4cdCount)
{
    vl53l0xCount = 0;
    vl53l4cdCount = 0;

    if (config.numSensors == 0)
    {
        return false;
    }

    if (config.sensorMap != nullptr)
    {
        for (uint8_t sensorIndex = 0; sensorIndex < config.numSensors; ++sensorIndex)
        {
            const LidarSensorModel model = resolveSlotModel(config.sensorMap[sensorIndex], config.model);
            if (model == LidarSensorModel::VL53L0X)
            {
                ++vl53l0xCount;
            }
            else if (model == LidarSensorModel::VL53L4CD)
            {
                ++vl53l4cdCount;
            }
            else
            {
                return false;
            }
        }

        return true;
    }

    if (config.xshutPins == nullptr || !isConcreteModel(config.model))
    {
        return false;
    }

    if (config.model == LidarSensorModel::VL53L0X)
    {
        vl53l0xCount = config.numSensors;
    }
    else
    {
        vl53l4cdCount = config.numSensors;
    }

    return true;
}

const __FlashStringHelper *sensorModelLabel(LidarSensorModel model)
{
    switch (model)
    {
    case LidarSensorModel::VL53L0X:
        return F("VL53L0X");
    case LidarSensorModel::VL53L4CD:
        return F("VL53L4CD");
    default:
        return F("inherit");
    }
}
} // namespace

LidarArrayConfig LidarArrayConfig::defaults(LidarSensorModel model)
{
    LidarArrayConfig config;
    config.model = model;

    if (model == LidarSensorModel::VL53L4CD)
    {
        config.timeoutMs = 100;
        config.vl53l4cdTimingBudgetMs = 50;
        config.vl53l4cdInterMeasurementMs = 0;
    }

    return config;
}

LidarArrayConfig LidarArrayConfig::forModel(
    LidarSensorModel model,
    uint8_t numPCF,
    uint8_t numSensors,
    const uint8_t *pcf8574Addresses,
    const uint8_t *xshutPins,
    TwoWire *wire,
    const uint8_t *sensorAddresses)
{
    LidarArrayConfig config = defaults(model);
    config.numPCF = numPCF;
    config.numSensors = numSensors;
    config.pcf8574Addresses = pcf8574Addresses;
    config.xshutPins = xshutPins;
    config.sensorAddresses = sensorAddresses;
    config.sensorMap = nullptr;
    config.wire = wire != nullptr ? wire : &Wire;
    return config;
}

LidarArrayConfig LidarArrayConfig::forModel(
    LidarSensorModel model,
    uint8_t numPCF,
    uint8_t numSensors,
    const uint8_t *pcf8574Addresses,
    const LidarSensorSlot *sensorMap,
    TwoWire *wire)
{
    LidarArrayConfig config = defaults(model);
    config.numPCF = numPCF;
    config.numSensors = numSensors;
    config.pcf8574Addresses = pcf8574Addresses;
    config.xshutPins = nullptr;
    config.sensorAddresses = nullptr;
    config.sensorMap = sensorMap;
    config.wire = wire != nullptr ? wire : &Wire;
    return config;
}

LidarArrayConfig LidarArrayConfig::forVL53L0X(
    uint8_t numPCF,
    uint8_t numSensors,
    const uint8_t *pcf8574Addresses,
    const uint8_t *xshutPins,
    TwoWire *wire,
    const uint8_t *sensorAddresses)
{
    return forModel(LidarSensorModel::VL53L0X, numPCF, numSensors, pcf8574Addresses, xshutPins, wire, sensorAddresses);
}

LidarArrayConfig LidarArrayConfig::forVL53L0X(
    uint8_t numPCF,
    uint8_t numSensors,
    const uint8_t *pcf8574Addresses,
    const LidarSensorSlot *sensorMap,
    TwoWire *wire)
{
    return forModel(LidarSensorModel::VL53L0X, numPCF, numSensors, pcf8574Addresses, sensorMap, wire);
}

LidarArrayConfig LidarArrayConfig::forVL53L4CD(
    uint8_t numPCF,
    uint8_t numSensors,
    const uint8_t *pcf8574Addresses,
    const uint8_t *xshutPins,
    TwoWire *wire,
    const uint8_t *sensorAddresses)
{
    return forModel(LidarSensorModel::VL53L4CD, numPCF, numSensors, pcf8574Addresses, xshutPins, wire, sensorAddresses);
}

LidarArrayConfig LidarArrayConfig::forVL53L4CD(
    uint8_t numPCF,
    uint8_t numSensors,
    const uint8_t *pcf8574Addresses,
    const LidarSensorSlot *sensorMap,
    TwoWire *wire)
{
    return forModel(LidarSensorModel::VL53L4CD, numPCF, numSensors, pcf8574Addresses, sensorMap, wire);
}

LidarArrayDebugConfig LidarArrayDebugConfig::verbose(
    Print *out,
    bool scanBeforeInit,
    bool scanEachStep,
    bool animated,
    uint16_t animationDelayMs,
    uint16_t bootDelayMs)
{
    LidarArrayDebugConfig config;
    config.out = out;
    config.level = LidarDebugLevel::Verbose;
    config.scanBeforeInit = scanBeforeInit;
    config.scanEachStep = scanEachStep;
    config.animated = animated;
    config.animationDelayMs = animationDelayMs;
    config.bootDelayMs = bootDelayMs;
    return config;
}

LidarFilterConfig LidarFilterConfig::disabled()
{
    return LidarFilterConfig();
}

LidarFilterConfig LidarFilterConfig::recommended()
{
    LidarFilterConfig config;
    config.enabled = true;
    config.medianWindow = 3;
    config.emaAlphaPercent = 50;
    config.holdLastValid = true;
    config.maxHeldReads = 2;
    config.maxJumpMm = 0;
    return config;
}

LidarArray::LidarArray(LidarSensorModel model)
{
    resetMembers();
    config_ = LidarArrayConfig::defaults(model);
}

LidarArray::LidarArray(const LidarArrayConfig &config)
{
    resetMembers();
    // Safe defaults live in the LidarArrayConfig member initializers, so a
    // hand-built configuration already carries them before it arrives here.
    config_ = config;
}

LidarArray::LidarArray(
    uint8_t numPCF,
    uint8_t numSensors,
    const uint8_t pcf8574Addresses[],
    const uint8_t xshutPins[][8])
{
    resetMembers();
    config_ = LidarArrayConfig::forVL53L0X(
        numPCF,
        numSensors,
        pcf8574Addresses,
        reinterpret_cast<const uint8_t *>(xshutPins),
        &Wire);
    config_.legacyReadBehavior = true;
}

LidarArray::~LidarArray()
{
    releaseStorage();
    releaseFilterStorage();
}

void LidarArray::resetMembers()
{
    numPCF_ = 0;
    sensorCount_ = 0;
    totalSlots_ = 0;
    initializedSensorCount_ = 0;
    configCopied_ = false;
    configValid_ = false;
    explicitSensorMap_ = false;
    filterStorageCount_ = 0;
    vl53l0xSensorCount_ = 0;
    vl53l4cdSensorCount_ = 0;

    model_ = LidarSensorModel::VL53L0X;
    wire_ = &Wire;

    timeoutMs_ = 100;
    shutdownDelayMs_ = 5;
    wakeDelayMs_ = 5;

    legacyReadBehavior_ = false;
    legacyMinDistanceMm_ = 10;
    legacyMaxDistanceMm_ = 700;
    legacyFallbackDistanceMm_ = 700;

    vl53l0xMeasurementTimingBudgetUs_ = 20000;
    applyVl53l0xVcselPeriods_ = false;
    vl53l0xPreRangeVcselPeriod_ = 14;
    vl53l0xFinalRangeVcselPeriod_ = 10;

    vl53l4cdTimingBudgetMs_ = 50;
    vl53l4cdInterMeasurementMs_ = 0;

    pcf8574Addresses_ = nullptr;
    pcf8574States_ = nullptr;
    sensorDriverIndices_ = nullptr;
    sensorSlots_ = nullptr;
    sensorReady_ = nullptr;
    sensorFilterConfigs_ = nullptr;
    sensorFilterOverrides_ = nullptr;
    filterStates_ = nullptr;
    vl53l0xSensors_ = nullptr;
    vl53l4cdSensors_ = nullptr;

    filterConfig_ = LidarFilterConfig::disabled();
    config_ = LidarArrayConfig::defaults();
}

bool LidarArray::allocateStorage(uint8_t numPCF, uint8_t numSensors, uint8_t vl53l0xSensorCount, uint8_t vl53l4cdSensorCount)
{
    if (numPCF == 0 || numSensors == 0)
    {
        return false;
    }

    numPCF_ = numPCF;
    sensorCount_ = numSensors;
    totalSlots_ = static_cast<uint8_t>(numPCF * kSlotsPerPcf);
    vl53l0xSensorCount_ = vl53l0xSensorCount;
    vl53l4cdSensorCount_ = vl53l4cdSensorCount;

    pcf8574Addresses_ = new (std::nothrow) uint8_t[numPCF_];
    pcf8574States_ = new (std::nothrow) uint8_t[numPCF_];
    sensorDriverIndices_ = new (std::nothrow) uint8_t[sensorCount_];
    sensorSlots_ = new (std::nothrow) LidarSensorSlot[sensorCount_];
    sensorReady_ = new (std::nothrow) bool[sensorCount_];

    if (pcf8574Addresses_ == nullptr ||
        pcf8574States_ == nullptr ||
        sensorDriverIndices_ == nullptr ||
        sensorSlots_ == nullptr ||
        sensorReady_ == nullptr)
    {
        releaseStorage();
        return false;
    }

    if (vl53l0xSensorCount_ > 0)
    {
        vl53l0xSensors_ = new (std::nothrow) VL53L0X[vl53l0xSensorCount_];
        if (vl53l0xSensors_ == nullptr)
        {
            releaseStorage();
            return false;
        }
    }

    if (vl53l4cdSensorCount_ > 0)
    {
        vl53l4cdSensors_ = new (std::nothrow) VL53L4CD[vl53l4cdSensorCount_];
        if (vl53l4cdSensors_ == nullptr)
        {
            releaseStorage();
            return false;
        }
    }

    return true;
}

void LidarArray::releaseStorage()
{
    delete[] pcf8574Addresses_;
    delete[] pcf8574States_;
    delete[] sensorDriverIndices_;
    delete[] sensorSlots_;
    delete[] sensorReady_;
    delete[] vl53l0xSensors_;
    delete[] vl53l4cdSensors_;

    pcf8574Addresses_ = nullptr;
    pcf8574States_ = nullptr;
    sensorDriverIndices_ = nullptr;
    sensorSlots_ = nullptr;
    sensorReady_ = nullptr;
    vl53l0xSensors_ = nullptr;
    vl53l4cdSensors_ = nullptr;
    numPCF_ = 0;
    sensorCount_ = 0;
    totalSlots_ = 0;
    initializedSensorCount_ = 0;
    configCopied_ = false;
    configValid_ = false;
    explicitSensorMap_ = false;
    vl53l0xSensorCount_ = 0;
    vl53l4cdSensorCount_ = 0;
}

bool LidarArray::ensureFilterStorage(uint8_t numSensors)
{
    if (numSensors == 0)
    {
        return false;
    }

    if (sensorFilterConfigs_ != nullptr &&
        sensorFilterOverrides_ != nullptr &&
        filterStates_ != nullptr &&
        filterStorageCount_ == numSensors)
    {
        return true;
    }

    releaseFilterStorage();

    sensorFilterConfigs_ = new (std::nothrow) LidarFilterConfig[numSensors];
    sensorFilterOverrides_ = new (std::nothrow) bool[numSensors];
    filterStates_ = new (std::nothrow) SensorFilterState[numSensors];

    if (sensorFilterConfigs_ == nullptr ||
        sensorFilterOverrides_ == nullptr ||
        filterStates_ == nullptr)
    {
        releaseFilterStorage();
        return false;
    }

    filterStorageCount_ = numSensors;
    for (uint8_t sensorIndex = 0; sensorIndex < numSensors; ++sensorIndex)
    {
        sensorFilterConfigs_[sensorIndex] = LidarFilterConfig::disabled();
        sensorFilterOverrides_[sensorIndex] = false;
        filterStates_[sensorIndex] = SensorFilterState();
    }

    return true;
}

void LidarArray::releaseFilterStorage()
{
    delete[] sensorFilterConfigs_;
    delete[] sensorFilterOverrides_;
    delete[] filterStates_;

    sensorFilterConfigs_ = nullptr;
    sensorFilterOverrides_ = nullptr;
    filterStates_ = nullptr;
    filterStorageCount_ = 0;
}

bool LidarArray::copyLegacyLayout(const LidarArrayConfig &config)
{
    if (config.xshutPins == nullptr)
    {
        return false;
    }

    for (uint8_t pcfIndex = 0; pcfIndex < numPCF_; ++pcfIndex)
    {
        uint8_t pinMask = 0;
        for (uint8_t slot = 0; slot < kSlotsPerPcf; ++slot)
        {
            const uint8_t index = static_cast<uint8_t>((pcfIndex * kSlotsPerPcf) + slot);
            const uint8_t pin = config.xshutPins[index];
            if (pin >= kSlotsPerPcf)
            {
                return false;
            }
            if ((pinMask & (1 << pin)) != 0)
            {
                return false;
            }
            pinMask |= (1 << pin);
        }
    }

    if (!isConcreteModel(config.model))
    {
        return false;
    }

    uint8_t driverIndex = 0;
    for (uint8_t sensorIndex = 0; sensorIndex < sensorCount_; ++sensorIndex)
    {
        sensorSlots_[sensorIndex].pcfIndex = sensorIndex / kSlotsPerPcf;
        sensorSlots_[sensorIndex].pin = config.xshutPins[sensorIndex];
        sensorSlots_[sensorIndex].address = config.sensorAddresses != nullptr
            ? config.sensorAddresses[sensorIndex]
            : static_cast<uint8_t>(kDefaultAddressBase + sensorIndex);
        sensorSlots_[sensorIndex].sensorId = sensorIndex;
        sensorSlots_[sensorIndex].model = config.model;
        sensorDriverIndices_[sensorIndex] = driverIndex++;
        sensorReady_[sensorIndex] = false;
    }

    return true;
}

bool LidarArray::copySparseLayout(const LidarArrayConfig &config)
{
    if (config.sensorMap == nullptr || config.sensorAddresses != nullptr)
    {
        return false;
    }

    uint8_t vl53l0xIndex = 0;
    uint8_t vl53l4cdIndex = 0;

    for (uint8_t sensorIndex = 0; sensorIndex < sensorCount_; ++sensorIndex)
    {
        sensorSlots_[sensorIndex] = config.sensorMap[sensorIndex];
        sensorSlots_[sensorIndex].model = resolveSlotModel(config.sensorMap[sensorIndex], config.model);

        if (sensorSlots_[sensorIndex].address == 0)
        {
            sensorSlots_[sensorIndex].address = static_cast<uint8_t>(kDefaultAddressBase + sensorIndex);
        }

        if (sensorSlots_[sensorIndex].sensorId < 0)
        {
            sensorSlots_[sensorIndex].sensorId = sensorIndex;
        }

        if (sensorSlots_[sensorIndex].model == LidarSensorModel::VL53L0X)
        {
            sensorDriverIndices_[sensorIndex] = vl53l0xIndex++;
        }
        else if (sensorSlots_[sensorIndex].model == LidarSensorModel::VL53L4CD)
        {
            sensorDriverIndices_[sensorIndex] = vl53l4cdIndex++;
        }
        else
        {
            return false;
        }

        sensorReady_[sensorIndex] = false;
    }

    return true;
}

void LidarArray::copyConfig(const LidarArrayConfig &config)
{
    if (config.pcf8574Addresses == nullptr ||
        (config.xshutPins == nullptr && config.sensorMap == nullptr))
    {
        configCopied_ = false;
        return;
    }

    model_ = config.model;
    wire_ = config.wire != nullptr ? config.wire : &Wire;
    timeoutMs_ = config.timeoutMs;
    shutdownDelayMs_ = config.shutdownDelayMs;
    wakeDelayMs_ = config.wakeDelayMs;
    legacyReadBehavior_ = config.legacyReadBehavior;
    legacyMinDistanceMm_ = config.legacyMinDistanceMm;
    legacyMaxDistanceMm_ = config.legacyMaxDistanceMm;
    legacyFallbackDistanceMm_ = config.legacyFallbackDistanceMm;
    vl53l0xMeasurementTimingBudgetUs_ = config.vl53l0xMeasurementTimingBudgetUs;
    applyVl53l0xVcselPeriods_ = config.applyVl53l0xVcselPeriods;
    vl53l0xPreRangeVcselPeriod_ = config.vl53l0xPreRangeVcselPeriod;
    vl53l0xFinalRangeVcselPeriod_ = config.vl53l0xFinalRangeVcselPeriod;
    vl53l4cdTimingBudgetMs_ = config.vl53l4cdTimingBudgetMs;
    vl53l4cdInterMeasurementMs_ = config.vl53l4cdInterMeasurementMs;
    explicitSensorMap_ = config.sensorMap != nullptr;

    for (uint8_t i = 0; i < numPCF_; ++i)
    {
        pcf8574Addresses_[i] = config.pcf8574Addresses[i];
        pcf8574States_[i] = 0xFF;
    }

    const bool copied = explicitSensorMap_
        ? copySparseLayout(config)
        : copyLegacyLayout(config);

    configCopied_ = copied;
}

bool LidarArray::syncConfiguration()
{
    uint8_t desiredVl53l0xCount = 0;
    uint8_t desiredVl53l4cdCount = 0;
    if (!resolveModelCounts(config_, desiredVl53l0xCount, desiredVl53l4cdCount))
    {
        configValid_ = false;
        return false;
    }

    if (!ensureFilterStorage(config_.numSensors))
    {
        configValid_ = false;
        return false;
    }

    const bool needsReallocation =
        pcf8574Addresses_ == nullptr ||
        pcf8574States_ == nullptr ||
        sensorDriverIndices_ == nullptr ||
        sensorSlots_ == nullptr ||
        sensorReady_ == nullptr ||
        numPCF_ != config_.numPCF ||
        sensorCount_ != config_.numSensors ||
        vl53l0xSensorCount_ != desiredVl53l0xCount ||
        vl53l4cdSensorCount_ != desiredVl53l4cdCount;

    if (needsReallocation)
    {
        releaseStorage();
        if (!allocateStorage(config_.numPCF, config_.numSensors, desiredVl53l0xCount, desiredVl53l4cdCount))
        {
            configValid_ = false;
            return false;
        }
    }

    copyConfig(config_);
    configValid_ = validateConfiguration();
    return configValid_;
}

bool LidarArray::validateConfiguration() const
{
    if (!configCopied_ ||
        numPCF_ == 0 ||
        sensorCount_ == 0 ||
        wire_ == nullptr ||
        pcf8574Addresses_ == nullptr ||
        pcf8574States_ == nullptr ||
        sensorDriverIndices_ == nullptr ||
        sensorSlots_ == nullptr ||
        sensorReady_ == nullptr)
    {
        return false;
    }

    if ((vl53l0xSensorCount_ > 0 && vl53l0xSensors_ == nullptr) ||
        (vl53l4cdSensorCount_ > 0 && vl53l4cdSensors_ == nullptr))
    {
        return false;
    }

    if (sensorCount_ > totalSlots_)
    {
        return false;
    }

    for (uint8_t i = 0; i < numPCF_; ++i)
    {
        if (pcf8574Addresses_[i] == 0 || pcf8574Addresses_[i] > 0x77)
        {
            return false;
        }

        for (uint8_t j = i + 1; j < numPCF_; ++j)
        {
            if (pcf8574Addresses_[i] == pcf8574Addresses_[j])
            {
                return false;
            }
        }
    }

    for (uint8_t i = 0; i < sensorCount_; ++i)
    {
        const LidarSensorSlot &slot = sensorSlots_[i];

        if (slot.pcfIndex >= numPCF_ || slot.pin >= kSlotsPerPcf)
        {
            return false;
        }

        if (!isConcreteModel(slot.model))
        {
            return false;
        }

        if (slot.sensorId < 0)
        {
            return false;
        }

        if (slot.address < kMinimumManualAddress ||
            slot.address > 0x77 ||
            slot.address == kTofDefaultAddress)
        {
            return false;
        }

        if ((slot.model == LidarSensorModel::VL53L0X && sensorDriverIndices_[i] >= vl53l0xSensorCount_) ||
            (slot.model == LidarSensorModel::VL53L4CD && sensorDriverIndices_[i] >= vl53l4cdSensorCount_))
        {
            return false;
        }

        for (uint8_t pcfIndex = 0; pcfIndex < numPCF_; ++pcfIndex)
        {
            if (slot.address == pcf8574Addresses_[pcfIndex])
            {
                return false;
            }
        }

        for (uint8_t otherIndex = i + 1; otherIndex < sensorCount_; ++otherIndex)
        {
            const LidarSensorSlot &otherSlot = sensorSlots_[otherIndex];

            if (slot.pcfIndex == otherSlot.pcfIndex && slot.pin == otherSlot.pin)
            {
                return false;
            }

            if (slot.sensorId == otherSlot.sensorId)
            {
                return false;
            }

            if (slot.address == otherSlot.address)
            {
                return false;
            }
        }
    }

    return true;
}

bool LidarArray::begin()
{
    if (!syncConfiguration())
    {
        logMessage(LidarDebugLevel::Errors, F("Invalid configuration."));
        return false;
    }

    for (uint8_t i = 0; i < sensorCount_; ++i)
    {
        sensorReady_[i] = false;
    }
    initializedSensorCount_ = 0;
    resetFilters();

    if (debugConfig_.scanBeforeInit)
    {
        logMessage(LidarDebugLevel::Info, F("I2C scan before sensor shutdown."));
        scanI2C();
    }

    if (debugConfig_.bootDelayMs > 0)
    {
        if (shouldLog(LidarDebugLevel::Info))
        {
            debugConfig_.out->print(F("[LidarArray] Debug boot delay: "));
            debugConfig_.out->print(debugConfig_.bootDelayMs);
            debugConfig_.out->println(F(" ms"));
        }
        delay(debugConfig_.bootDelayMs);
    }

    if (!shutdownAllSensors())
    {
        logMessage(LidarDebugLevel::Errors, F("Failed to place sensors in shutdown."));
        return false;
    }

    if (debugConfig_.scanEachStep)
    {
        logMessage(LidarDebugLevel::Info, F("I2C scan with all TOF sensors in shutdown."));
        scanI2C();
    }

    bool allSensorsReady = true;

    for (uint8_t sensorIndex = 0; sensorIndex < sensorCount_; ++sensorIndex)
    {
        const bool ready = initializeSensor(sensorIndex);
        sensorReady_[sensorIndex] = ready;
        if (ready)
        {
            ++initializedSensorCount_;
        }
        else
        {
            allSensorsReady = false;
        }
    }

    if (allSensorsReady)
    {
        logMessage(LidarDebugLevel::Info, F("All sensors initialized."));
    }
    else
    {
        logMessage(LidarDebugLevel::Errors, F("Initialization completed with unavailable sensors."));
    }

    return allSensorsReady;
}

bool LidarArray::initSensors()
{
    return begin();
}

bool LidarArray::initSensors(int measurementT)
{
    config_.vl53l0xMeasurementTimingBudgetUs = static_cast<uint32_t>(measurementT);
    vl53l0xMeasurementTimingBudgetUs_ = static_cast<uint32_t>(measurementT);
    return begin();
}

bool LidarArray::initSensors(int measurementT, uint8_t preRange, uint8_t finalRange)
{
    config_.vl53l0xMeasurementTimingBudgetUs = static_cast<uint32_t>(measurementT);
    config_.applyVl53l0xVcselPeriods = true;
    config_.vl53l0xPreRangeVcselPeriod = preRange;
    config_.vl53l0xFinalRangeVcselPeriod = finalRange;
    vl53l0xMeasurementTimingBudgetUs_ = static_cast<uint32_t>(measurementT);
    applyVl53l0xVcselPeriods_ = true;
    vl53l0xPreRangeVcselPeriod_ = preRange;
    vl53l0xFinalRangeVcselPeriod_ = finalRange;
    return begin();
}

bool LidarArray::initSensors(int measurementT, uint8_t preRange, uint8_t finalRange, int timeout)
{
    config_.vl53l0xMeasurementTimingBudgetUs = static_cast<uint32_t>(measurementT);
    config_.applyVl53l0xVcselPeriods = true;
    config_.vl53l0xPreRangeVcselPeriod = preRange;
    config_.vl53l0xFinalRangeVcselPeriod = finalRange;
    config_.timeoutMs = timeout > 0 ? static_cast<uint16_t>(timeout) : 0;
    vl53l0xMeasurementTimingBudgetUs_ = static_cast<uint32_t>(measurementT);
    applyVl53l0xVcselPeriods_ = true;
    vl53l0xPreRangeVcselPeriod_ = preRange;
    vl53l0xFinalRangeVcselPeriod_ = finalRange;
    timeoutMs_ = timeout > 0 ? static_cast<uint16_t>(timeout) : 0;
    return begin();
}

LidarArrayConfig &LidarArray::config()
{
    return config_;
}

const LidarArrayConfig &LidarArray::config() const
{
    return config_;
}

LidarArrayDebugConfig &LidarArray::debug()
{
    return debugConfig_;
}

const LidarArrayDebugConfig &LidarArray::debug() const
{
    return debugConfig_;
}

LidarFilterConfig &LidarArray::filter()
{
    return filterConfig_;
}

const LidarFilterConfig &LidarArray::filter() const
{
    return filterConfig_;
}

void LidarArray::setLayout(uint8_t numPCF, uint8_t numSensors, const uint8_t *pcf8574Addresses, const uint8_t *xshutPins)
{
    config_.numPCF = numPCF;
    config_.numSensors = numSensors;
    config_.pcf8574Addresses = pcf8574Addresses;
    config_.xshutPins = xshutPins;
    config_.sensorMap = nullptr;
}

void LidarArray::setLayout(uint8_t numPCF, uint8_t numSensors, const uint8_t *pcf8574Addresses, const LidarSensorSlot *sensorMap)
{
    config_.numPCF = numPCF;
    config_.numSensors = numSensors;
    config_.pcf8574Addresses = pcf8574Addresses;
    config_.xshutPins = nullptr;
    config_.sensorAddresses = nullptr;
    config_.sensorMap = sensorMap;
}

void LidarArray::setSensorAddresses(const uint8_t *sensorAddresses)
{
    config_.sensorAddresses = sensorAddresses;
}

void LidarArray::setWire(TwoWire *wire)
{
    config_.wire = wire != nullptr ? wire : &Wire;
    wire_ = config_.wire;
}

void LidarArray::setTimeout(uint16_t timeoutMs)
{
    config_.timeoutMs = timeoutMs;
    timeoutMs_ = timeoutMs;
}

void LidarArray::setVL53L4CDTiming(uint8_t timingBudgetMs, uint32_t interMeasurementMs)
{
    config_.vl53l4cdTimingBudgetMs = timingBudgetMs;
    config_.vl53l4cdInterMeasurementMs = interMeasurementMs;
    vl53l4cdTimingBudgetMs_ = timingBudgetMs;
    vl53l4cdInterMeasurementMs_ = interMeasurementMs;

    for (uint8_t i = 0; i < sensorCount_; ++i)
    {
        VL53L4CD *sensor = getVL53L4CDSensor(i);
        if (sensor == nullptr || !sensorReady_[i])
        {
            continue;
        }

        if (!sensor->setRangeTiming(vl53l4cdTimingBudgetMs_, vl53l4cdInterMeasurementMs_))
        {
            logSensorStep(i, F("failed to update VL53L4CD timing"), LidarDebugLevel::Errors);
        }
    }
}

void LidarArray::setFilterConfig(const LidarFilterConfig &config)
{
    filterConfig_ = sanitizeFilterConfig(config);
    resetFilters();
}

void LidarArray::setSensorFilterConfig(uint8_t sensorIndex, const LidarFilterConfig &config)
{
    const uint8_t configuredSensorCount = sensorCount_ != 0 ? sensorCount_ : config_.numSensors;
    if (configuredSensorCount == 0 || sensorIndex >= configuredSensorCount)
    {
        return;
    }

    if (!ensureFilterStorage(configuredSensorCount))
    {
        return;
    }

    sensorFilterConfigs_[sensorIndex] = sanitizeFilterConfig(config);
    sensorFilterOverrides_[sensorIndex] = true;
    resetFilter(sensorIndex);
}

void LidarArray::clearSensorFilterConfig(uint8_t sensorIndex)
{
    const uint8_t configuredSensorCount = sensorCount_ != 0 ? sensorCount_ : config_.numSensors;
    if (configuredSensorCount == 0 ||
        sensorIndex >= configuredSensorCount ||
        sensorFilterConfigs_ == nullptr ||
        sensorFilterOverrides_ == nullptr)
    {
        return;
    }

    sensorFilterConfigs_[sensorIndex] = LidarFilterConfig::disabled();
    sensorFilterOverrides_[sensorIndex] = false;
    resetFilter(sensorIndex);
}

void LidarArray::clearSensorFilterConfigs()
{
    if (sensorFilterConfigs_ == nullptr || sensorFilterOverrides_ == nullptr)
    {
        return;
    }

    for (uint8_t sensorIndex = 0; sensorIndex < filterStorageCount_; ++sensorIndex)
    {
        sensorFilterConfigs_[sensorIndex] = LidarFilterConfig::disabled();
        sensorFilterOverrides_[sensorIndex] = false;
    }

    resetFilters();
}

bool LidarArray::hasSensorFilterConfig(uint8_t sensorIndex) const
{
    if (sensorFilterOverrides_ == nullptr || sensorIndex >= filterStorageCount_)
    {
        return false;
    }

    return sensorFilterOverrides_[sensorIndex];
}

LidarFilterConfig LidarArray::getEffectiveFilterConfig(uint8_t sensorIndex) const
{
    if (sensorIndex < filterStorageCount_ &&
        sensorFilterOverrides_ != nullptr &&
        sensorFilterConfigs_ != nullptr &&
        sensorFilterOverrides_[sensorIndex])
    {
        return sanitizeFilterConfig(sensorFilterConfigs_[sensorIndex]);
    }

    if (sensorIndex < (sensorCount_ != 0 ? sensorCount_ : config_.numSensors))
    {
        return sanitizeFilterConfig(filterConfig_);
    }

    return LidarFilterConfig::disabled();
}

void LidarArray::resetFilter(uint8_t sensorIndex)
{
    const uint8_t configuredSensorCount = sensorCount_ != 0 ? sensorCount_ : config_.numSensors;
    if (configuredSensorCount == 0 || sensorIndex >= configuredSensorCount)
    {
        return;
    }

    if (!ensureFilterStorage(configuredSensorCount))
    {
        return;
    }

    resetFilterState(sensorIndex);
}

void LidarArray::resetFilters()
{
    const uint8_t configuredSensorCount = sensorCount_ != 0 ? sensorCount_ : config_.numSensors;
    if (configuredSensorCount == 0)
    {
        return;
    }

    if (!ensureFilterStorage(configuredSensorCount))
    {
        return;
    }

    for (uint8_t sensorIndex = 0; sensorIndex < configuredSensorCount; ++sensorIndex)
    {
        resetFilterState(sensorIndex);
    }
}

uint16_t LidarArray::readSensor(uint8_t sensorIndex)
{
    const LidarReading reading = readReading(sensorIndex, true);
    if (legacyReadBehavior_)
    {
        return applyLegacyReadBehavior(reading);
    }
    return reading.distanceMm;
}

uint16_t LidarArray::readSensorNB(uint8_t sensorIndex)
{
    const LidarReading reading = readReading(sensorIndex, false);
    if (legacyReadBehavior_)
    {
        return applyLegacyReadBehavior(reading);
    }
    return reading.distanceMm;
}

uint16_t LidarArray::readSensorById(int16_t sensorId)
{
    const LidarReading reading = readReadingById(sensorId, true);
    if (legacyReadBehavior_)
    {
        return applyLegacyReadBehavior(reading);
    }
    return reading.distanceMm;
}

uint16_t LidarArray::readSensorNBById(int16_t sensorId)
{
    const LidarReading reading = readReadingById(sensorId, false);
    if (legacyReadBehavior_)
    {
        return applyLegacyReadBehavior(reading);
    }
    return reading.distanceMm;
}

LidarReading LidarArray::readReading(uint8_t sensorIndex, bool blocking)
{
    if (sensorIndex >= sensorCount_)
    {
        return buildInvalidReading(sensorIndex, kLibraryStatusInvalidIndex);
    }

    if (!sensorReady_[sensorIndex])
    {
        return buildInvalidReading(sensorIndex, kLibraryStatusNotReady);
    }

    if (sensorSlots_[sensorIndex].model == LidarSensorModel::VL53L0X)
    {
        return readVL53L0X(sensorIndex, blocking);
    }
    if (sensorSlots_[sensorIndex].model == LidarSensorModel::VL53L4CD)
    {
        return readVL53L4CD(sensorIndex, blocking);
    }

    return buildInvalidReading(sensorIndex, kLibraryStatusNotReady);
}

LidarReading LidarArray::readReadingById(int16_t sensorId, bool blocking)
{
    const int16_t sensorIndex = indexOfSensorId(sensorId);
    if (sensorIndex < 0)
    {
        return buildInvalidReadingById(sensorId, kLibraryStatusInvalidIndex);
    }

    return readReading(static_cast<uint8_t>(sensorIndex), blocking);
}

uint16_t LidarArray::readFilteredSensor(uint8_t sensorIndex)
{
    return readFilteredReading(sensorIndex, true).distanceMm;
}

uint16_t LidarArray::readFilteredSensorNB(uint8_t sensorIndex)
{
    return readFilteredReading(sensorIndex, false).distanceMm;
}

uint16_t LidarArray::readFilteredSensorById(int16_t sensorId)
{
    return readFilteredReadingById(sensorId, true).distanceMm;
}

uint16_t LidarArray::readFilteredSensorNBById(int16_t sensorId)
{
    return readFilteredReadingById(sensorId, false).distanceMm;
}

LidarFilteredReading LidarArray::readFilteredReading(uint8_t sensorIndex, bool blocking)
{
    if (sensorIndex >= sensorCount_)
    {
        return buildInvalidFilteredReading(sensorIndex, kLibraryStatusInvalidIndex);
    }

    const LidarReading raw = readReading(sensorIndex, blocking);
    return applyFilterPipeline(sensorIndex, raw);
}

LidarFilteredReading LidarArray::readFilteredReadingById(int16_t sensorId, bool blocking)
{
    const int16_t sensorIndex = indexOfSensorId(sensorId);
    if (sensorIndex < 0)
    {
        return buildInvalidFilteredReadingById(sensorId, kLibraryStatusInvalidIndex);
    }

    return readFilteredReading(static_cast<uint8_t>(sensorIndex), blocking);
}

VL53L0X &LidarArray::getSensor(uint8_t sensorIndex)
{
    VL53L0X *sensor = getVL53L0XSensor(sensorIndex);
    if (sensor != nullptr)
    {
        return *sensor;
    }

    static VL53L0X nullSensor;
    return nullSensor;
}

VL53L0X *LidarArray::getVL53L0XSensor(uint8_t sensorIndex)
{
    if (sensorIndex >= sensorCount_ ||
        sensorSlots_ == nullptr ||
        sensorDriverIndices_ == nullptr ||
        vl53l0xSensors_ == nullptr ||
        sensorSlots_[sensorIndex].model != LidarSensorModel::VL53L0X)
    {
        return nullptr;
    }
    return &vl53l0xSensors_[sensorDriverIndices_[sensorIndex]];
}

VL53L4CD *LidarArray::getVL53L4CDSensor(uint8_t sensorIndex)
{
    if (sensorIndex >= sensorCount_ ||
        sensorSlots_ == nullptr ||
        sensorDriverIndices_ == nullptr ||
        vl53l4cdSensors_ == nullptr ||
        sensorSlots_[sensorIndex].model != LidarSensorModel::VL53L4CD)
    {
        return nullptr;
    }
    return &vl53l4cdSensors_[sensorDriverIndices_[sensorIndex]];
}

void LidarArray::setMeasurementTimingBudget(uint32_t timingBudget)
{
    config_.vl53l0xMeasurementTimingBudgetUs = timingBudget;
    vl53l0xMeasurementTimingBudgetUs_ = timingBudget;

    for (uint8_t i = 0; i < sensorCount_; ++i)
    {
        VL53L0X *sensor = getVL53L0XSensor(i);
        if (sensor == nullptr || !sensorReady_[i])
        {
            continue;
        }

        if (!sensor->setMeasurementTimingBudget(vl53l0xMeasurementTimingBudgetUs_))
        {
            logSensorStep(i, F("failed to update timing budget"), LidarDebugLevel::Errors);
        }
    }
}

void LidarArray::setVcselPulsePeriod(uint8_t type, uint8_t period)
{
    if (type == 0)
    {
        config_.vl53l0xPreRangeVcselPeriod = period;
        vl53l0xPreRangeVcselPeriod_ = period;
    }
    else
    {
        config_.vl53l0xFinalRangeVcselPeriod = period;
        vl53l0xFinalRangeVcselPeriod_ = period;
    }
    config_.applyVl53l0xVcselPeriods = true;
    applyVl53l0xVcselPeriods_ = true;

    for (uint8_t i = 0; i < sensorCount_; ++i)
    {
        VL53L0X *sensor = getVL53L0XSensor(i);
        if (sensor == nullptr || !sensorReady_[i])
        {
            continue;
        }

        bool applied = false;
        if (type == 0)
        {
            applied = sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, period);
        }
        else if (type == 1)
        {
            applied = sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, period);
        }

        if (!applied)
        {
            logSensorStep(i, F("failed to update VCSEL period"), LidarDebugLevel::Errors);
        }
    }
}

void LidarArray::setDebugConfig(const LidarArrayDebugConfig &config)
{
    debugConfig_ = config;
}

void LidarArray::setDebugOutput(Print *out)
{
    debugConfig_.out = out;
}

void LidarArray::setDebugLevel(LidarDebugLevel level)
{
    debugConfig_.level = level;
}

void LidarArray::setDebugScanBeforeInit(bool enabled)
{
    debugConfig_.scanBeforeInit = enabled;
}

void LidarArray::setDebugScanEachStep(bool enabled)
{
    debugConfig_.scanEachStep = enabled;
}

void LidarArray::setDebugBootDelay(uint16_t delayMs)
{
    debugConfig_.bootDelayMs = delayMs;
}

void LidarArray::setDebugStepDelay(uint16_t delayMs)
{
    debugConfig_.animationDelayMs = delayMs;
    debugConfig_.animated = delayMs > 0;
}

uint8_t LidarArray::getSensorCount() const
{
    return sensorCount_ != 0 ? sensorCount_ : config_.numSensors;
}

uint8_t LidarArray::getInitializedSensorCount() const
{
    return initializedSensorCount_;
}

bool LidarArray::isSensorReady(uint8_t sensorIndex) const
{
    return sensorIndex < sensorCount_ && sensorReady_[sensorIndex];
}

int16_t LidarArray::getSensorId(uint8_t sensorIndex) const
{
    if (sensorIndex >= sensorCount_ || sensorSlots_ == nullptr)
    {
        return -1;
    }

    return sensorSlots_[sensorIndex].sensorId;
}

LidarSensorModel LidarArray::getSensorModel(uint8_t sensorIndex) const
{
    if (sensorIndex >= sensorCount_ || sensorSlots_ == nullptr)
    {
        return LidarSensorModel::InheritArrayDefault;
    }

    return sensorSlots_[sensorIndex].model;
}

int16_t LidarArray::indexOfSensorId(int16_t sensorId) const
{
    if (sensorSlots_ == nullptr)
    {
        return -1;
    }

    for (uint8_t sensorIndex = 0; sensorIndex < sensorCount_; ++sensorIndex)
    {
        if (sensorSlots_[sensorIndex].sensorId == sensorId)
        {
            return sensorIndex;
        }
    }

    return -1;
}

LidarSensorModel LidarArray::getSensorModelById(int16_t sensorId) const
{
    const int16_t sensorIndex = indexOfSensorId(sensorId);
    if (sensorIndex < 0)
    {
        return LidarSensorModel::InheritArrayDefault;
    }

    return getSensorModel(static_cast<uint8_t>(sensorIndex));
}

const LidarSensorSlot *LidarArray::getSensorSlot(uint8_t sensorIndex) const
{
    if (sensorIndex >= sensorCount_ || sensorSlots_ == nullptr)
    {
        return nullptr;
    }

    return &sensorSlots_[sensorIndex];
}

uint8_t LidarArray::scanI2C()
{
    uint8_t foundCount = 0;

    if (shouldLog(LidarDebugLevel::Info))
    {
        debugConfig_.out->println(F("[LidarArray] I2C scan started."));
    }

    for (uint8_t address = 1; address < 127; ++address)
    {
        wire_->beginTransmission(address);
        const uint8_t errorCode = wire_->endTransmission();
        if (errorCode == 0)
        {
            ++foundCount;
            logScanResult(address, errorCode);
        }
        else if (errorCode == 4 && shouldLog(LidarDebugLevel::Verbose))
        {
            logScanResult(address, errorCode);
        }
    }

    if (shouldLog(LidarDebugLevel::Info))
    {
        debugConfig_.out->print(F("[LidarArray] I2C scan complete. Devices found: "));
        debugConfig_.out->println(foundCount);
    }

    return foundCount;
}

bool LidarArray::shutdownAllSensors()
{
    bool success = true;

    for (uint8_t pcfIndex = 0; pcfIndex < numPCF_; ++pcfIndex)
    {
        pcf8574States_[pcfIndex] = 0x00;
        if (!writePcf8574State(pcfIndex))
        {
            success = false;
        }
    }

    if (shutdownDelayMs_ > 0)
    {
        delay(shutdownDelayMs_);
    }

    return success;
}

bool LidarArray::bringSensorOutOfShutdown(uint8_t sensorIndex)
{
    const LidarSensorSlot &slot = sensorSlots_[sensorIndex];
    const bool success = pcf8574Write(slot.pcfIndex, slot.pin, true);

    if (wakeDelayMs_ > 0)
    {
        delay(wakeDelayMs_);
    }

    return success;
}

bool LidarArray::initializeSensor(uint8_t sensorIndex)
{
    const LidarSensorSlot &slot = sensorSlots_[sensorIndex];
    const uint8_t address = slot.address;

    logSensorStep(sensorIndex, F("starting initialization"), LidarDebugLevel::Info);

    if (!bringSensorOutOfShutdown(sensorIndex))
    {
        logSensorStep(sensorIndex, F("failed to enable XSHUT"), LidarDebugLevel::Errors);
        return false;
    }

    bool ready = false;
    if (slot.model == LidarSensorModel::VL53L0X)
    {
        ready = initializeVL53L0X(sensorIndex, address);
    }
    else if (slot.model == LidarSensorModel::VL53L4CD)
    {
        ready = initializeVL53L4CD(sensorIndex, address);
    }

    if (!ready)
    {
        pcf8574Write(slot.pcfIndex, slot.pin, false);
        if (shutdownDelayMs_ > 0)
        {
            delay(shutdownDelayMs_);
        }
        logSensorStep(sensorIndex, F("sensor unavailable"), LidarDebugLevel::Errors);
    }
    else
    {
        logSensorStep(sensorIndex, F("sensor ready"), LidarDebugLevel::Info);
    }

    if (debugConfig_.scanEachStep)
    {
        scanI2C();
    }

    return ready;
}

uint16_t LidarArray::resolveEffectiveTimeoutMs() const
{
    if (timeoutMs_ > 0)
    {
        return timeoutMs_;
    }

    // A zero timeout makes the Pololu drivers spin forever on an unresponsive sensor,
    // both while ranging and inside init(). Derive a bounded timeout from the longest
    // configured measurement period instead.
    uint32_t measurementPeriodMs = 0;
    if (vl53l0xSensorCount_ > 0)
    {
        measurementPeriodMs = vl53l0xMeasurementTimingBudgetUs_ / 1000UL;
    }
    if (vl53l4cdSensorCount_ > 0 && vl53l4cdTimingBudgetMs_ > measurementPeriodMs)
    {
        measurementPeriodMs = vl53l4cdTimingBudgetMs_;
    }

    const uint32_t derivedTimeoutMs = (measurementPeriodMs * 2UL) + 50UL;
    if (derivedTimeoutMs < kMinimumAutoTimeoutMs)
    {
        return kMinimumAutoTimeoutMs;
    }
    if (derivedTimeoutMs > kMaximumAutoTimeoutMs)
    {
        return kMaximumAutoTimeoutMs;
    }

    return static_cast<uint16_t>(derivedTimeoutMs);
}

bool LidarArray::initializeVL53L0X(uint8_t sensorIndex, uint8_t address)
{
    VL53L0X *sensorPointer = getVL53L0XSensor(sensorIndex);
    if (sensorPointer == nullptr)
    {
        return false;
    }

    const uint16_t effectiveTimeoutMs = resolveEffectiveTimeoutMs();

    VL53L0X &sensor = *sensorPointer;
    sensor.setBus(wire_);
    sensor.setTimeout(effectiveTimeoutMs);

    if (!sensor.init())
    {
        return false;
    }

    sensor.setAddress(address);
    sensor.setTimeout(effectiveTimeoutMs);

    if (!sensor.setMeasurementTimingBudget(vl53l0xMeasurementTimingBudgetUs_))
    {
        return false;
    }

    if (applyVl53l0xVcselPeriods_)
    {
        if (!sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, vl53l0xPreRangeVcselPeriod_))
        {
            return false;
        }
        if (!sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, vl53l0xFinalRangeVcselPeriod_))
        {
            return false;
        }
    }

    sensor.startContinuous();
    return true;
}

bool LidarArray::initializeVL53L4CD(uint8_t sensorIndex, uint8_t address)
{
    VL53L4CD *sensorPointer = getVL53L4CDSensor(sensorIndex);
    if (sensorPointer == nullptr)
    {
        return false;
    }

    const uint16_t effectiveTimeoutMs = resolveEffectiveTimeoutMs();

    VL53L4CD &sensor = *sensorPointer;
    sensor.setBus(wire_);
    sensor.setTimeout(effectiveTimeoutMs);

    if (!sensor.init())
    {
        return false;
    }

    sensor.setAddress(address);
    sensor.setTimeout(effectiveTimeoutMs);

    if (!sensor.setRangeTiming(vl53l4cdTimingBudgetMs_, vl53l4cdInterMeasurementMs_))
    {
        return false;
    }

    sensor.startContinuous();
    return true;
}

bool LidarArray::pcf8574Write(uint8_t pcf8574Index, uint8_t pin, bool state)
{
    if (pcf8574Index >= numPCF_ || pin >= kSlotsPerPcf)
    {
        return false;
    }

    if (state)
    {
        pcf8574States_[pcf8574Index] |= (1 << pin);
    }
    else
    {
        pcf8574States_[pcf8574Index] &= static_cast<uint8_t>(~(1 << pin));
    }

    return writePcf8574State(pcf8574Index);
}

bool LidarArray::writePcf8574State(uint8_t pcf8574Index)
{
    if (pcf8574Index >= numPCF_)
    {
        return false;
    }

    wire_->beginTransmission(pcf8574Addresses_[pcf8574Index]);
    wire_->write(pcf8574States_[pcf8574Index]);
    return wire_->endTransmission() == 0;
}

LidarReading LidarArray::buildInvalidReading(uint8_t sensorIndex, uint8_t status) const
{
    LidarReading reading;
    reading.status = status;
    if (sensorIndex < sensorCount_ && sensorSlots_ != nullptr)
    {
        reading.address = sensorSlots_[sensorIndex].address;
        reading.sensorId = sensorSlots_[sensorIndex].sensorId;
        reading.model = sensorSlots_[sensorIndex].model;
    }
    return reading;
}

LidarReading LidarArray::buildInvalidReadingById(int16_t sensorId, uint8_t status) const
{
    LidarReading reading;
    reading.status = status;
    reading.sensorId = sensorId;
    return reading;
}

LidarFilteredReading LidarArray::buildInvalidFilteredReading(uint8_t sensorIndex, uint8_t status) const
{
    LidarFilteredReading reading;
    reading.raw = buildInvalidReading(sensorIndex, status);
    return reading;
}

LidarFilteredReading LidarArray::buildInvalidFilteredReadingById(int16_t sensorId, uint8_t status) const
{
    LidarFilteredReading reading;
    reading.raw = buildInvalidReadingById(sensorId, status);
    return reading;
}

LidarFilterConfig LidarArray::sanitizeFilterConfig(const LidarFilterConfig &config)
{
    LidarFilterConfig sanitized = config;

    if (sanitized.medianWindow <= 1)
    {
        sanitized.medianWindow = 1;
    }
    else if (sanitized.medianWindow <= 3)
    {
        sanitized.medianWindow = 3;
    }
    else
    {
        sanitized.medianWindow = 5;
    }

    if (sanitized.emaAlphaPercent > 100)
    {
        sanitized.emaAlphaPercent = 100;
    }

    if (!sanitized.holdLastValid || sanitized.maxHeldReads == 0)
    {
        sanitized.holdLastValid = false;
        sanitized.maxHeldReads = 0;
    }

    if (sanitized.maxJumpMm > 0 && sanitized.maxJumpRejections == 0)
    {
        sanitized.maxJumpRejections = 1;
    }

    return sanitized;
}

void LidarArray::resetFilterState(uint8_t sensorIndex)
{
    if (filterStates_ == nullptr || sensorIndex >= filterStorageCount_)
    {
        return;
    }

    filterStates_[sensorIndex] = SensorFilterState();
}

void LidarArray::resyncFilterState(uint8_t sensorIndex, uint16_t value)
{
    if (filterStates_ == nullptr || sensorIndex >= filterStorageCount_)
    {
        return;
    }

    filterStates_[sensorIndex] = SensorFilterState();
    filterStates_[sensorIndex].lastFilteredValue = value;
    filterStates_[sensorIndex].hasLastFilteredValue = true;
    appendValidFilterSample(sensorIndex, value);
}

uint16_t LidarArray::medianFromSamples(const uint16_t *samples, uint8_t count) const
{
    if (count == 0)
    {
        return 0;
    }

    uint16_t sorted[kMaxFilterWindow] = {0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < count; ++i)
    {
        sorted[i] = samples[i];
    }

    for (uint8_t i = 1; i < count; ++i)
    {
        uint16_t value = sorted[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && sorted[j] > value)
        {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = value;
    }

    return sorted[count / 2];
}

uint16_t LidarArray::computeMedianCandidate(uint8_t sensorIndex, uint16_t sample, uint8_t window) const
{
    if (filterStates_ == nullptr || sensorIndex >= filterStorageCount_ || window <= 1)
    {
        return sample;
    }

    const SensorFilterState &state = filterStates_[sensorIndex];
    if (state.sampleCount + 1 < window)
    {
        return sample;
    }

    uint16_t candidateSamples[kMaxFilterWindow] = {0, 0, 0, 0, 0};
    uint8_t candidateCount = 0;
    const uint8_t historyCount = static_cast<uint8_t>(window - 1);

    for (uint8_t offset = historyCount; offset > 0; --offset)
    {
        const uint8_t index = static_cast<uint8_t>((state.sampleHead + kMaxFilterWindow - offset) % kMaxFilterWindow);
        candidateSamples[candidateCount++] = state.samples[index];
    }

    candidateSamples[candidateCount++] = sample;
    return medianFromSamples(candidateSamples, candidateCount);
}

void LidarArray::appendValidFilterSample(uint8_t sensorIndex, uint16_t sample)
{
    if (filterStates_ == nullptr || sensorIndex >= filterStorageCount_)
    {
        return;
    }

    SensorFilterState &state = filterStates_[sensorIndex];
    state.samples[state.sampleHead] = sample;
    state.sampleHead = static_cast<uint8_t>((state.sampleHead + 1) % kMaxFilterWindow);
    if (state.sampleCount < kMaxFilterWindow)
    {
        ++state.sampleCount;
    }
}

uint16_t LidarArray::applyEma(uint16_t previousValue, uint16_t sample, uint8_t alphaPercent) const
{
    if (alphaPercent == 0 || alphaPercent >= 100)
    {
        return sample;
    }

    const uint32_t numerator =
        (static_cast<uint32_t>(alphaPercent) * sample) +
        (static_cast<uint32_t>(100 - alphaPercent) * previousValue);
    return static_cast<uint16_t>((numerator + 50U) / 100U);
}

LidarFilteredReading LidarArray::applyFilterPipeline(uint8_t sensorIndex, const LidarReading &raw)
{
    LidarFilteredReading filtered;
    filtered.raw = raw;

    const LidarFilterConfig config = getEffectiveFilterConfig(sensorIndex);
    if (!config.enabled)
    {
        filtered.distanceMm = raw.distanceMm;
        filtered.valid = raw.valid;
        return filtered;
    }

    filtered.filterApplied = true;

    if (filterStates_ == nullptr || sensorIndex >= filterStorageCount_)
    {
        filtered.distanceMm = raw.distanceMm;
        filtered.valid = raw.valid;
        return filtered;
    }

    SensorFilterState &state = filterStates_[sensorIndex];

    if (!raw.valid || raw.timeout)
    {
        if (config.holdLastValid &&
            state.hasLastFilteredValue &&
            state.heldReads < config.maxHeldReads)
        {
            filtered.distanceMm = state.lastFilteredValue;
            filtered.valid = true;
            filtered.heldLastValid = true;
            ++state.heldReads;
            return filtered;
        }

        filtered.distanceMm = 0;
        filtered.valid = false;
        return filtered;
    }

    const uint16_t medianCandidate = computeMedianCandidate(sensorIndex, raw.distanceMm, config.medianWindow);

    if (config.maxJumpMm > 0 && state.hasLastFilteredValue)
    {
        const uint16_t jumpMm = state.lastFilteredValue > medianCandidate
            ? static_cast<uint16_t>(state.lastFilteredValue - medianCandidate)
            : static_cast<uint16_t>(medianCandidate - state.lastFilteredValue);

        if (jumpMm > config.maxJumpMm)
        {
            if (state.rejectedReads >= config.maxJumpRejections)
            {
                // The rejection budget is spent, so the new distance level is real.
                // Without this the filter would keep measuring every future sample
                // against a stale value and reject all of them forever.
                resyncFilterState(sensorIndex, raw.distanceMm);
                filtered.distanceMm = raw.distanceMm;
                filtered.valid = true;
                filtered.jumpResynced = true;
                return filtered;
            }

            ++state.rejectedReads;
            filtered.jumpRejected = true;

            if (config.holdLastValid &&
                state.heldReads < config.maxHeldReads)
            {
                filtered.distanceMm = state.lastFilteredValue;
                filtered.valid = true;
                filtered.heldLastValid = true;
                ++state.heldReads;
                return filtered;
            }

            filtered.distanceMm = 0;
            filtered.valid = false;
            return filtered;
        }
    }

    appendValidFilterSample(sensorIndex, raw.distanceMm);

    uint16_t finalValue = medianCandidate;
    if (config.emaAlphaPercent != 0)
    {
        if (state.hasLastFilteredValue)
        {
            finalValue = applyEma(state.lastFilteredValue, medianCandidate, config.emaAlphaPercent);
        }
    }

    state.lastFilteredValue = finalValue;
    state.hasLastFilteredValue = true;
    state.heldReads = 0;
    state.rejectedReads = 0;

    filtered.distanceMm = finalValue;
    filtered.valid = true;
    return filtered;
}

LidarReading LidarArray::readVL53L0X(uint8_t sensorIndex, bool blocking)
{
    VL53L0X *sensorPointer = getVL53L0XSensor(sensorIndex);
    if (sensorPointer == nullptr)
    {
        return buildInvalidReading(sensorIndex, kLibraryStatusNotReady);
    }

    VL53L0X &sensor = *sensorPointer;
    LidarReading reading;
    reading.address = sensorSlots_[sensorIndex].address;
    reading.sensorId = sensorSlots_[sensorIndex].sensorId;
    reading.model = sensorSlots_[sensorIndex].model;

    if (!blocking)
    {
        const uint8_t interruptStatus = sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
        if ((interruptStatus & 0x07) == 0)
        {
            reading.status = kLibraryStatusNotReady;
            return reading;
        }

        const uint8_t rangeStatus = sensor.readReg(VL53L0X::RESULT_RANGE_STATUS);
        reading.distanceMm = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
        reading.status = (rangeStatus & 0x78) >> 3;
        reading.dataReady = true;
        reading.valid = reading.status == 0 && reading.distanceMm > 0;
        return reading;
    }

    reading.distanceMm = sensor.readRangeContinuousMillimeters();
    reading.timeout = sensor.timeoutOccurred();
    if (reading.timeout)
    {
        reading.distanceMm = 0;
        reading.status = kLibraryStatusTimeout;
        return reading;
    }

    reading.status = (sensor.readReg(VL53L0X::RESULT_RANGE_STATUS) & 0x78) >> 3;
    reading.dataReady = true;
    reading.valid = reading.status == 0 && reading.distanceMm > 0;
    return reading;
}

LidarReading LidarArray::readVL53L4CD(uint8_t sensorIndex, bool blocking)
{
    VL53L4CD *sensorPointer = getVL53L4CDSensor(sensorIndex);
    if (sensorPointer == nullptr)
    {
        return buildInvalidReading(sensorIndex, kLibraryStatusNotReady);
    }

    VL53L4CD &sensor = *sensorPointer;
    LidarReading reading;
    reading.address = sensorSlots_[sensorIndex].address;
    reading.sensorId = sensorSlots_[sensorIndex].sensorId;
    reading.model = sensorSlots_[sensorIndex].model;

    if (!blocking && !sensor.dataReady())
    {
        reading.status = kLibraryStatusNotReady;
        return reading;
    }

    reading.distanceMm = sensor.read(blocking);
    reading.timeout = sensor.timeoutOccurred();
    if (reading.timeout)
    {
        reading.distanceMm = 0;
        reading.status = kLibraryStatusTimeout;
        return reading;
    }

    reading.status = sensor.ranging_data.range_status;
    reading.dataReady = true;
    reading.valid = reading.status == 0;
    return reading;
}

uint16_t LidarArray::applyLegacyReadBehavior(const LidarReading &reading) const
{
    if (!reading.valid || reading.distanceMm == 0)
    {
        return legacyFallbackDistanceMm_;
    }

    return static_cast<uint16_t>(constrain(reading.distanceMm, legacyMinDistanceMm_, legacyMaxDistanceMm_));
}

void LidarArray::logMessage(LidarDebugLevel level, const __FlashStringHelper *message) const
{
    if (!shouldLog(level))
    {
        return;
    }

    debugConfig_.out->print(F("[LidarArray] "));
    debugConfig_.out->println(message);
}

void LidarArray::logScanResult(uint8_t address, uint8_t errorCode) const
{
    if (debugConfig_.out == nullptr)
    {
        return;
    }

    if (errorCode == 0 && !shouldLog(LidarDebugLevel::Info))
    {
        return;
    }

    if (errorCode != 0 && !shouldLog(LidarDebugLevel::Verbose))
    {
        return;
    }

    debugConfig_.out->print(F("[LidarArray] I2C "));
    if (errorCode == 0)
    {
        debugConfig_.out->print(F("device found at 0x"));
    }
    else
    {
        debugConfig_.out->print(F("bus error at 0x"));
    }

    if (address < 0x10)
    {
        debugConfig_.out->print('0');
    }
    debugConfig_.out->println(address, HEX);
}

void LidarArray::logSensorStep(
    uint8_t sensorIndex,
    const __FlashStringHelper *message,
    LidarDebugLevel level) const
{
    if (!shouldLog(level) || sensorSlots_ == nullptr || sensorIndex >= sensorCount_)
    {
        return;
    }

    const LidarSensorSlot &slot = sensorSlots_[sensorIndex];

    debugConfig_.out->print(F("[LidarArray] Sensor "));
    debugConfig_.out->print(sensorIndex);
    debugConfig_.out->print(F(" (ID "));
    debugConfig_.out->print(slot.sensorId);
    debugConfig_.out->print(F(", PCF "));
    debugConfig_.out->print(slot.pcfIndex);
    debugConfig_.out->print(F(" pin "));
    debugConfig_.out->print(slot.pin);
    debugConfig_.out->print(F(", "));
    debugConfig_.out->print(sensorModelLabel(slot.model));
    debugConfig_.out->print(F(") @ 0x"));
    if (slot.address < 0x10)
    {
        debugConfig_.out->print('0');
    }
    debugConfig_.out->print(slot.address, HEX);
    debugConfig_.out->print(F(": "));
    debugConfig_.out->println(message);
    animatePause();
}

bool LidarArray::shouldLog(LidarDebugLevel level) const
{
    if (debugConfig_.out == nullptr)
    {
        return false;
    }

    return static_cast<uint8_t>(debugConfig_.level) >= static_cast<uint8_t>(level);
}

void LidarArray::animatePause() const
{
    if (!debugConfig_.animated || debugConfig_.animationDelayMs == 0)
    {
        return;
    }

    delay(debugConfig_.animationDelayMs);
}
