#ifndef LIDARARRAY_H
#define LIDARARRAY_H

#include <Arduino.h>
#include <Print.h>
#include <VL53L0X.h>
#include <VL53L4CD.h>
#include <Wire.h>

/**
 * @brief Supported ToF sensor models for a LidarArray instance.
 */
enum class LidarSensorModel : uint8_t
{
    /// Reuse the array default model stored in LidarArrayConfig::model.
    InheritArrayDefault = 0,
    /// Use the Pololu VL53L0X driver.
    VL53L0X = 1,
    /// Use the Pololu VL53L4CD driver.
    VL53L4CD = 2
};

/// Short alias for slots that inherit the array default model.
constexpr LidarSensorModel TOF_INHERIT_DEFAULT = LidarSensorModel::InheritArrayDefault;
/// Short alias for a VL53L0X slot model.
constexpr LidarSensorModel TOF_VL53L0X = LidarSensorModel::VL53L0X;
/// Short alias for a VL53L4CD slot model.
constexpr LidarSensorModel TOF_VL53L4CD = LidarSensorModel::VL53L4CD;

/**
 * @brief Short pin tokens for the eight PCF8574 outputs.
 */
enum LidarPcfPin : uint8_t
{
    pcf_p0 = 0,
    pcf_p1 = 1,
    pcf_p2 = 2,
    pcf_p3 = 3,
    pcf_p4 = 4,
    pcf_p5 = 5,
    pcf_p6 = 6,
    pcf_p7 = 7
};

#ifndef LIDAR_SLOT
#define LIDAR_SLOT(pcf, pin, model, id) \
    {static_cast<uint8_t>(pcf), static_cast<uint8_t>(pin), static_cast<uint8_t>(0), static_cast<int16_t>(id), model}
#endif

#ifndef LIDAR_SLOT_ADDR
#define LIDAR_SLOT_ADDR(pcf, pin, addr, model, id) \
    {static_cast<uint8_t>(pcf), static_cast<uint8_t>(pin), static_cast<uint8_t>(addr), static_cast<int16_t>(id), model}
#endif

/**
 * @brief Verbosity levels used by the built-in debug output.
 */
enum class LidarDebugLevel : uint8_t
{
    /// Disable library debug messages.
    Off,
    /// Show only error messages.
    Errors,
    /// Show high-level progress messages.
    Info,
    /// Show detailed scan and initialization messages.
    Verbose
};

/**
 * @brief Describes the physical and logical placement of one sensor.
 * @note Slot literal order is {pcfIndex, pin, address, sensorId, model}.
 * @code
 * const LidarSensorSlot sensorMap[] = {
 *     LIDAR_SLOT(0, pcf_p3, TOF_VL53L4CD, 10),
 *     {1, pcf_p4, 0x36, -1, TOF_VL53L0X},
 *     {1, pcf_p5, 0, 11} // legacy 4-field form still works and inherits config.model
 * };
 * @endcode
 */
struct LidarSensorSlot
{
    /// Index of the PCF8574 that controls this sensor XSHUT line.
    uint8_t pcfIndex = 0;
    /// Physical PCF8574 pin connected to XSHUT, always in the range 0..7.
    uint8_t pin = 0;
    /// Final I2C address to assign after boot. Use 0 for automatic addressing.
    uint8_t address = 0;
    /// Public logical ID exposed by the library. Use -1 to reuse the internal index.
    int16_t sensorId = -1;
    /// Sensor model used by this slot. Use InheritArrayDefault to reuse LidarArrayConfig::model.
    LidarSensorModel model = LidarSensorModel::InheritArrayDefault;
};

/**
 * @brief Detailed result returned by the structured reading API.
 */
struct LidarReading
{
    /// Measured distance in millimeters.
    uint16_t distanceMm = 0;
    /// Raw sensor status or a library-defined availability status.
    uint8_t status = 0xFF;
    /// True when the reading can be used as a valid measurement.
    bool valid = false;
    /// True when the underlying driver reported a timeout.
    bool timeout = false;
    /// True when the sensor had data ready for this reading.
    bool dataReady = false;
    /// Final I2C address assigned to the sensor.
    uint8_t address = 0;
    /// Public logical ID associated with the sensor that produced this reading.
    int16_t sensorId = -1;
    /// Resolved sensor model associated with this reading.
    LidarSensorModel model = LidarSensorModel::InheritArrayDefault;
};

/**
 * @brief Configures the optional per-sensor filtering pipeline.
 */
struct LidarFilterConfig
{
    /// Enable the internal filter pipeline for filtered reads.
    bool enabled = false;
    /// Median window size. Supported values are 1, 3, or 5. A value of 1 disables median filtering.
    uint8_t medianWindow = 3;
    /// EMA alpha in percent. Use 0 to disable EMA and 100 to strongly favor the newest sample.
    uint8_t emaAlphaPercent = 50;
    /// Keep the last valid filtered value when the next raw read is invalid.
    bool holdLastValid = true;
    /// Maximum number of consecutive held reads. Use 0 to disable hold behavior.
    uint8_t maxHeldReads = 2;
    /// Reject changes larger than this threshold in millimeters. Use 0 to disable jump rejection.
    uint16_t maxJumpMm = 0;
    /// Maximum number of consecutive samples the max-jump rule may reject before the
    /// filter accepts the new distance level and re-acquires it.
    /// A value of 0 is coerced to 1 while jump rejection is active, so the filter can
    /// never stay locked on a stale value after the real distance changes.
    /// Keep this less than or equal to maxHeldReads to avoid invalid outputs during the transition.
    uint8_t maxJumpRejections = 2;

    /**
     * @brief Build a disabled filter configuration.
     * @return A configuration that bypasses the filter pipeline.
     */
    static LidarFilterConfig disabled();
    /**
     * @brief Build the recommended starting filter configuration.
     * @return Enabled median(3) + EMA(50%) + hold-last-valid defaults.
     * @note Jump rejection stays disabled. Set maxJumpMm to enable it.
     */
    static LidarFilterConfig recommended();
};

/**
 * @brief Result returned by the filtered reading API.
 */
struct LidarFilteredReading
{
    /// Raw reading produced by the sensor before filtering.
    LidarReading raw;
    /// Final filtered or held output distance in millimeters.
    uint16_t distanceMm = 0;
    /// True when the final filtered output is valid for control use.
    bool valid = false;
    /// True when the filter pipeline was enabled for this read.
    bool filterApplied = false;
    /// True when the library returned the previous valid filtered value.
    bool heldLastValid = false;
    /// True when the current sample was rejected by the max-jump rule.
    bool jumpRejected = false;
    /// True when the filter re-acquired a new distance level after repeated jump rejections.
    bool jumpResynced = false;
};

/**
 * @brief User-facing configuration for a LidarArray instance.
 */
struct LidarArrayConfig
{
    /// Default sensor model used by the array and by slots that inherit their model.
    LidarSensorModel model = LidarSensorModel::VL53L0X;
    /// Number of PCF8574 expanders used to drive XSHUT lines.
    uint8_t numPCF = 0;
    /// Number of active sensors managed by the array.
    uint8_t numSensors = 0;
    /// I2C address list for the PCF8574 expanders.
    const uint8_t *pcf8574Addresses = nullptr;
    /// Legacy dense XSHUT map. The order of this array defines the logical sensor order.
    const uint8_t *xshutPins = nullptr;
    /// I2C bus used by the library. The sketch must still call Wire.begin().
    TwoWire *wire = &Wire;
    /// Timeout forwarded to the Pololu sensor drivers.
    /// A value of 0 makes the drivers block forever on an unresponsive sensor, so the
    /// library replaces 0 with a timeout derived from the configured measurement period.
    uint16_t timeoutMs = 100;
    /// Delay after forcing all sensors into shutdown.
    uint16_t shutdownDelayMs = 5;
    /// Delay after enabling one sensor before initialization.
    uint16_t wakeDelayMs = 5;
    /// Advanced legacy VL53L0X compatibility options.
    bool legacyReadBehavior = false;
    uint16_t legacyMinDistanceMm = 10;
    uint16_t legacyMaxDistanceMm = 700;
    uint16_t legacyFallbackDistanceMm = 700;
    uint32_t vl53l0xMeasurementTimingBudgetUs = 20000;
    bool applyVl53l0xVcselPeriods = false;
    uint8_t vl53l0xPreRangeVcselPeriod = 14;
    uint8_t vl53l0xFinalRangeVcselPeriod = 10;
    /// Advanced VL53L4CD timing options.
    uint8_t vl53l4cdTimingBudgetMs = 50;
    uint32_t vl53l4cdInterMeasurementMs = 0;
    /// Optional final address per sensor for the legacy dense layout.
    const uint8_t *sensorAddresses = nullptr;
    /// Recommended sparse layout mapping for mixed PCF/pin placement.
    /// Each item uses {pcfIndex, pin, address, sensorId, model}. Do not combine this with sensorAddresses.
    const LidarSensorSlot *sensorMap = nullptr;

    /**
     * @brief Build a configuration object filled with defaults for the selected model.
     * @param model Sensor model used by the array.
     * @return A configuration initialized with safe default values.
     */
    static LidarArrayConfig defaults(LidarSensorModel model = LidarSensorModel::VL53L0X);
    /**
     * @brief Build a complete configuration using the legacy dense XSHUT map.
     * @param model Sensor model used by the array.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param xshutPins Dense XSHUT map in logical sensor order.
     * @param wire I2C bus instance used by the library.
     * @param sensorAddresses Optional final address list for the dense layout.
     * @return A populated configuration for the selected model.
     */
    static LidarArrayConfig forModel(
        LidarSensorModel model,
        uint8_t numPCF,
        uint8_t numSensors,
        const uint8_t *pcf8574Addresses,
        const uint8_t *xshutPins,
        TwoWire *wire = &Wire,
        const uint8_t *sensorAddresses = nullptr);
    /**
     * @brief Build a complete configuration using the sparse sensor map API.
     * @param model Sensor model used by the array.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param sensorMap Sparse physical mapping for the active sensors.
     * Each item uses {pcfIndex, pin, address, sensorId, model}.
     * @param wire I2C bus instance used by the library.
     * @return A populated configuration for the selected model.
     * @note When sensorMap is used, define manual addresses inside each slot instead of using sensorAddresses.
     */
    static LidarArrayConfig forModel(
        LidarSensorModel model,
        uint8_t numPCF,
        uint8_t numSensors,
        const uint8_t *pcf8574Addresses,
        const LidarSensorSlot *sensorMap,
        TwoWire *wire = &Wire);
    /**
     * @brief Build a VL53L0X configuration using the legacy dense XSHUT map.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param xshutPins Dense XSHUT map in logical sensor order.
     * @param wire I2C bus instance used by the library.
     * @param sensorAddresses Optional final address list for the dense layout.
     * @return A VL53L0X configuration object.
     */
    static LidarArrayConfig forVL53L0X(
        uint8_t numPCF,
        uint8_t numSensors,
        const uint8_t *pcf8574Addresses,
        const uint8_t *xshutPins,
        TwoWire *wire = &Wire,
        const uint8_t *sensorAddresses = nullptr);
    /**
     * @brief Build a VL53L0X configuration using the sparse sensor map API.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param sensorMap Sparse physical mapping for the active sensors.
     * Each item uses {pcfIndex, pin, address, sensorId, model}.
     * @param wire I2C bus instance used by the library.
     * @return A VL53L0X configuration object.
     */
    static LidarArrayConfig forVL53L0X(
        uint8_t numPCF,
        uint8_t numSensors,
        const uint8_t *pcf8574Addresses,
        const LidarSensorSlot *sensorMap,
        TwoWire *wire = &Wire);
    /**
     * @brief Build a VL53L4CD configuration using the legacy dense XSHUT map.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param xshutPins Dense XSHUT map in logical sensor order.
     * @param wire I2C bus instance used by the library.
     * @param sensorAddresses Optional final address list for the dense layout.
     * @return A VL53L4CD configuration object.
     */
    static LidarArrayConfig forVL53L4CD(
        uint8_t numPCF,
        uint8_t numSensors,
        const uint8_t *pcf8574Addresses,
        const uint8_t *xshutPins,
        TwoWire *wire = &Wire,
        const uint8_t *sensorAddresses = nullptr);
    /**
     * @brief Build a VL53L4CD configuration using the sparse sensor map API.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param sensorMap Sparse physical mapping for the active sensors.
     * Each item uses {pcfIndex, pin, address, sensorId, model}.
     * @param wire I2C bus instance used by the library.
     * @return A VL53L4CD configuration object.
     */
    static LidarArrayConfig forVL53L4CD(
        uint8_t numPCF,
        uint8_t numSensors,
        const uint8_t *pcf8574Addresses,
        const LidarSensorSlot *sensorMap,
        TwoWire *wire = &Wire);
};

/**
 * @brief Debug output options used during initialization and I2C scanning.
 */
struct LidarArrayDebugConfig
{
    /// Output stream used for debug messages. Set to nullptr to disable output.
    Print *out = nullptr;
    /// Minimum debug verbosity emitted by the library.
    LidarDebugLevel level = LidarDebugLevel::Off;
    /// Run an I2C scan before the library changes sensor states.
    bool scanBeforeInit = false;
    /// Run an I2C scan after each sensor initialization step.
    bool scanEachStep = false;
    /// Enable delays between debug steps for easier visual tracking.
    bool animated = false;
    /// Delay used between animated debug steps.
    uint16_t animationDelayMs = 40;
    /// Delay before the initialization sequence starts.
    uint16_t bootDelayMs = 0;

    /**
     * @brief Build a verbose debug configuration in one call.
     * @param out Output stream used for debug messages.
     * @param scanBeforeInit Enable a scan before sensor shutdown.
     * @param scanEachStep Enable a scan after each sensor step.
     * @param animated Enable pauses between debug steps.
     * @param animationDelayMs Delay used between animated debug steps.
     * @param bootDelayMs Delay before the initialization sequence starts.
     * @return A ready-to-use verbose debug configuration.
     */
    static LidarArrayDebugConfig verbose(
        Print *out,
        bool scanBeforeInit = false,
        bool scanEachStep = false,
        bool animated = false,
        uint16_t animationDelayMs = 40,
        uint16_t bootDelayMs = 0);
};

/**
 * @brief Manages a group of VL53L0X and/or VL53L4CD sensors connected through PCF8574 XSHUT control.
 */
class LidarArray
{
public:
    /**
     * @brief Create an array instance using default settings for the selected model.
     * @param model Sensor model handled by this instance.
     */
    explicit LidarArray(LidarSensorModel model = LidarSensorModel::VL53L0X);
    /**
     * @brief Create an array instance from a prepared configuration object.
     * @param config Full library configuration.
     */
    explicit LidarArray(const LidarArrayConfig &config);
    /**
     * @brief Create a legacy dense VL53L0X array using the original constructor shape.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param xshutPins Dense XSHUT map in row-major order.
     * @note This constructor is a compatibility path for legacy dense VL53L0X layouts.
     */
    LidarArray(uint8_t numPCF, uint8_t numSensors, const uint8_t pcf8574Addresses[], const uint8_t xshutPins[][8]);
    /**
     * @brief Release all internal buffers owned by the library.
     */
    ~LidarArray();

    LidarArray(const LidarArray &) = delete;
    LidarArray &operator=(const LidarArray &) = delete;

    /**
     * @brief Initialize the configured sensor array.
     * @return True when all configured sensors were initialized successfully.
     */
    bool begin();

    /**
     * @brief Start the array using the legacy initialization path and current defaults.
     * @return True when all configured sensors were initialized successfully.
     * @note This is a compatibility wrapper around begin().
     */
    bool initSensors();
    /**
     * @brief Start the legacy VL53L0X path with a custom timing budget.
     * @param measurementT VL53L0X timing budget in microseconds.
     * @return True when all configured sensors were initialized successfully.
     * @note This is a compatibility wrapper for dense VL53L0X setups.
     */
    bool initSensors(int measurementT);
    /**
     * @brief Start the legacy VL53L0X path with custom timing and VCSEL periods.
     * @param measurementT VL53L0X timing budget in microseconds.
     * @param preRange Pre-range VCSEL period.
     * @param finalRange Final-range VCSEL period.
     * @return True when all configured sensors were initialized successfully.
     * @note This is a compatibility wrapper for dense VL53L0X setups.
     */
    bool initSensors(int measurementT, uint8_t preRange, uint8_t finalRange);
    /**
     * @brief Start the legacy VL53L0X path with custom timing, VCSEL periods, and timeout.
     * @param measurementT VL53L0X timing budget in microseconds.
     * @param preRange Pre-range VCSEL period.
     * @param finalRange Final-range VCSEL period.
     * @param timeout Driver timeout in milliseconds.
     * @return True when all configured sensors were initialized successfully.
     * @note This is a compatibility wrapper for dense VL53L0X setups.
     */
    bool initSensors(int measurementT, uint8_t preRange, uint8_t finalRange, int timeout);

    /**
     * @brief Access the mutable configuration object used by begin().
     * @return Reference to the live configuration object.
     */
    LidarArrayConfig &config();
    /**
     * @brief Access the current configuration object.
     * @return Read-only reference to the live configuration object.
     */
    const LidarArrayConfig &config() const;
    /**
     * @brief Access the mutable debug configuration used by begin().
     * @return Reference to the live debug configuration object.
     */
    LidarArrayDebugConfig &debug();
    /**
     * @brief Access the current debug configuration.
     * @return Read-only reference to the live debug configuration object.
     */
    const LidarArrayDebugConfig &debug() const;
    /**
     * @brief Access the mutable global filter configuration used by filtered reads.
     * @return Reference to the global filter configuration.
     * @note If you change this after the array is already running, call resetFilters() to clear old filter state.
     */
    LidarFilterConfig &filter();
    /**
     * @brief Access the current global filter configuration used by filtered reads.
     * @return Read-only reference to the global filter configuration.
     */
    const LidarFilterConfig &filter() const;

    /**
     * @brief Define a legacy dense sensor layout using the XSHUT order array.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param xshutPins Dense XSHUT map in logical sensor order.
     */
    void setLayout(uint8_t numPCF, uint8_t numSensors, const uint8_t *pcf8574Addresses, const uint8_t *xshutPins);
    /**
     * @brief Define a sparse sensor layout using explicit PCF and pin placement.
     * @param numPCF Number of PCF8574 expanders.
     * @param numSensors Number of active sensors.
     * @param pcf8574Addresses I2C addresses for the PCF8574 expanders.
     * @param sensorMap Sparse physical mapping for the active sensors.
     * Each item uses {pcfIndex, pin, address, sensorId, model}.
     * @note This is the recommended API for non-dense layouts across multiple PCF8574 devices.
     */
    void setLayout(uint8_t numPCF, uint8_t numSensors, const uint8_t *pcf8574Addresses, const LidarSensorSlot *sensorMap);
    /**
     * @brief Set manual final addresses for the legacy dense layout.
     * @param sensorAddresses Array with one address per active sensor, or nullptr to restore automatic addressing.
     * @note Do not combine this API with sensorMap.
     */
    void setSensorAddresses(const uint8_t *sensorAddresses);
    /**
     * @brief Select the I2C bus used by the library.
     * @param wire Bus instance that will be used during initialization and reads.
     */
    void setWire(TwoWire *wire);
    /**
     * @brief Set the timeout forwarded to the underlying sensor driver.
     * @param timeoutMs Timeout in milliseconds.
     */
    void setTimeout(uint16_t timeoutMs);
    /**
     * @brief Configure VL53L4CD timing parameters.
     * @param timingBudgetMs Ranging timing budget in milliseconds.
     * @param interMeasurementMs Delay between measurements in milliseconds.
     */
    void setVL53L4CDTiming(uint8_t timingBudgetMs, uint32_t interMeasurementMs = 0);
    /**
     * @brief Replace the global filter configuration used by filtered reads.
     * @param config New global filter configuration.
     * @note This resets all filter state.
     */
    void setFilterConfig(const LidarFilterConfig &config);
    /**
     * @brief Set a per-sensor filter override.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @param config Filter configuration to use for that sensor.
     * @note This resets only the selected sensor filter state.
     */
    void setSensorFilterConfig(uint8_t sensorIndex, const LidarFilterConfig &config);
    /**
     * @brief Remove the filter override from one sensor.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @note This resets only the selected sensor filter state.
     */
    void clearSensorFilterConfig(uint8_t sensorIndex);
    /**
     * @brief Remove all per-sensor filter overrides.
     * @note This resets all filter state.
     */
    void clearSensorFilterConfigs();
    /**
     * @brief Check whether one sensor has a filter override.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return True when a per-sensor filter override is present.
     */
    bool hasSensorFilterConfig(uint8_t sensorIndex) const;
    /**
     * @brief Return the effective filter configuration for one sensor.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Sanitized per-sensor config when present, otherwise the sanitized global config.
     */
    LidarFilterConfig getEffectiveFilterConfig(uint8_t sensorIndex) const;
    /**
     * @brief Reset the filter state of one sensor.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     */
    void resetFilter(uint8_t sensorIndex);
    /**
     * @brief Reset the filter state of all sensors.
     */
    void resetFilters();

    /**
     * @brief Read one sensor by internal index and return only the distance.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Distance in millimeters, optionally filtered by legacy compatibility behavior.
     * @note Legacy mode may clamp invalid readings to the configured fallback range.
     */
    uint16_t readSensor(uint8_t sensorIndex);
    /**
     * @brief Perform a non-blocking distance read by internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Distance in millimeters, optionally filtered by legacy compatibility behavior.
     * @note Legacy mode may clamp invalid readings to the configured fallback range.
     */
    uint16_t readSensorNB(uint8_t sensorIndex);
    /**
     * @brief Read one sensor by public sensor ID and return only the distance.
     * @param sensorId Public logical sensor ID.
     * @return Distance in millimeters, optionally filtered by legacy compatibility behavior.
     */
    uint16_t readSensorById(int16_t sensorId);
    /**
     * @brief Perform a non-blocking distance read by public sensor ID.
     * @param sensorId Public logical sensor ID.
     * @return Distance in millimeters, optionally filtered by legacy compatibility behavior.
     */
    uint16_t readSensorNBById(int16_t sensorId);
    /**
     * @brief Read one sensor by internal index and return the full structured result.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @param blocking True for blocking behavior, false for a non-blocking attempt.
     * @return Detailed measurement and availability information.
     */
    LidarReading readReading(uint8_t sensorIndex, bool blocking = true);
    /**
     * @brief Read one sensor by public sensor ID and return the full structured result.
     * @param sensorId Public logical sensor ID.
     * @param blocking True for blocking behavior, false for a non-blocking attempt.
     * @return Detailed measurement and availability information.
     */
    LidarReading readReadingById(int16_t sensorId, bool blocking = true);
    /**
     * @brief Read one sensor by internal index using the filter pipeline.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Final filtered or held distance in millimeters.
     */
    uint16_t readFilteredSensor(uint8_t sensorIndex);
    /**
     * @brief Perform a non-blocking filtered distance read by internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Final filtered or held distance in millimeters.
     */
    uint16_t readFilteredSensorNB(uint8_t sensorIndex);
    /**
     * @brief Read one sensor by public sensor ID using the filter pipeline.
     * @param sensorId Public logical sensor ID.
     * @return Final filtered or held distance in millimeters.
     */
    uint16_t readFilteredSensorById(int16_t sensorId);
    /**
     * @brief Perform a non-blocking filtered distance read by public sensor ID.
     * @param sensorId Public logical sensor ID.
     * @return Final filtered or held distance in millimeters.
     */
    uint16_t readFilteredSensorNBById(int16_t sensorId);
    /**
     * @brief Read one sensor by internal index and return the filtered result.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @param blocking True for blocking behavior, false for a non-blocking attempt.
     * @return Filtered result containing the raw and final output values.
     */
    LidarFilteredReading readFilteredReading(uint8_t sensorIndex, bool blocking = true);
    /**
     * @brief Read one sensor by public sensor ID and return the filtered result.
     * @param sensorId Public logical sensor ID.
     * @param blocking True for blocking behavior, false for a non-blocking attempt.
     * @return Filtered result containing the raw and final output values.
     */
    LidarFilteredReading readFilteredReadingById(int16_t sensorId, bool blocking = true);

    /**
     * @brief Access a VL53L0X driver instance by internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Reference to the VL53L0X driver object.
     * @note This method exists for VL53L0X compatibility. Prefer the structured APIs for model-independent code.
     */
    VL53L0X &getSensor(uint8_t sensorIndex);
    /**
     * @brief Access a VL53L0X driver pointer by internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Pointer to the VL53L0X driver, or nullptr when the slot uses another model.
     */
    VL53L0X *getVL53L0XSensor(uint8_t sensorIndex);
    /**
     * @brief Access a VL53L4CD driver pointer by internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Pointer to the VL53L4CD driver, or nullptr when the slot uses another model.
     */
    VL53L4CD *getVL53L4CDSensor(uint8_t sensorIndex);

    /**
     * @brief Update the VL53L0X timing budget for ready sensors and future initialization.
     * @param timingBudget Timing budget in microseconds.
     */
    void setMeasurementTimingBudget(uint32_t timingBudget);
    /**
     * @brief Update a VL53L0X VCSEL period for ready sensors and future initialization.
     * @param type Use 0 for pre-range and 1 for final-range.
     * @param period VCSEL period value accepted by the driver.
     */
    void setVcselPulsePeriod(uint8_t type, uint8_t period);
    /**
     * @brief Replace the entire debug configuration.
     * @param config New debug configuration.
     */
    void setDebugConfig(const LidarArrayDebugConfig &config);
    /**
     * @brief Set the debug output stream.
     * @param out Output stream used for debug messages.
     */
    void setDebugOutput(Print *out);
    /**
     * @brief Set the minimum debug verbosity emitted by the library.
     * @param level New minimum verbosity level.
     */
    void setDebugLevel(LidarDebugLevel level);
    /**
     * @brief Enable or disable the scan performed before shutdown.
     * @param enabled True to enable the pre-init scan.
     */
    void setDebugScanBeforeInit(bool enabled);
    /**
     * @brief Enable or disable the scan performed after each sensor step.
     * @param enabled True to enable per-step scans.
     */
    void setDebugScanEachStep(bool enabled);
    /**
     * @brief Set a delay before initialization starts.
     * @param delayMs Delay in milliseconds.
     */
    void setDebugBootDelay(uint16_t delayMs);
    /**
     * @brief Set the delay used between animated debug steps.
     * @param delayMs Delay in milliseconds. Zero disables animation.
     */
    void setDebugStepDelay(uint16_t delayMs);

    /**
     * @brief Return the number of sensors configured for this array.
     * @return Configured sensor count.
     */
    uint8_t getSensorCount() const;
    /**
     * @brief Return how many sensors were initialized successfully.
     * @return Number of sensors marked ready after begin().
     */
    uint8_t getInitializedSensorCount() const;
    /**
     * @brief Report whether one sensor finished initialization successfully.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return True when the sensor is ready for reads.
     */
    bool isSensorReady(uint8_t sensorIndex) const;
    /**
     * @brief Return the public logical ID associated with one internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Public sensor ID, or -1 when the index is invalid.
     */
    int16_t getSensorId(uint8_t sensorIndex) const;
    /**
     * @brief Return the resolved sensor model associated with one internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return The resolved slot model, or InheritArrayDefault when the index is invalid.
     */
    LidarSensorModel getSensorModel(uint8_t sensorIndex) const;
    /**
     * @brief Find the internal index associated with one public sensor ID.
     * @param sensorId Public logical sensor ID.
     * @return Internal sensor index, or -1 when the ID is not present.
     */
    int16_t indexOfSensorId(int16_t sensorId) const;
    /**
     * @brief Return the resolved sensor model associated with one public sensor ID.
     * @param sensorId Public logical sensor ID.
     * @return The resolved slot model, or InheritArrayDefault when the ID is not present.
     */
    LidarSensorModel getSensorModelById(int16_t sensorId) const;
    /**
     * @brief Return the sparse slot description associated with one internal index.
     * @param sensorIndex Internal sensor index in the range 0..N-1.
     * @return Pointer to the slot description, or nullptr when the index is invalid.
     */
    const LidarSensorSlot *getSensorSlot(uint8_t sensorIndex) const;
    /**
     * @brief Scan the configured I2C bus and report the number of detected devices.
     * @return Number of I2C devices found on the bus.
     */
    uint8_t scanI2C();

private:
    static constexpr uint8_t kSlotsPerPcf = 8;
    static constexpr uint8_t kMaxFilterWindow = 5;
    static constexpr uint8_t kTofDefaultAddress = 0x29;
    static constexpr uint8_t kDefaultAddressBase = 0x30;
    static constexpr uint8_t kMinimumManualAddress = 0x08;
    static constexpr uint8_t kLibraryStatusInvalidIndex = 0xFD;
    static constexpr uint8_t kLibraryStatusNotReady = 0xFE;
    static constexpr uint8_t kLibraryStatusTimeout = 0xFF;
    static constexpr uint16_t kMinimumAutoTimeoutMs = 100;
    static constexpr uint16_t kMaximumAutoTimeoutMs = 1000;

    struct SensorFilterState
    {
        uint16_t samples[kMaxFilterWindow] = {0, 0, 0, 0, 0};
        uint8_t sampleCount = 0;
        uint8_t sampleHead = 0;
        uint16_t lastFilteredValue = 0;
        bool hasLastFilteredValue = false;
        uint8_t heldReads = 0;
        uint8_t rejectedReads = 0;
    };

    uint8_t numPCF_;
    uint8_t sensorCount_;
    uint8_t totalSlots_;
    uint8_t initializedSensorCount_;
    bool configCopied_;
    bool configValid_;
    bool explicitSensorMap_;
    uint8_t filterStorageCount_;
    uint8_t vl53l0xSensorCount_;
    uint8_t vl53l4cdSensorCount_;

    LidarSensorModel model_;
    TwoWire *wire_;

    uint16_t timeoutMs_;
    uint16_t shutdownDelayMs_;
    uint16_t wakeDelayMs_;

    bool legacyReadBehavior_;
    uint16_t legacyMinDistanceMm_;
    uint16_t legacyMaxDistanceMm_;
    uint16_t legacyFallbackDistanceMm_;

    uint32_t vl53l0xMeasurementTimingBudgetUs_;
    bool applyVl53l0xVcselPeriods_;
    uint8_t vl53l0xPreRangeVcselPeriod_;
    uint8_t vl53l0xFinalRangeVcselPeriod_;

    uint8_t vl53l4cdTimingBudgetMs_;
    uint32_t vl53l4cdInterMeasurementMs_;

    LidarArrayDebugConfig debugConfig_;
    LidarFilterConfig filterConfig_;

    uint8_t *pcf8574Addresses_;
    uint8_t *pcf8574States_;
    uint8_t *sensorDriverIndices_;
    LidarSensorSlot *sensorSlots_;
    bool *sensorReady_;
    LidarFilterConfig *sensorFilterConfigs_;
    bool *sensorFilterOverrides_;
    SensorFilterState *filterStates_;

    VL53L0X *vl53l0xSensors_;
    VL53L4CD *vl53l4cdSensors_;

    LidarArrayConfig config_;

    void resetMembers();
    bool allocateStorage(uint8_t numPCF, uint8_t numSensors, uint8_t vl53l0xSensorCount, uint8_t vl53l4cdSensorCount);
    void releaseStorage();
    bool ensureFilterStorage(uint8_t numSensors);
    void releaseFilterStorage();
    void copyConfig(const LidarArrayConfig &config);
    bool copyLegacyLayout(const LidarArrayConfig &config);
    bool copySparseLayout(const LidarArrayConfig &config);
    bool syncConfiguration();
    bool validateConfiguration() const;

    bool shutdownAllSensors();
    bool bringSensorOutOfShutdown(uint8_t sensorIndex);
    bool initializeSensor(uint8_t sensorIndex);

    uint16_t resolveEffectiveTimeoutMs() const;

    bool initializeVL53L0X(uint8_t sensorIndex, uint8_t address);
    bool initializeVL53L4CD(uint8_t sensorIndex, uint8_t address);

    bool pcf8574Write(uint8_t pcf8574Index, uint8_t pin, bool state);
    bool writePcf8574State(uint8_t pcf8574Index);

    LidarReading buildInvalidReading(uint8_t sensorIndex, uint8_t status) const;
    LidarReading buildInvalidReadingById(int16_t sensorId, uint8_t status) const;
    LidarFilteredReading buildInvalidFilteredReading(uint8_t sensorIndex, uint8_t status) const;
    LidarFilteredReading buildInvalidFilteredReadingById(int16_t sensorId, uint8_t status) const;
    LidarReading readVL53L0X(uint8_t sensorIndex, bool blocking);
    LidarReading readVL53L4CD(uint8_t sensorIndex, bool blocking);
    uint16_t applyLegacyReadBehavior(const LidarReading &reading) const;
    static LidarFilterConfig sanitizeFilterConfig(const LidarFilterConfig &config);
    void resetFilterState(uint8_t sensorIndex);
    void resyncFilterState(uint8_t sensorIndex, uint16_t value);
    uint16_t medianFromSamples(const uint16_t *samples, uint8_t count) const;
    uint16_t computeMedianCandidate(uint8_t sensorIndex, uint16_t sample, uint8_t window) const;
    void appendValidFilterSample(uint8_t sensorIndex, uint16_t sample);
    uint16_t applyEma(uint16_t previousValue, uint16_t sample, uint8_t alphaPercent) const;
    LidarFilteredReading applyFilterPipeline(uint8_t sensorIndex, const LidarReading &raw);

    void logMessage(LidarDebugLevel level, const __FlashStringHelper *message) const;
    void logScanResult(uint8_t address, uint8_t errorCode) const;
    void logSensorStep(uint8_t sensorIndex, const __FlashStringHelper *message, LidarDebugLevel level) const;
    bool shouldLog(LidarDebugLevel level) const;
    void animatePause() const;
};

#endif
