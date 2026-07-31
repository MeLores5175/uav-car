#include "stepper_driver.h"
#include "mission_config.h"

#include <math.h>

#if defined(ESP32) && __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

namespace {

constexpr uint8_t kLeftLedcChannel = 0;
// 通道0和1可能映射/复用同一LEDC timer；右轮改用通道2。
constexpr uint8_t kRightLedcChannel = 2;
constexpr uint8_t kLedcResolutionBits = 10;
// 两路必须以不同频率挂接，避免Arduino-ESP32 3.x将它们自动合并到同一timer。
constexpr uint32_t kLeftInitialLedcFrequencyHz = 997;
constexpr uint32_t kRightInitialLedcFrequencyHz = 1009;
constexpr uint32_t kLedcRunDuty =
    1UL << (kLedcResolutionBits - 1);
constexpr float kStopSpeedCmS = 0.001f;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
constexpr bool kUseLedcPinApi = true;
#else
constexpr bool kUseLedcPinApi = false;
#endif

struct MotorPins {
    uint8_t step;
    uint8_t dir;
    uint8_t en;
    uint8_t channel;
    uint8_t forwardDirLevel;
    uint32_t initialFrequencyHz;
};

constexpr MotorPins kLeftPins{
    CarConfig::LEFT_STP,
    CarConfig::LEFT_DIR,
    CarConfig::LEFT_EN,
    kLeftLedcChannel,
    CarConfig::LEFT_FORWARD_DIR_LEVEL,
    kLeftInitialLedcFrequencyHz
};

constexpr MotorPins kRightPins{
    CarConfig::RIGHT_STP,
    CarConfig::RIGHT_DIR,
    CarConfig::RIGHT_EN,
    kRightLedcChannel,
    CarConfig::RIGHT_FORWARD_DIR_LEVEL,
    kRightInitialLedcFrequencyHz
};

bool stepperConfigValid()
{
    return CarConfig::WHEEL_DIAMETER_CM > 0.0f &&
           CarConfig::STEPPER_PULSES_PER_REV > 0.0f;
}

void refreshPulseGeneratorReady(StepperDriverStatus& status)
{
    status.pulseGeneratorReady =
        status.left.pulseGeneratorAttached &&
        status.right.pulseGeneratorAttached;
}

float wheelCircumferenceCm()
{
    return PI * CarConfig::WHEEL_DIAMETER_CM;
}

uint32_t speedToPulseFrequencyHz(float speedCmS)
{
    if (!stepperConfigValid() || fabsf(speedCmS) < kStopSpeedCmS) {
        return 0;
    }

    const float rawFrequency =
        fabsf(speedCmS) /
        wheelCircumferenceCm() *
        CarConfig::STEPPER_PULSES_PER_REV;

    if (!(rawFrequency > 0.0f)) {
        return 0;
    }
    if (rawFrequency >= static_cast<float>(CarConfig::STEPPER_MAX_PULSE_HZ)) {
        return CarConfig::STEPPER_MAX_PULSE_HZ;
    }

    const uint32_t roundedFrequency =
        static_cast<uint32_t>(rawFrequency + 0.5f);
    return roundedFrequency == 0 ? 1 : roundedFrequency;
}

float pulseFrequencyToSpeedCmS(uint32_t pulseFrequencyHz, bool forward)
{
    if (!stepperConfigValid() || pulseFrequencyHz == 0) {
        return 0.0f;
    }

    const float speed =
        static_cast<float>(pulseFrequencyHz) *
        wheelCircumferenceCm() /
        CarConfig::STEPPER_PULSES_PER_REV;
    return forward ? speed : -speed;
}

void writeEnable(const MotorPins& pins, bool enable)
{
    digitalWrite(
        pins.en,
        enable
            ? CarConfig::STEPPER_EN_ENABLE_LEVEL
            : CarConfig::STEPPER_EN_DISABLE_LEVEL);
}

void writeDirection(const MotorPins& pins, bool forward)
{
    const uint8_t reverseDirLevel =
        pins.forwardDirLevel == HIGH ? LOW : HIGH;

    digitalWrite(
        pins.dir,
        forward ? pins.forwardDirLevel : reverseDirLevel);
    delayMicroseconds(20);
}

void stopPulseOutput(const MotorPins& pins)
{
#if defined(ESP32)
    if (kUseLedcPinApi) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(pins.step, 0);
#endif
    } else {
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
        ledcWrite(pins.channel, 0);
#endif
    }
#else
    digitalWrite(pins.step, LOW);
#endif
}

bool attachPulseOutput(const MotorPins& pins)
{
#if defined(ESP32)
    if (kUseLedcPinApi) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        return ledcAttachChannel(
            pins.step,
            pins.initialFrequencyHz,
            kLedcResolutionBits,
            pins.channel);
#else
        return false;
#endif
    }

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
    ledcSetup(pins.channel, pins.initialFrequencyHz, kLedcResolutionBits);
    ledcAttachPin(pins.step, pins.channel);
    stopPulseOutput(pins);
    return true;
#else
    return false;
#endif
#else
    (void)pins;
    return false;
#endif
}

void runPulseOutput(const MotorPins& pins, uint32_t pulseFrequencyHz)
{
    if (pulseFrequencyHz == 0) {
        stopPulseOutput(pins);
        return;
    }

#if defined(ESP32)
    if (kUseLedcPinApi) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcChangeFrequency(
            pins.step,
            pulseFrequencyHz,
            kLedcResolutionBits);
        ledcWrite(pins.step, kLedcRunDuty);
#endif
    } else {
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
        ledcSetup(pins.channel, pulseFrequencyHz, kLedcResolutionBits);
        ledcWrite(pins.channel, kLedcRunDuty);
#endif
    }
#else
    (void)pins;
    (void)pulseFrequencyHz;
#endif
}

void initializeMotorPins(const MotorPins& pins)
{
    pinMode(pins.step, OUTPUT);
    pinMode(pins.dir, OUTPUT);
    pinMode(pins.en, OUTPUT);

    digitalWrite(pins.step, LOW);
    writeDirection(pins, true);
    writeEnable(pins, false);
}

void markPulseStopped(StepperMotorStatus& status)
{
    status.pulseFrequencyHz = 0;
    status.pulseOutputRunning = false;
}

void resetMotorStatus(StepperMotorStatus& status)
{
    status.targetSpeedCmS = 0.0f;
    status.forward = true;
    status.enabled = false;
    markPulseStopped(status);
}

float updateMotor(const MotorPins& pins,
                  StepperMotorStatus& status,
                  float targetSpeedCmS)
{
    status.targetSpeedCmS = targetSpeedCmS;

    const uint32_t pulseFrequencyHz =
        speedToPulseFrequencyHz(targetSpeedCmS);
    const bool shouldRun = pulseFrequencyHz > 0;

    if (!shouldRun) {
        stopPulseOutput(pins);
        markPulseStopped(status);
        return 0.0f;
    }

    const bool forward = targetSpeedCmS >= 0.0f;
    if (!status.enabled) {
        writeEnable(pins, true);
        status.enabled = true;
    }
    if (!status.pulseOutputRunning || status.forward != forward) {
        stopPulseOutput(pins);
        writeDirection(pins, forward);
    }

    runPulseOutput(pins, pulseFrequencyHz);

    status.forward = forward;
    status.pulseFrequencyHz = pulseFrequencyHz;
    status.pulseOutputRunning = true;
    return pulseFrequencyToSpeedCmS(pulseFrequencyHz, forward);
}

}  // namespace

bool StepperDriver::begin()
{
    initializeMotorPins(kLeftPins);
    initializeMotorPins(kRightPins);

    const bool leftAttached = attachPulseOutput(kLeftPins);
    const bool rightAttached = attachPulseOutput(kRightPins);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    if (leftAttached && rightAttached) {
        const uint32_t leftAttachHz = ledcReadFreq(kLeftPins.step);
        const uint32_t rightAttachHz = ledcReadFreq(kRightPins.step);
        Serial.printf(
            "[LEDC] independent attach check: left=%lu Hz right=%lu Hz "
            "(channels %u/%u)\n",
            static_cast<unsigned long>(leftAttachHz),
            static_cast<unsigned long>(rightAttachHz),
            static_cast<unsigned>(kLeftPins.channel),
            static_cast<unsigned>(kRightPins.channel));
    }
#endif

    status_.left.pulseGeneratorAttached = leftAttached;
    status_.right.pulseGeneratorAttached = rightAttached;
    refreshPulseGeneratorReady(status_);
    status_.begun = status_.pulseGeneratorReady;
    status_.configValid = stepperConfigValid();

    reset();

    return status_.begun && status_.configValid;
}

void StepperDriver::reset()
{
    stopPulseOutput(kLeftPins);
    stopPulseOutput(kRightPins);

    writeDirection(kLeftPins, true);
    writeDirection(kRightPins, true);
    writeEnable(kLeftPins, false);
    writeEnable(kRightPins, false);

    command_ = {0.0f, 0.0f};
    resetMotorStatus(status_.left);
    resetMotorStatus(status_.right);

    status_.configValid = stepperConfigValid();
    refreshPulseGeneratorReady(status_);
}

WheelState StepperDriver::update(const WheelState& target, float dtSeconds)
{
    (void)dtSeconds;

    if (!status_.begun || !stepperConfigValid()) {
        reset();
        return command_;
    }

    status_.configValid = true;
    refreshPulseGeneratorReady(status_);

    command_.leftSpeedCmS =
        updateMotor(kLeftPins, status_.left, target.leftSpeedCmS);
    command_.rightSpeedCmS =
        updateMotor(kRightPins, status_.right, target.rightSpeedCmS);

    return command_;
}

void StepperDriver::stop()
{
    stopPulseOutput(kLeftPins);
    stopPulseOutput(kRightPins);

    if (status_.begun && stepperConfigValid()) {
        writeEnable(kLeftPins, true);
        writeEnable(kRightPins, true);
        status_.left.enabled = true;
        status_.right.enabled = true;
    }

    command_ = {0.0f, 0.0f};
    status_.left.targetSpeedCmS = 0.0f;
    markPulseStopped(status_.left);
    status_.right.targetSpeedCmS = 0.0f;
    markPulseStopped(status_.right);

    status_.configValid = stepperConfigValid();
    refreshPulseGeneratorReady(status_);
}

const WheelState& StepperDriver::wheelState() const
{
    return command_;
}

const StepperDriverStatus& StepperDriver::status() const
{
    return status_;
}

bool StepperDriver::hasAlarm() const
{
    // ALM is not wired yet; keep external alarm reporting disabled for now.
    return false;
}
