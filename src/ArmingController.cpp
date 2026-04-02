#include "ArmingController.h"

ArmingController::ArmingController(uint8_t armPin, uint8_t disarmPin,
                                   uint32_t pulseMs, uint32_t cooldownMs)
    : _armPin(armPin), _disarmPin(disarmPin),
      _pulseMs(pulseMs), _cooldownMs(cooldownMs)
{
}

void ArmingController::begin() {
    pinMode(_armPin, OUTPUT);
    pinMode(_disarmPin, OUTPUT);

    // Power-on: assert disarm pulse to ensure known-safe state
    digitalWrite(_armPin, LOW);
    digitalWrite(_disarmPin, HIGH);
    delay(_pulseMs);
    digitalWrite(_disarmPin, LOW);

    _state = State::DISARMED;
    _phase = Phase::IDLE;
}

ArmingController::Result ArmingController::arm() {
    return requestState(State::ARMED);
}

ArmingController::Result ArmingController::disarm() {
    return requestState(State::DISARMED);
}

ArmingController::Result ArmingController::requestState(State desired) {
    if (_state == desired && _phase == Phase::IDLE) return Result::REDUNDANT;
    if (_phase != Phase::IDLE) return Result::IGNORED;

    allPinsLow();
    // Don't transition state yet — wait for pulse to complete
    _pendingState = desired;

    uint8_t activePin = (desired == State::ARMED) ? _armPin : _disarmPin;
    digitalWrite(activePin, HIGH);

    _phase = Phase::PULSING;
    _timer = 0;
    return Result::ACCEPTED;
}

void ArmingController::update() {
    switch (_phase) {
    case Phase::IDLE:
        break;

    case Phase::PULSING:
        if (_timer >= _pulseMs) {
            allPinsLow();
            _state = _pendingState; // commit state only after pulse delivered
            _phase = Phase::COOLDOWN;
            _timer = 0;
        }
        break;

    case Phase::COOLDOWN:
        if (_timer >= _cooldownMs) {
            _phase = Phase::IDLE;
        }
        break;
    }
}

void ArmingController::allPinsLow() {
    digitalWrite(_armPin, LOW);
    digitalWrite(_disarmPin, LOW);
}
