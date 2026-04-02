#pragma once
#include <Arduino.h>

// Arming state machine with explicit transitions and safety guards.
//
// V1 had arm/disarm as raw GPIO writes inline in main — no debounce,
// no guard against accidental re-arm, no minimum dwell time. This version
// enforces:
//   - Minimum pulse duration on the arm/disarm output
//   - Mandatory cooldown between state changes
//   - Explicit state queries so other subsystems can gate on armed status
//
// The arm and disarm pins drive external relay/pyro logic. Only one
// should ever be asserted at a time.

class ArmingController {
public:
    enum class State : uint8_t { DISARMED, ARMED };

    enum class Result : uint8_t {
        ACCEPTED,   // transition started
        IGNORED,    // still in pulse or cooldown
        REDUNDANT   // already in the requested state
    };

    ArmingController(uint8_t armPin, uint8_t disarmPin,
                     uint32_t pulseMs = 1000, uint32_t cooldownMs = 1000);

    void begin();
    void update();

    Result arm();
    Result disarm();

    State state() const { return _state; }
    bool isArmed() const { return _state == State::ARMED; }

private:
    enum class Phase : uint8_t { IDLE, PULSING, COOLDOWN };

    uint8_t _armPin;
    uint8_t _disarmPin;
    uint32_t _pulseMs;
    uint32_t _cooldownMs;

    State _state = State::DISARMED;
    State _pendingState = State::DISARMED;
    Phase _phase = Phase::IDLE;
    elapsedMillis _timer;

    Result requestState(State desired);
    void allPinsLow();
};
