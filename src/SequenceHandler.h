#pragma once
#include <Arduino.h>
#include "BoardConfig.h"

// Timed actuator sequence engine.
// Parses V1-compatible sequence strings, drives solenoids via DC_PINS GPIO.
// Format: "s<chan_hex><state>.<delay_ms_5dig>,..."
// Example: "s11.01000,s10.00000,s81.02000,s80.00000"

struct SequenceStep {
    uint8_t channel; // 1-indexed (1..NUM_ACTUATORS)
    bool state;
    uint32_t delay_ms;
};

class SequenceHandler {
public:
    SequenceHandler();

    // Configure DC_PINS as outputs, all off. Call from setup().
    void begin();

    // Parse a sequence command string. Returns false on parse error.
    bool setCommand(const char* command);

    // Execute the loaded sequence. Returns false if nothing loaded.
    bool execute();

    // Call from main loop — advances the state machine.
    void update();

    void cancelExecution();
    void setAllOff();

    // Direct single-channel control (bypasses sequencing)
    bool setChannel(uint8_t channel, bool state);

    bool isActive() const { return _active; }
    bool hasSequence() const { return _numSteps > 0; }
    uint8_t getNumSteps() const { return _numSteps; }
    const char* getLastCommand() const { return _lastCmdValid ? _lastCmd : ""; }

private:
    SequenceStep _steps[NUM_MAX_COMMANDS];
    uint8_t _numSteps = 0;
    uint8_t _currentStep = 0;
    bool _active = false;
    bool _inDelay = false;
    elapsedMillis _stepTimer;

    bool _lastCmdValid = false;
    char _lastCmd[256] = {0};
};
