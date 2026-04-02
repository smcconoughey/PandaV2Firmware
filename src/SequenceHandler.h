#pragma once
#include <Arduino.h>
#include <MCP23S17.h>
#include "BoardConfig.h"

// Timed actuator sequence engine.
// Parses V1-compatible sequence strings, executes through the MCP23S17 I/O expander.
// Format: "s<chan_hex><state>.<delay_ms_5dig>,..."
// Example: "s11.01000,s10.00000,sA1.02000,sA0.00000"

struct SequenceStep {
    uint8_t channel; // 1-indexed (1..16)
    bool state;
    uint32_t delay_ms;
};

class SequenceHandler {
public:
    explicit SequenceHandler(MCP23S17& ioexp);

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
    MCP23S17& _io;

    SequenceStep _steps[NUM_MAX_COMMANDS];
    uint8_t _numSteps = 0;
    uint8_t _currentStep = 0;
    bool _active = false;
    bool _inDelay = false;
    elapsedMillis _stepTimer;

    bool _lastCmdValid = false;
    char _lastCmd[256] = {0};
};
