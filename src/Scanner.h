#pragma once
#include <Arduino.h>
#include <MCP3561RT.h>
#include "pins.h"
#include "BoardConfig.h"

// Generic mux-scanning ADC reader.
// Each Scanner instance owns one ADC and one or more mux banks.
// The state machine is non-blocking: call update() from the main loop.

struct MuxBank {
    const uint8_t pins[4];      // S0..S3 GPIO
    uint8_t numChannels;
    MCP3561RT::Mux adcInput;    // which ADC input the mux feeds
    float* out;                 // output buffer (caller-owned)
    float vref;                 // reference voltage for this bank's conversion
};

class Scanner {
public:
    Scanner(MCP3561RT& adc, MuxBank* banks, uint8_t numBanks);

    void begin();
    void update();

    bool scanComplete() const { return _scanComplete; }
    void clearScanComplete() { _scanComplete = false; }

    // Read ADC internal temperature sensor (for TC cold junction compensation).
    // Blocking — only call during setup or when you can afford ~1ms.
    float readBoardTemp();

private:
    enum State : uint8_t { IDLE, WAIT_MUX, WAIT_CONV };

    MCP3561RT& _adc;
    MuxBank* _banks;
    uint8_t _numBanks;

    State _state = IDLE;
    uint8_t _bankIdx = 0;
    uint8_t _chanIdx = 0;
    elapsedMicros _timer;
    bool _scanComplete = false;

    void selectMuxChannel(const MuxBank& bank, uint8_t ch);
    void advanceChannel();
};
