#include "Scanner.h"

Scanner::Scanner(MCP3561RT& adc, MuxBank* banks, uint8_t numBanks)
    : _adc(adc), _banks(banks), _numBanks(numBanks)
{
}

void Scanner::begin() {
    for (uint8_t b = 0; b < _numBanks; b++) {
        for (uint8_t i = 0; i < 4; i++) {
            pinMode(_banks[b].pins[i], OUTPUT);
            digitalWrite(_banks[b].pins[i], LOW);
        }
    }
}

void Scanner::update() {
    if (_bankIdx >= _numBanks) { _bankIdx = 0; }
    MuxBank& bank = _banks[_bankIdx];
    if (_chanIdx >= bank.numChannels) { _chanIdx = 0; }

    switch (_state) {
    case IDLE:
        _adc.setMux(bank.adcInput, MCP3561RT::Mux::AGND);
        selectMuxChannel(bank, _chanIdx);
        _timer = 0;
        _state = WAIT_MUX;
        break;

    case WAIT_MUX:
        if (_timer >= T_MUX_SETTLE_US) {
            _adc.trigger();
            _timer = 0;
            _state = WAIT_CONV;
        }
        break;

    case WAIT_CONV:
        if (_adc.dataReady()) {
            int32_t raw;
            if (_adc.readRaw(raw)) {
                bank.out[_chanIdx] = bank.vref * (float(raw) / 8388608.0f);
            }
            advanceChannel();
            _state = IDLE;
        }
        break;
    }
}

void Scanner::selectMuxChannel(const MuxBank& bank, uint8_t ch) {
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(bank.pins[i], (ch >> i) & 1);
    }
}

float Scanner::readBoardTemp() {
    // MCP3561RT internal temp sensor: set mux to TEMP_P vs TEMP_N,
    // do a blocking one-shot, convert using datasheet formula.
    // T(°C) = (V_temp - 0.492) / 0.00176 approximately.
    // This is a rough estimate — datasheet says ~±2°C typical.
    _adc.setMux(MCP3561RT::Mux::TEMP_P, MCP3561RT::Mux::TEMP_N);
    delayMicroseconds(T_MUX_SETTLE_US);
    _adc.trigger();

    elapsedMicros timeout;
    while (!_adc.dataReady()) {
        if (timeout > 50000) return -999.0f;
    }

    int32_t raw;
    if (!_adc.readRaw(raw)) return -999.0f;

    float voltage = 1.25f * (float(raw) / 8388608.0f);
    return (voltage - 0.492f) / 0.00176f;
}

void Scanner::advanceChannel() {
    _chanIdx++;
    if (_chanIdx >= _banks[_bankIdx].numChannels) {
        _chanIdx = 0;
        _bankIdx++;
        if (_bankIdx >= _numBanks) {
            _bankIdx = 0;
            _scanComplete = true;
        }
    }
}
