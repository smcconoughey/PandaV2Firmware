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
                float vref = 1.25f; // internal ref; override per-bank if needed
                bank.out[_chanIdx] = vref * (float(raw) / 8388608.0f);
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
