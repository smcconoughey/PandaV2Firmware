#include "BangBang.h"
#include <EEPROM.h>

BBController::BBController(const float* ptSrc, uint8_t dcCh, char busId)
    : _ptSrc(ptSrc), _dcCh(dcCh), _busId(busId) {}

void BBController::configure(float setpoint, float deadband, uint32_t waitMs) {
    _cfg.setpoint_psi = setpoint;
    _cfg.deadband_psi = deadband;
    _cfg.wait_ms      = waitMs;
}

bool BBController::enable() {
    if (_enabled) return false;
    _enabled = true;
    // Don't reset _switchTimer here — let it reflect time since last switch
    // (or time since boot on first enable, which is always ≥ wait_ms).
    return true;
}

void BBController::_closeValve(bool (*setChannel)(uint8_t, bool)) {
    if (_valveOpen) {
        setChannel(_dcCh, false);
        _valveOpen = false;
    }
}

void BBController::disableNow(bool (*setChannel)(uint8_t, bool)) {
    _enabled = false;
    _closeValve(setChannel);
}

void BBController::update(bool armed, bool (*setChannel)(uint8_t, bool)) {
    if (!_enabled) return;

    // Safety: arm state lost → shut down immediately
    if (!armed) {
        disableNow(setChannel);
        return;
    }

    float pressure = *_ptSrc;
    _lastPressure = pressure;

    // Sanity check — out-of-range means ADC fault or disconnected sensor
    if (pressure < BB_PRESSURE_MIN_PSI || pressure > BB_PRESSURE_MAX_PSI) {
        disableNow(setChannel);
        return;
    }

    // Enforce minimum dwell time between valve switches
    if (_switchTimer < _cfg.wait_ms) return;

    const float hi = _cfg.setpoint_psi + _cfg.deadband_psi * 0.5f;
    const float lo = _cfg.setpoint_psi - _cfg.deadband_psi * 0.5f;

    if (pressure > hi && !_valveOpen) {
        _valveOpen = true;
        _switchTimer = 0;
        setChannel(_dcCh, true);
    } else if (pressure < lo && _valveOpen) {
        _valveOpen = false;
        _switchTimer = 0;
        setChannel(_dcCh, false);
    }
}

// ── EEPROM helpers ─────────────────────────────────────────────────────────

static uint8_t computeCrc(const BBConfig& lox, const BBConfig& fuel) {
    uint8_t crc = 0;
    const uint8_t* p;

    p = reinterpret_cast<const uint8_t*>(&lox);
    for (size_t i = 0; i < sizeof(BBConfig); i++) crc ^= p[i];

    p = reinterpret_cast<const uint8_t*>(&fuel);
    for (size_t i = 0; i < sizeof(BBConfig); i++) crc ^= p[i];

    return crc;
}

void bbLoadEeprom(BBController& lox, BBController& fuel) {
    BBEepromBlock block;
    EEPROM.get(BB_EEPROM_ADDR, block);

    if (block.magic != BB_EEPROM_MAGIC) return;

    uint8_t expected = computeCrc(block.lox, block.fuel);
    if (block.crc != expected) return;

    lox.configure(block.lox.setpoint_psi, block.lox.deadband_psi, block.lox.wait_ms);
    fuel.configure(block.fuel.setpoint_psi, block.fuel.deadband_psi, block.fuel.wait_ms);
}

void bbSaveEeprom(const BBController& lox, const BBController& fuel) {
    BBEepromBlock block;
    block.magic = BB_EEPROM_MAGIC;
    block.lox   = lox.config();
    block.fuel  = fuel.config();
    block.crc   = computeCrc(block.lox, block.fuel);
    EEPROM.put(BB_EEPROM_ADDR, block);
}
