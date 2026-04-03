#include "MCP3561RT.h"

MCP3561RT::MCP3561RT(uint8_t csPin, uint8_t irqPin, SPIClass& bus,
                     SPISettings settings, float vref)
    : _cs(csPin), _irq(irqPin), _spi(bus), _settings(settings), _vref(vref)
{
}

bool MCP3561RT::begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    pinMode(_irq, INPUT_PULLUP);

    // CONFIG0: internal clock, conversion mode
    writeReg(REG_CONFIG0, 0b00110011);

    // CONFIG1: OSR 256
    writeReg(REG_CONFIG1, 0b00001100);

    // CONFIG2: defaults (gain 1x, boost current)
    writeReg(REG_CONFIG2, 0b10001111);

    // CONFIG3: one-shot mode, 24-bit data
    writeReg(REG_CONFIG3, 0b10000000);

    // IRQ register: disable STATUS byte prefix on reads (EN_STP=0),
    // keep fast commands enabled (EN_FASTCMD=1), open-drain IRQ output.
    // Upper nibble is read-only status bits (writes ignored).
    writeReg(REG_IRQ, 0b00000010);

    // MUX: CH0 vs AGND
    setMux(Mux::CH0, Mux::AGND);

    // Verify CONFIG0 was written correctly
    uint8_t check = readReg(REG_CONFIG0);
    return (check == 0b00110011);
}

void MCP3561RT::setMux(Mux vinp, Mux vinm) {
    uint8_t data = (static_cast<uint8_t>(vinp) << 4) | static_cast<uint8_t>(vinm);
    writeReg(REG_MUX, data);
}

void MCP3561RT::setGain(Gain g) {
    uint8_t reg = readReg(REG_CONFIG2);
    reg = (reg & 0b11000111) | (static_cast<uint8_t>(g) << 3);
    writeReg(REG_CONFIG2, reg);
}

void MCP3561RT::setOSR(OSR osr) {
    uint8_t reg = readReg(REG_CONFIG1);
    reg = (reg & 0b11000011) | (static_cast<uint8_t>(osr) << 2);
    writeReg(REG_CONFIG1, reg);
}

void MCP3561RT::trigger() {
    fastCmd(FAST_START);
}

bool MCP3561RT::dataReady() const {
    return !digitalRead(_irq);
}

bool MCP3561RT::readRaw(int32_t& out) {
    // Caller must confirm data-ready (via IRQ pin) before calling.
    // No redundant IRQ register read — that extra SPI transaction was
    // interfering with the data-ready state on subsequent conversions.

    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(cmdRead(REG_ADCDATA));
    uint8_t b0 = _spi.transfer(0); // MSB
    uint8_t b1 = _spi.transfer(0);
    uint8_t b2 = _spi.transfer(0); // LSB
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();

    uint32_t raw = (uint32_t(b0) << 16) | (uint32_t(b1) << 8) | uint32_t(b2);
    // Sign-extend 24-bit to 32-bit
    out = (raw & 0x800000) ? int32_t(raw | 0xFF000000) : int32_t(raw);
    return true;
}

float MCP3561RT::read() {
    int32_t raw;
    if (!readRaw(raw)) return 0.0f;
    return _vref * (float(raw) / 8388608.0f);
}

uint8_t MCP3561RT::readRegister(uint8_t addr) {
    return readReg(addr);
}

// --- private ---

void MCP3561RT::writeReg(uint8_t addr, uint8_t data) {
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(cmdWrite(addr));
    _spi.transfer(data);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}

uint8_t MCP3561RT::readReg(uint8_t addr) {
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(cmdRead(addr));
    uint8_t val = _spi.transfer(0);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
    return val;
}

void MCP3561RT::fastCmd(uint8_t cmd) {
    _spi.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    _spi.transfer(cmd);
    digitalWrite(_cs, HIGH);
    _spi.endTransaction();
}
