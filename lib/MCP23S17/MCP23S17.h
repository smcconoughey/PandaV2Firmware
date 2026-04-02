#pragma once
#include <Arduino.h>
#include <SPI.h>

// MCP23S17 16-bit I/O expander driver
// All 16 pins configured as outputs driving actuator transistor array.
// Always write to OLAT registers (not GPIO) to avoid read-modify-write races.

class MCP23S17 {
public:
    MCP23S17(uint8_t csPin, SPIClass& bus, SPISettings settings, uint8_t hwAddr = 0);

    void begin();

    // Set a single actuator channel (1-indexed, 1..16). Returns false if out of range.
    bool setChannel(uint8_t channel, bool state);

    // Set all 16 channels at once (bit 0 = actuator 1, bit 15 = actuator 16)
    void setAll(uint16_t mask);

    // Turn all channels off. Call this on disarm.
    void allOff();

    // Read back the current OLAT state
    uint16_t getState() const { return _state; }

private:
    static constexpr uint8_t OPCODE_BASE = 0x40;

    // Register addresses (IOCON.BANK = 0, default)
    static constexpr uint8_t REG_IODIRA  = 0x00;
    static constexpr uint8_t REG_IODIRB  = 0x01;
    static constexpr uint8_t REG_OLATA   = 0x14;
    static constexpr uint8_t REG_OLATB   = 0x15;
    static constexpr uint8_t REG_IOCON   = 0x0A;

    uint8_t _cs;
    SPIClass& _spi;
    SPISettings _settings;
    uint8_t _opWrite;
    uint8_t _opRead;
    uint16_t _state = 0;

    void writeReg(uint8_t addr, uint8_t data);
    uint8_t readReg(uint8_t addr);
    void flushState();

    // Map 1-indexed channel to port/bit
    // PORTB[7:0] → ACTUATE1..8  (GPB7=ACT1, GPB0=ACT8)
    // PORTA[7:0] → ACTUATE9..16 (GPA7=ACT9, GPA0=ACT16)
    static uint16_t channelBit(uint8_t ch);
};
