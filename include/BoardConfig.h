#pragma once
#include <SPI.h>

// SPI
static const SPISettings SPI_ADC_SETTINGS(20000000, MSBFIRST, SPI_MODE0);

// ADC timing
static constexpr uint32_t T_MUX_SETTLE_US = 500;

// Serial
static constexpr uint32_t RS485_BAUD       = 460800;
static constexpr uint32_t DEBUG_BAUD       = 460800;
static constexpr size_t   RS485_RX_BUF     = 512;
static constexpr size_t   RS485_TX_BUF     = 2048;
static constexpr uint32_t PACKET_IDLE_MS   = 100;

// Channel counts
static constexpr uint8_t NUM_ACTUATORS     = 8;

// DC solenoid output pins (direct GPIO, 1-indexed channel n → DC_PINS[n-1])
// NOTE: pins 0,1,7,8 overlap RS-485 Serial1/Serial2 — verify hardware mux intent.
//       pins 15,16 overlap PIN_ARM/PIN_DISARM placeholders in pins.h.
static constexpr uint8_t DC_PINS[8]        = {17, 16, 15, 0, 7, 8, 9, 1};
static constexpr uint8_t NUM_MUX_A_CH      = 16;  // voltage/differential via mux A
static constexpr uint8_t NUM_MUX_B_CH      = 16;  // current sense via mux B
static constexpr uint8_t NUM_MUX_C_CH      = 16;  // ADC2 channels via mux C
static constexpr uint8_t NUM_MAX_COMMANDS   = 64;

// Telemetry identifiers (must match control software)
static constexpr char ID_SOLENOID_CURRENT  = 's';
static constexpr char ID_LCTC              = 't';
static constexpr char ID_POWER             = 'v';

// Conversion constants (carry forward from V1, recalibrate on V2 hardware)
static constexpr float TC_CONSTANT         = 2217.294f;
static constexpr float S_CONSTANT          = 0.5f;
static constexpr uint8_t DATA_DECIMALS     = 5;
