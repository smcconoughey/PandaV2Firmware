#pragma once
#include <Arduino.h>

// Sensor channel mapping and conversion for PandaV2.
//
// Mux A (16ch) → ADC1 CH0: PTs through INA132 difference amps
// Mux B (16ch) → ADC1 CH1: solenoid current through INA181 (200x gain, 100mΩ shunt)
// Mux C (16ch) → ADC2 CH0: LCs (ch 0-7) + TCs (ch 8-15) through INA317/INA826
//
// Channel counts are configurable — adjust if your board populates fewer.

enum class SensorType : uint8_t {
    RAW,            // no conversion, report voltage
    PRESSURE,       // PT: voltage → PSI
    LOAD_CELL,      // LC: voltage → lbf (or N)
    THERMOCOUPLE,   // TC: voltage → °C (with CJC)
    CURRENT_SENSE   // solenoid current: voltage → amps
};

struct SensorCal {
    SensorType type;
    float scale;    // engineering_units = (voltage - offset) * scale
    float offset;   // voltage offset
};

// ── Mux C channel layout ───────────────────────────────────────────
// Channels 0..7  = load cells (INA317/INA826)
// Channels 8..15 = thermocouples (INA317/INA826)
// Adjust these if your board is wired differently.

static constexpr uint8_t MUX_C_LC_START  = 0;
static constexpr uint8_t MUX_C_LC_COUNT  = 8;
static constexpr uint8_t MUX_C_TC_START  = 8;
static constexpr uint8_t MUX_C_TC_COUNT  = 8;

static constexpr uint8_t NUM_PT_CH = 16;
static constexpr uint8_t NUM_LC_CH = MUX_C_LC_COUNT;
static constexpr uint8_t NUM_TC_CH = MUX_C_TC_COUNT;

// ── PT calibration ─────────────────────────────────────────────────
// Default: report raw voltage. Replace with per-channel cal from
// transducer datasheets once known (e.g. 0-5000 PSI over 0-1.0V).

static constexpr SensorCal PT_DEFAULT = {SensorType::PRESSURE, 1.0f, 0.0f};

// ── Load cell calibration ──────────────────────────────────────────
// Default: report raw voltage. Replace with per-channel cal.

static constexpr SensorCal LC_DEFAULT = {SensorType::LOAD_CELL, 1.0f, 0.0f};

// ── Thermocouple conversion ────────────────────────────────────────
// Carried forward from V1: temperature = (V - offset) * TC_CONSTANT + T_board
// TC_CONSTANT = 2217.294 °C/V (Type K, through amp chain)
// Per-channel offsets from V1 (first 6 channels populated):

static constexpr float TC_SCALE = 2217.294f;

static constexpr float TC_OFFSETS[NUM_TC_CH] = {
    -0.07429f, -0.06889f, -0.07387f, -0.0786f,
    -0.06998f, -0.07754f, 0.0f, 0.0f  // channels 6-7 unpopulated on V1
};

// ── Current sense conversion ───────────────────────────────────────
// INA181 at 200x gain with 100mΩ shunt:
// V_out = I_load * R_shunt * Gain = I_load * 0.1 * 200 = I_load * 20
// So I_load = V_adc / 20

static constexpr float CURRENT_SENSE_SCALE = 1.0f / 20.0f;  // A per V

// ── Conversion functions ───────────────────────────────────────────

inline float convertPT(float voltage, uint8_t ch) {
    (void)ch; // per-channel cal not yet implemented
    return (voltage - PT_DEFAULT.offset) * PT_DEFAULT.scale;
}

inline float convertLC(float voltage, uint8_t ch) {
    (void)ch;
    return (voltage - LC_DEFAULT.offset) * LC_DEFAULT.scale;
}

inline float convertTC(float voltage, uint8_t ch, float boardTemp) {
    float offset = (ch < NUM_TC_CH) ? TC_OFFSETS[ch] : 0.0f;
    return (voltage - offset) * TC_SCALE + boardTemp;
}

inline float convertCurrent(float voltage) {
    return voltage * CURRENT_SENSE_SCALE;
}
