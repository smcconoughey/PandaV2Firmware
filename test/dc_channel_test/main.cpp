// DC Channel Hardware Test — PandaV2
// Menu-driven serial console for validating ADC + mux + actuation chain.

#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "BoardConfig.h"
#include <MCP3561RT.h>
#include <MCP23S17.h>

// ── Configuration ───────────────────────────────────────────────────

static constexpr uint8_t MUX_A_PINS[4] = {PIN_MUX_A_S0, PIN_MUX_A_S1, PIN_MUX_A_S2, PIN_MUX_A_S3};
static constexpr uint8_t MUX_B_PINS[4] = {PIN_MUX_B_S0, PIN_MUX_B_S1, PIN_MUX_B_S2, PIN_MUX_B_S3};
static constexpr uint8_t MUX_C_PINS[4] = {PIN_MUX_C_S0, PIN_MUX_C_S1, PIN_MUX_C_S2, PIN_MUX_C_S3};

static constexpr uint32_t CONV_TIMEOUT_US = 50000;
static constexpr uint16_t DEFAULT_NOISE_SAMPLES = 200;
static constexpr uint32_t DEFAULT_PULSE_MS = 500;

// ── Hardware instances ──────────────────────────────────────────────

MCP3561RT adc1(PIN_ADC1_CS, PIN_ADC1_IRQ, SPI,  SPI_ADC_SETTINGS, 1.25f);
MCP3561RT adc2(PIN_ADC2_CS, PIN_ADC2_IRQ, SPI1, SPI_ADC_SETTINGS, 1.25f);
MCP23S17  ioexp(PIN_IOEXP_CS, SPI1, SPI_IOEXP_SETTINGS);

// ── Mux helpers ─────────────────────────────────────────────────────

static void initMuxPins(const uint8_t pins[4]) {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
}

static void setMuxChannel(const uint8_t pins[4], uint8_t ch) {
    for (uint8_t i = 0; i < 4; i++) {
        digitalWrite(pins[i], (ch >> i) & 1);
    }
}

// ── ADC read helpers ────────────────────────────────────────────────

struct ReadResult {
    bool ok;
    int32_t raw;
    float voltage;
};

static ReadResult singleRead(MCP3561RT& adc) {
    ReadResult r = {false, 0, 0.0f};

    adc.trigger();

    elapsedMicros timeout;
    while (!adc.dataReady()) {
        if (timeout > CONV_TIMEOUT_US) return r;
    }

    if (adc.readRaw(r.raw)) {
        r.voltage = 1.25f * (float(r.raw) / 8388608.0f);
        r.ok = true;
    }
    return r;
}

static ReadResult readMuxChannel(MCP3561RT& adc, const uint8_t muxPins[4],
                                  MCP3561RT::Mux adcInput, uint8_t ch) {
    adc.setMux(adcInput, MCP3561RT::Mux::AGND);
    setMuxChannel(muxPins, ch);
    delayMicroseconds(T_MUX_SETTLE_US);
    return singleRead(adc);
}

// ── Serial input helpers ────────────────────────────────────────────

static int readSerialInt() {
    while (!Serial.available()) { yield(); }
    return Serial.parseInt();
}

// ── Test functions ──────────────────────────────────────────────────

static void testRegisterDump() {
    Serial.println("=== ADC Register Dump ===");

    const char* regNames[] = {
        "ADCDATA", "CONFIG0", "CONFIG1", "CONFIG2", "CONFIG3", "IRQ", "MUX"
    };

    for (uint8_t adcIdx = 0; adcIdx < 2; adcIdx++) {
        MCP3561RT& adc = (adcIdx == 0) ? adc1 : adc2;
        Serial.printf("\nADC%u:\n", adcIdx + 1);
        for (uint8_t reg = 0; reg <= 6; reg++) {
            uint8_t val = adc.readRegister(reg);
            Serial.printf("  [0x%02X] %-8s = 0x%02X (0b", reg, regNames[reg], val);
            for (int8_t b = 7; b >= 0; b--) Serial.print((val >> b) & 1);
            Serial.println(")");
        }
    }
    Serial.println();
}

static void testSingleChannel() {
    Serial.println("=== Single Channel Read ===");
    Serial.println("Select bank: 1=MuxA(ADC1) 2=MuxB(ADC1) 3=MuxC(ADC2)");

    int bank = readSerialInt();
    if (bank < 1 || bank > 3) { Serial.println("Invalid bank."); return; }

    Serial.printf("Channel (0-%u): ", NUM_MUX_A_CH - 1);
    int ch = readSerialInt();
    if (ch < 0 || ch > 15) { Serial.println("Invalid channel."); return; }

    MCP3561RT& adc       = (bank <= 2) ? adc1 : adc2;
    const uint8_t* pins  = (bank == 1) ? MUX_A_PINS : (bank == 2) ? MUX_B_PINS : MUX_C_PINS;
    MCP3561RT::Mux input = (bank == 2) ? MCP3561RT::Mux::CH1 : MCP3561RT::Mux::CH0;

    ReadResult r = readMuxChannel(adc, pins, input, (uint8_t)ch);

    if (r.ok) {
        Serial.printf("Bank %d CH%02d: raw=%d  voltage=%.6f V\n", bank, ch, r.raw, r.voltage);
    } else {
        Serial.printf("Bank %d CH%02d: TIMEOUT / READ FAIL\n", bank, ch);
    }
    Serial.println();
}

static void testSweep() {
    Serial.println("=== 16-Channel Sweep ===");
    Serial.println("Select bank: 1=MuxA(ADC1) 2=MuxB(ADC1) 3=MuxC(ADC2)");

    int bank = readSerialInt();
    if (bank < 1 || bank > 3) { Serial.println("Invalid bank."); return; }

    MCP3561RT& adc       = (bank <= 2) ? adc1 : adc2;
    const uint8_t* pins  = (bank == 1) ? MUX_A_PINS : (bank == 2) ? MUX_B_PINS : MUX_C_PINS;
    MCP3561RT::Mux input = (bank == 2) ? MCP3561RT::Mux::CH1 : MCP3561RT::Mux::CH0;

    Serial.printf("Bank %d sweep:\n", bank);
    Serial.println("  CH   |     Raw     |   Voltage (V)");
    Serial.println("  -----+-------------+--------------");

    for (uint8_t ch = 0; ch < 16; ch++) {
        ReadResult r = readMuxChannel(adc, pins, input, ch);
        if (r.ok) {
            Serial.printf("  %02d   | %10d  |  %+.6f\n", ch, r.raw, r.voltage);
        } else {
            Serial.printf("  %02d   |    FAIL     |     ---\n", ch);
        }
    }
    Serial.println();
}

static void testNoise() {
    Serial.println("=== Noise Statistics ===");
    Serial.println("Select bank: 1=MuxA(ADC1) 2=MuxB(ADC1) 3=MuxC(ADC2)");

    int bank = readSerialInt();
    if (bank < 1 || bank > 3) { Serial.println("Invalid bank."); return; }

    Serial.printf("Channel (0-%u): ", NUM_MUX_A_CH - 1);
    int ch = readSerialInt();
    if (ch < 0 || ch > 15) { Serial.println("Invalid channel."); return; }

    MCP3561RT& adc       = (bank <= 2) ? adc1 : adc2;
    const uint8_t* pins  = (bank == 1) ? MUX_A_PINS : (bank == 2) ? MUX_B_PINS : MUX_C_PINS;
    MCP3561RT::Mux input = (bank == 2) ? MCP3561RT::Mux::CH1 : MCP3561RT::Mux::CH0;

    adc.setMux(input, MCP3561RT::Mux::AGND);
    setMuxChannel(pins, (uint8_t)ch);
    delayMicroseconds(T_MUX_SETTLE_US);

    Serial.printf("Sampling %u readings on bank %d CH%02d...\n", DEFAULT_NOISE_SAMPLES, bank, ch);

    double sum = 0.0, sumSq = 0.0;
    int32_t minRaw = INT32_MAX, maxRaw = INT32_MIN;
    uint16_t good = 0;

    for (uint16_t i = 0; i < DEFAULT_NOISE_SAMPLES; i++) {
        ReadResult r = singleRead(adc);
        if (!r.ok) continue;

        good++;
        sum += r.raw;
        sumSq += (double)r.raw * (double)r.raw;
        if (r.raw < minRaw) minRaw = r.raw;
        if (r.raw > maxRaw) maxRaw = r.raw;
    }

    if (good < 2) {
        Serial.println("Too few valid samples.");
        return;
    }

    double mean = sum / good;
    double variance = (sumSq / good) - (mean * mean);
    double stddev = sqrt(variance);
    double meanV = 1.25 * (mean / 8388608.0);
    double stddevV = 1.25 * (stddev / 8388608.0);

    Serial.printf("  Samples:  %u / %u\n", good, DEFAULT_NOISE_SAMPLES);
    Serial.printf("  Mean:     %.1f counts  (%.6f V)\n", mean, meanV);
    Serial.printf("  Std Dev:  %.1f counts  (%.6f V)\n", stddev, stddevV);
    Serial.printf("  Peak-Pk:  %d counts  (min=%d, max=%d)\n",
                  maxRaw - minRaw, minRaw, maxRaw);
    Serial.printf("  ENOB:     %.1f bits\n", log2(8388608.0 / stddev));
    Serial.println();
}

// ── Actuation test functions ────────────────────────────────────────

static void testActuateSingle() {
    Serial.println("=== Single Actuator Toggle ===");
    Serial.printf("Channel (1-%u): ", NUM_ACTUATORS);
    int ch = readSerialInt();
    if (ch < 1 || ch > 16) { Serial.println("Invalid channel."); return; }

    Serial.printf("Pulse duration ms (%lu default): ", DEFAULT_PULSE_MS);
    int dur = readSerialInt();
    if (dur <= 0) dur = DEFAULT_PULSE_MS;

    Serial.printf("Actuating CH%02d for %d ms...", ch, dur);
    ioexp.setChannel((uint8_t)ch, true);
    delay(dur);
    ioexp.setChannel((uint8_t)ch, false);
    Serial.println(" done.");
    Serial.println();
}

static void testActuateWalk() {
    Serial.println("=== Walk All Actuators ===");
    Serial.printf("Pulse duration ms per channel (%lu default): ", DEFAULT_PULSE_MS);
    int dur = readSerialInt();
    if (dur <= 0) dur = DEFAULT_PULSE_MS;

    for (uint8_t ch = 1; ch <= NUM_ACTUATORS; ch++) {
        Serial.printf("  CH%02d ON...", ch);
        ioexp.setChannel(ch, true);
        delay(dur);
        ioexp.setChannel(ch, false);
        Serial.println(" OFF");
        delay(50);  // brief gap between channels
    }
    Serial.println("Walk complete.");
    Serial.println();
}

static void testActuateAllOnOff() {
    Serial.println("=== All Actuators On/Off ===");
    Serial.printf("Pulse duration ms (%lu default): ", DEFAULT_PULSE_MS);
    int dur = readSerialInt();
    if (dur <= 0) dur = DEFAULT_PULSE_MS;

    Serial.printf("All ON for %d ms...", dur);
    ioexp.setAll(0xFFFF);
    delay(dur);
    ioexp.allOff();
    Serial.println(" All OFF.");
    Serial.println();
}

static void testActuation() {
    Serial.println("=== Actuation Test ===");
    Serial.println("  1  Toggle single channel");
    Serial.println("  2  Walk all channels");
    Serial.println("  3  All on / all off");
    Serial.print("> ");

    int sub = readSerialInt();
    switch (sub) {
        case 1: testActuateSingle();   break;
        case 2: testActuateWalk();     break;
        case 3: testActuateAllOnOff(); break;
        default: Serial.println("Invalid option."); break;
    }
}

// ── Menu ────────────────────────────────────────────────────────────

static void printMenu() {
    Serial.println("──────────────────────────────────");
    Serial.println("  PandaV2 DC Channel Test");
    Serial.println("──────────────────────────────────");
    Serial.println("  1  ADC register dump");
    Serial.println("  2  Single channel read");
    Serial.println("  3  16-channel sweep");
    Serial.println("  4  Noise statistics");
    Serial.println("  5  Actuation test");
    Serial.println("──────────────────────────────────");
    Serial.print("> ");
}

// ── Entry points ────────────────────────────────────────────────────

void setup() {
    Serial.begin(DEBUG_BAUD);
    while (!Serial && millis() < 3000) {}

    SPI.begin();
    SPI1.setMISO(PIN_SPI1_MISO);  // default is pin 1; hardware uses pin 39
    SPI1.begin();

    initMuxPins(MUX_A_PINS);
    initMuxPins(MUX_B_PINS);
    initMuxPins(MUX_C_PINS);

    bool adc1_ok = adc1.begin();
    bool adc2_ok = adc2.begin();
    ioexp.begin();

    Serial.println("\nPandaV2 DC Channel Test");
    Serial.printf("ADC1:  %s\n", adc1_ok ? "OK" : "FAIL");
    Serial.printf("ADC2:  %s\n", adc2_ok ? "OK" : "FAIL");
    Serial.println("IOEXP: OK");

    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    int cmd = Serial.parseInt();

    switch (cmd) {
        case 1: testRegisterDump();  break;
        case 2: testSingleChannel(); break;
        case 3: testSweep();         break;
        case 4: testNoise();         break;
        case 5: testActuation();     break;
        default: break;
    }

    printMenu();
}
