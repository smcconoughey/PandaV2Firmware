// RS-485 Hardware Test — PandaV2
// Validates Serial1 (RS485_1) and Serial2 (RS485_2) transceivers.
// Both ports are always-driven (DE tied to +3V3, /RE tied to GND) so
// loopback tests require a physical wire between the two ports' A/B lines.

#include <Arduino.h>
#include "pins.h"
#include "BoardConfig.h"

// ── Configuration ────────────────────────────────────────────────────

static constexpr uint32_t TEST_BAUD          = RS485_BAUD;
static constexpr uint32_t ECHO_TIMEOUT_MS    = 200;
static constexpr uint16_t BURST_BYTES        = 256;
static constexpr uint16_t STRESS_ITERATIONS  = 100;

// ── Serial input helpers ─────────────────────────────────────────────

static int readSerialInt() {
    while (!Serial.available()) { yield(); }
    return Serial.parseInt();
}

static void flushRx(HardwareSerial& port) {
    uint32_t deadline = millis() + 20;
    while (millis() < deadline) {
        while (port.available()) port.read();
    }
}

// ── Test helpers ─────────────────────────────────────────────────────

static const char* portName(uint8_t p) {
    return (p == 1) ? "RS485_1 (Serial7)" : "RS485_2 (Serial8)";
}

static HardwareSerial& getPort(uint8_t p) {
    return (p == 1) ? Serial7 : Serial8;
}

// Send a byte pattern and wait to receive it back on the other port.
// Both ports must be connected back-to-back (A-A, B-B cross-wired or
// via a loopback adapter).
static bool loopbackByte(HardwareSerial& tx, HardwareSerial& rx,
                          uint8_t val, uint32_t timeoutMs = ECHO_TIMEOUT_MS) {
    flushRx(rx);
    tx.write(val);
    tx.flush();

    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (rx.available()) {
            uint8_t got = rx.read();
            return (got == val);
        }
    }
    return false;  // timeout
}

// ── Test functions ───────────────────────────────────────────────────

static void testBaudCheck() {
    Serial.println("=== Baud Rate Verify ===");
    Serial.printf("  RS485_1: %lu baud  (Serial1)\n", TEST_BAUD);
    Serial.printf("  RS485_2: %lu baud  (Serial2)\n", TEST_BAUD);
    Serial.println("  Confirm baud matches control software.");
    Serial.println();
}

static void testLoopback() {
    Serial.println("=== Port-to-Port Loopback ===");
    Serial.println("  Requires: RS485_1 A/B wired to RS485_2 A/B (cross-connect).");
    Serial.println("  Press ENTER to begin...");
    while (!Serial.available()) { yield(); }
    while (Serial.available()) Serial.read();

    // Walk all 256 byte values
    uint16_t pass12 = 0, fail12 = 0;
    uint16_t pass21 = 0, fail21 = 0;

    Serial.println("  Serial7 → Serial6:");
    for (uint16_t v = 0; v < 256; v++) {
        if (loopbackByte(Serial7, Serial6, (uint8_t)v)) pass12++;
        else                                              fail12++;
    }
    Serial.printf("    Pass: %u / 256  Fail: %u\n", pass12, fail12);

    Serial.println("  Serial6 → Serial7:");
    for (uint16_t v = 0; v < 256; v++) {
        if (loopbackByte(Serial6, Serial7, (uint8_t)v)) pass21++;
        else                                              fail21++;
    }
    Serial.printf("    Pass: %u / 256  Fail: %u\n", pass21, fail21);

    bool ok = (fail12 == 0 && fail21 == 0);
    Serial.printf("  Result: %s\n", ok ? "PASS" : "FAIL");
    Serial.println();
}

static void testBurstThroughput() {
    Serial.println("=== Burst Throughput ===");
    Serial.println("  Requires: RS485_1 A/B wired to RS485_2 A/B.");
    Serial.println("  Press ENTER to begin...");
    while (!Serial.available()) { yield(); }
    while (Serial.available()) Serial.read();

    // Build payload
    uint8_t payload[BURST_BYTES];
    for (uint16_t i = 0; i < BURST_BYTES; i++) payload[i] = (uint8_t)i;

    // TX from Serial7, RX on Serial6
    flushRx(Serial6);
    uint32_t t0 = micros();
    Serial7.write(payload, BURST_BYTES);
    Serial7.flush();
    uint32_t txUs = micros() - t0;

    // Collect reply
    uint32_t deadline = millis() + ECHO_TIMEOUT_MS * 4;
    uint8_t rxBuf[BURST_BYTES];
    uint16_t rxCount = 0;

    while (rxCount < BURST_BYTES && millis() < deadline) {
        while (Serial6.available() && rxCount < BURST_BYTES) {
            rxBuf[rxCount++] = Serial6.read();
        }
    }

    uint16_t errors = 0;
    for (uint16_t i = 0; i < rxCount; i++) {
        if (rxBuf[i] != payload[i]) errors++;
    }

    float kbps = (rxCount * 8.0f) / (txUs / 1000.0f);  // approx line rate
    Serial.printf("  Sent: %u bytes  Received: %u  Errors: %u\n",
                  BURST_BYTES, rxCount, errors);
    Serial.printf("  TX time: %lu µs  (~%.1f kbps line)\n", txUs, kbps);
    Serial.printf("  Result: %s\n", (rxCount == BURST_BYTES && errors == 0) ? "PASS" : "FAIL");
    Serial.println();
}

static void testStress() {
    Serial.println("=== Stress Test ===");
    Serial.printf("  %u back-to-back random patterns, Serial1→Serial2.\n", STRESS_ITERATIONS);
    Serial.println("  Requires: RS485_1 A/B wired to RS485_2 A/B.");
    Serial.println("  Press ENTER to begin...");
    while (!Serial.available()) { yield(); }
    while (Serial.available()) Serial.read();

    uint16_t pass = 0, fail = 0;
    for (uint16_t i = 0; i < STRESS_ITERATIONS; i++) {
        uint8_t val = (uint8_t)(random(256));
        if (loopbackByte(Serial7, Serial6, val)) pass++;
        else                                      fail++;
    }

    Serial.printf("  Pass: %u  Fail: %u\n", pass, fail);
    Serial.printf("  Result: %s\n", (fail == 0) ? "PASS" : "FAIL");
    Serial.println();
}

static void testMonitor() {
    Serial.println("=== Live Monitor ===");
    Serial.println("  Select port to monitor: 1=RS485_1  2=RS485_2");
    int p = readSerialInt();
    if (p < 1 || p > 2) { Serial.println("  Invalid."); return; }

    HardwareSerial& mon = getPort((uint8_t)p);
    Serial.printf("  Monitoring %s — press any key to stop.\n", portName((uint8_t)p));
    Serial.println("  Bytes shown as hex:");

    uint32_t count = 0;
    while (!Serial.available()) {
        while (mon.available()) {
            uint8_t b = mon.read();
            Serial.printf("%02X ", b);
            if (++count % 16 == 0) Serial.println();
        }
    }
    while (Serial.available()) Serial.read();
    Serial.printf("\n  Total bytes: %lu\n", count);
    Serial.println();
}

static void testTransmitPattern() {
    Serial.println("=== Transmit Pattern ===");
    Serial.println("  Select TX port: 1=RS485_1  2=RS485_2");
    int p = readSerialInt();
    if (p < 1 || p > 2) { Serial.println("  Invalid."); return; }

    HardwareSerial& tx = getPort((uint8_t)p);
    Serial.printf("  Sending 0xAA/0x55 pattern on %s — press any key to stop.\n",
                  portName((uint8_t)p));

    while (!Serial.available()) {
        tx.write((uint8_t)0xAA);
        tx.write((uint8_t)0x55);
        tx.flush();
        delayMicroseconds(100);
    }
    while (Serial.available()) Serial.read();
    Serial.println("  Stopped.");
    Serial.println();
}

static void testSendText() {
    Serial.println("=== Send Text Packet ===");
    Serial.println("  Select TX port: 1=RS485_1  2=RS485_2");
    int p = readSerialInt();
    if (p < 1 || p > 2) { Serial.println("  Invalid."); return; }

    HardwareSerial& tx = getPort((uint8_t)p);
    const char* msg = "PandaV2 RS485 test\n";
    tx.print(msg);
    tx.flush();
    Serial.printf("  Sent on %s: \"%s\"\n", portName((uint8_t)p), msg);
    Serial.println();
}

// ── Menu ─────────────────────────────────────────────────────────────

static void printMenu() {
    Serial.println("──────────────────────────────────");
    Serial.println("  PandaV2 RS-485 Test");
    Serial.println("──────────────────────────────────");
    Serial.println("  1  Baud rate info");
    Serial.println("  2  Port-to-port loopback (256 bytes)");
    Serial.println("  3  Burst throughput");
    Serial.println("  4  Stress test (100 random bytes)");
    Serial.println("  5  Live monitor");
    Serial.println("  6  Transmit 0xAA/0x55 pattern");
    Serial.println("  7  Send text packet");
    Serial.println("──────────────────────────────────");
    Serial.print("> ");
}

// ── Entry points ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(DEBUG_BAUD);
    while (!Serial && millis() < 3000) {}

    Serial1.begin(TEST_BAUD);
    Serial2.begin(TEST_BAUD);

    Serial.println("\nPandaV2 RS-485 Hardware Test");
    Serial.printf("RS485_1: Serial1  RX=%u TX=%u  baud=%lu\n",
                  PIN_RS485_1_RX, PIN_RS485_1_TX, TEST_BAUD);
    Serial.printf("RS485_2: Serial2  RX=%u TX=%u  baud=%lu\n",
                  PIN_RS485_2_RX, PIN_RS485_2_TX, TEST_BAUD);
    Serial.println("DE/RE: hardwired (no GPIO control)");
    Serial.println();

    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    int cmd = Serial.parseInt();
    switch (cmd) {
        case 1: testBaudCheck();         break;
        case 2: testLoopback();          break;
        case 3: testBurstThroughput();   break;
        case 4: testStress();            break;
        case 5: testMonitor();           break;
        case 6: testTransmitPattern();   break;
        case 7: testSendText();          break;
        default: break;
    }

    printMenu();
}
