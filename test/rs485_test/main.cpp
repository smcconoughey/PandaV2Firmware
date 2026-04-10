// RS-485 Hardware Test — PandaV2
// Single-port focused: TX pattern, live RX monitor, packet send/receive.
// Serial7 = RS485_1 (flight computer link), Serial6 = RS485_2.
// DE is hardwired to +3V3; no GPIO control needed.

#include <Arduino.h>
#include "pins.h"
#include "BoardConfig.h"

// ── Configuration ────────────────────────────────────────────────────

static constexpr uint32_t TEST_BAUD     = RS485_BAUD;
static constexpr uint8_t  LED_PIN       = 13;   // Teensy 4.1 built-in LED

// ── Helpers ──────────────────────────────────────────────────────────

static HardwareSerial& getPort(uint8_t p) {
    return (p == 1) ? Serial7 : Serial6;
}

static const char* portLabel(uint8_t p) {
    return (p == 1) ? "Serial7 (RS485_1)" : "Serial6 (RS485_2)";
}

static int readSerialInt() {
    // Non-blocking poll with LED heartbeat so the board visibly stays alive
    while (!Serial.available()) {
        digitalWrite(LED_PIN, HIGH); delay(100);
        digitalWrite(LED_PIN, LOW);  delay(100);
    }
    return Serial.parseInt();
}

static uint8_t pickPort() {
    Serial.println("  Port: 1=RS485_1 (Serial7)  2=RS485_2 (Serial6)");
    Serial.print("  > ");
    int p = readSerialInt();
    if (p < 1 || p > 2) { Serial.println("Invalid."); return 0; }
    return (uint8_t)p;
}

// ── Tests ─────────────────────────────────────────────────────────────

// Continuously send 0xAA 0x55 — confirm with scope / logic analyser.
// Press any key to stop.
static void testTxPattern() {
    Serial.println("=== TX Pattern (0xAA/0x55) ===");
    uint8_t p = pickPort();
    if (!p) return;
    HardwareSerial& port = getPort(p);

    Serial.printf("  Sending on %s — press any key to stop.\n", portLabel(p));
    uint32_t count = 0;
    while (!Serial.available()) {
        port.write((uint8_t)0xAA);
        port.write((uint8_t)0x55);
        port.flush();
        count += 2;
        if (count % 1000 == 0) {
            Serial.printf("  %lu bytes sent\n", count);
        }
    }
    while (Serial.available()) Serial.read();
    Serial.printf("  Stopped. Total: %lu bytes.\n", count);
    Serial.println();
}

// Send an incrementing counter byte every 100 ms.
static void testTxCounter() {
    Serial.println("=== TX Counter ===");
    uint8_t p = pickPort();
    if (!p) return;
    HardwareSerial& port = getPort(p);

    Serial.printf("  Sending incrementing bytes on %s — press any key to stop.\n", portLabel(p));
    uint8_t val = 0;
    while (!Serial.available()) {
        port.write(val);
        port.flush();
        Serial.printf("  TX: 0x%02X (%3u)\n", val, val);
        val++;
        delay(100);
    }
    while (Serial.available()) Serial.read();
    Serial.println();
}

// Send a text string and wait to see it echo back (requires external loopback or
// a device that echoes).
static void testTxText() {
    Serial.println("=== Send Text Packet ===");
    uint8_t p = pickPort();
    if (!p) return;
    HardwareSerial& port = getPort(p);

    const char* msg = "PandaV2 RS485 test\r\n";
    port.print(msg);
    port.flush();
    Serial.printf("  Sent on %s: \"%s\"\n", portLabel(p), msg);

    // Wait briefly for any echo
    uint32_t deadline = millis() + 500;
    bool gotReply = false;
    Serial.print("  RX echo: ");
    while (millis() < deadline) {
        while (port.available()) {
            uint8_t b = port.read();
            Serial.printf("0x%02X('%c') ", b, isprint(b) ? b : '.');
            gotReply = true;
        }
    }
    if (!gotReply) Serial.print("(none)");
    Serial.println();
    Serial.println();
}

// Monitor incoming bytes on a port — print as hex + ASCII.
static void testRxMonitor() {
    Serial.println("=== RX Monitor ===");
    uint8_t p = pickPort();
    if (!p) return;
    HardwareSerial& port = getPort(p);

    Serial.printf("  Listening on %s — press any key to stop.\n", portLabel(p));
    uint32_t total = 0;
    uint8_t col = 0;
    while (!Serial.available()) {
        while (port.available()) {
            uint8_t b = port.read();
            Serial.printf("%02X ", b);
            total++;
            col++;
            if (col == 16) { Serial.println(); col = 0; }
        }
    }
    while (Serial.available()) Serial.read();
    if (col) Serial.println();
    Serial.printf("  Total bytes received: %lu\n", total);
    Serial.println();
}

// Monitor both ports simultaneously.
static void testRxBoth() {
    Serial.println("=== RX Both Ports ===");
    Serial.println("  Monitoring Serial7 and Serial6 — press any key to stop.");
    uint32_t totals[2] = {0, 0};
    while (!Serial.available()) {
        for (uint8_t p = 1; p <= 2; p++) {
            HardwareSerial& port = getPort(p);
            while (port.available()) {
                uint8_t b = port.read();
                Serial.printf("  [%s] 0x%02X (%3u)\n", portLabel(p), b, b);
                totals[p-1]++;
            }
        }
    }
    while (Serial.available()) Serial.read();
    Serial.printf("  Serial7: %lu bytes  Serial6: %lu bytes\n", totals[0], totals[1]);
    Serial.println();
}

// Print baud / pin info.
static void testInfo() {
    Serial.println("=== RS-485 Config ===");
    Serial.printf("  RS485_1: Serial7  baud=%lu\n", TEST_BAUD);
    Serial.printf("  RS485_2: Serial6  baud=%lu\n", TEST_BAUD);
    Serial.println("  DE: hardwired +3V3 (always driving)");
    Serial.println("  /RE: hardwired GND (receiver always enabled)");
    Serial.println("  NOTE: always-drive means point-to-point only.");
    Serial.println();
}

// ── Menu ─────────────────────────────────────────────────────────────

static void printMenu() {
    Serial.println("──────────────────────────────────");
    Serial.println("  PandaV2 RS-485 Test");
    Serial.println("──────────────────────────────────");
    Serial.println("  1  Config info");
    Serial.println("  2  TX: 0xAA/0x55 pattern");
    Serial.println("  3  TX: incrementing counter");
    Serial.println("  4  TX: text packet");
    Serial.println("  5  RX: monitor one port");
    Serial.println("  6  RX: monitor both ports");
    Serial.println("──────────────────────────────────");
    Serial.print("> ");
}

// ── Entry points ─────────────────────────────────────────────────────

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Blink 3× to confirm code is running before USB serial connects
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(200);
        digitalWrite(LED_PIN, LOW);  delay(200);
    }

    Serial.begin(DEBUG_BAUD);
    while (!Serial && millis() < 3000) {}

    Serial7.begin(TEST_BAUD);
    Serial6.begin(TEST_BAUD);

    Serial.println("\nPandaV2 RS-485 Hardware Test");
    testInfo();
    printMenu();
}

void loop() {
    if (!Serial.available()) {
        // Heartbeat — slow blink while idle so you can see the board is alive
        static elapsedMillis heartbeat;
        if (heartbeat > 1000) {
            heartbeat = 0;
            digitalWrite(LED_PIN, HIGH); delay(50);
            digitalWrite(LED_PIN, LOW);
        }
        return;
    }

    int cmd = Serial.parseInt();
    switch (cmd) {
        case 1: testInfo();       break;
        case 2: testTxPattern();  break;
        case 3: testTxCounter();  break;
        case 4: testTxText();     break;
        case 5: testRxMonitor();  break;
        case 6: testRxBoth();     break;
        default: break;
    }

    printMenu();
}
