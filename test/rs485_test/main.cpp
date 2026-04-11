#include <Arduino.h>

// Bare-minimum RS-485 TX test for V2 board
// Serial7 = bus 1 (pins 28 TX / 29 RX), ISL83491 DE hardwired +3V3
// Sends "hello" at 1 Hz on bus 1. If GC receives clean lines, hardware is good.

static constexpr uint32_t BAUD = 460800;
static uint32_t counter = 0;

void setup() {
    Serial.begin(115200);
    Serial7.begin(BAUD);

    // V2 PCB has Y/Z (TX diff pair) swapped on the RJ45.
    // Invert UART TX signal at the peripheral to compensate.
    LPUART7_CTRL |= LPUART_CTRL_TXINV;

    Serial.println("RS485 TX test -- Serial7 @ 460800 (TXINV)");
    Serial7.println("BOOT");
}

void loop() {
    Serial7.print("hello ");
    Serial7.println(counter++);
    Serial.print("sent: hello ");
    Serial.println(counter - 1);
    delay(1000);
}
