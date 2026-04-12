#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "BoardConfig.h"
#include "pins.h"

#include <INA230.h>
#include <MCP3561RT.h>

#include "ArmingController.h"
#include "BangBang.h"
#include "CommsHandler.h"
#include "Scanner.h"
#include "SensorConfig.h"
#include "SequenceHandler.h"

// ── Hardware instances ──────────────────────────────────────────────

MCP3561RT adc1(PIN_ADC1_CS, PIN_ADC1_IRQ, SPI, SPI_ADC_SETTINGS, 1.25f);
MCP3561RT adc2(PIN_ADC2_CS, PIN_ADC2_IRQ, SPI1, SPI_ADC_SETTINGS, 1.25f);

INA230 pmon0(Wire, INA230_ADDR_U8);
INA230 pmon1(Wire, INA230_ADDR_U10);
INA230 pmon2(Wire, INA230_ADDR_U12);

static bool pmonOk[3] = {false, false, false};

// RS-485 comms — Serial7 = bus 1 (pins 28/29), Serial6 = bus 2 (pins 25/24)
CommsHandler comms(Serial7, PIN_RS485_1_DE);
CommsHandler comms2(Serial6, PIN_RS485_2_DE);

ArmingController arming(PIN_ARM, PIN_DISARM);
SequenceHandler seq;

// ── Scanner output buffers ──────────────────────────────────────────

float muxA_data[NUM_MUX_A_CH] = {0};
float muxB_data[NUM_MUX_B_CH] = {0};
float muxC_data[NUM_MUX_C_CH] = {0};

float ptData[NUM_PT_CH] = {0};
float lcData[NUM_LC_CH] = {0};
float tcData[NUM_TC_CH] = {0};
float curData[NUM_MUX_B_CH] = {0};
float boardTemp = 25.0f;

MuxBank adc1Banks[] = {
    {{PIN_MUX_A_S0, PIN_MUX_A_S1, PIN_MUX_A_S2, PIN_MUX_A_S3},
     NUM_MUX_A_CH,
     MCP3561RT::Mux::CH0,
     muxA_data,
     1.25f},
    {{PIN_MUX_B_S0, PIN_MUX_B_S1, PIN_MUX_B_S2, PIN_MUX_B_S3},
     NUM_MUX_B_CH,
     MCP3561RT::Mux::CH1,
     muxB_data,
     1.25f},
};

MuxBank adc2Banks[] = {
    {{PIN_MUX_C_S0, PIN_MUX_C_S1, PIN_MUX_C_S2, PIN_MUX_C_S3},
     NUM_MUX_C_CH,
     MCP3561RT::Mux::CH0,
     muxC_data,
     1.25f},
};

Scanner scanner1(adc1, adc1Banks, 2);
Scanner scanner2(adc2, adc2Banks, 1);

// ── Bang-bang controllers ───────────────────────────────────────────

BBController bbLox(&ptData[BB_LOX_PT_CH], BB_LOX_DC_CH, 'L');
BBController bbFuel(&ptData[BB_FUEL_PT_CH], BB_FUEL_DC_CH, 'F');

static bool bbSetChannel(uint8_t ch, bool state) {
  return seq.setChannel(ch, state);
}

// ── Command handling ────────────────────────────────────────────────
// Receives a packet and a reference to the bus it arrived on so
// responses go back on the same bus.

static void handleCommand(char *packet, CommsHandler &out) {
  if (!packet || packet[0] == '\0')
    return;

  char id = packet[0];

  switch (id) {

  case 'a': {
    auto result = arming.arm();
    if (result == ArmingController::Result::ACCEPTED)
      out.sendLine("ARM_ACK");
    else if (result == ArmingController::Result::IGNORED)
      out.sendLine("ARM_BUSY");
    else
      out.sendLine("ARM_ALREADY");
    break;
  }

  case 'r': {
    seq.cancelExecution();
    seq.setAllOff();
    bbLox.disableNow(bbSetChannel);
    bbFuel.disableNow(bbSetChannel);
    auto result = arming.disarm();
    out.sendLine("DISARM_ACK");
    if (seq.hasSequence()) {
      char buf[280];
      snprintf(buf, sizeof(buf), "SEQ_READY:%s", seq.getLastCommand());
      out.sendLine(buf);
    }
    (void)result;
    break;
  }

  case 's': {
    if (seq.setCommand(packet)) {
      char buf[300];
      snprintf(buf, sizeof(buf), "SEQ_ACK:count=%u,raw=%s", seq.getNumSteps(),
               seq.getLastCommand());
      out.sendLine(buf);
    } else {
      out.sendLine("SEQ_ERROR:parse_failed");
    }
    break;
  }

  case 'S': {
    if (!arming.isArmed()) {
      out.sendLine("CMD_ERROR:not_armed");
      break;
    }
    if (strlen(packet) < 3) {
      out.sendLine("CMD_ERROR:short_packet");
      break;
    }

    unsigned chan = 0, state = 0;
    char ch = packet[1];
    if (ch >= '0' && ch <= '9')
      chan = ch - '0';
    else if (ch >= 'A' && ch <= 'F')
      chan = 10 + (ch - 'A');
    else if (ch >= 'a' && ch <= 'f')
      chan = 10 + (ch - 'a');
    else {
      out.sendLine("CMD_ERROR:bad_channel");
      break;
    }

    state = packet[2] - '0';
    if (state > 1) {
      out.sendLine("CMD_ERROR:bad_state");
      break;
    }
    if (chan < 1 || chan > NUM_ACTUATORS) {
      out.sendLine("CMD_ERROR:chan_range");
      break;
    }

    seq.setChannel(chan, state);
    char ack[64];
    snprintf(ack, sizeof(ack), "ACT:%u:%s", chan, state ? "ON" : "OFF");
    out.sendLine(ack);
    break;
  }

  case 'f': {
    if (!seq.hasSequence()) {
      out.sendLine("SEQ_ERROR:no_sequence");
    } else if (!arming.isArmed()) {
      out.sendLine("SEQ_ERROR:not_armed");
    } else if (seq.execute()) {
      char buf[300];
      snprintf(buf, sizeof(buf), "SEQ_EXEC_START:count=%u,raw=%s",
               seq.getNumSteps(), seq.getLastCommand());
      out.sendLine(buf);
    } else {
      out.sendLine("SEQ_ERROR:exec_failed");
    }
    break;
  }

  case 'B': {
    if (strlen(packet) < 4) {
      out.sendLine("BB_ERROR:short");
      break;
    }
    char side = packet[1];
    if (side != 'L' && side != 'F') {
      out.sendLine("BB_ERROR:bad_bus");
      break;
    }

    float sp, db;
    unsigned long wt;
    if (sscanf(packet + 2, "%f,%f,%lu", &sp, &db, &wt) != 3 || sp < 0.0f ||
        db <= 0.0f || wt > 60000UL) {
      out.sendLine("BB_ERROR:parse");
      break;
    }

    BBController &ctrl = (side == 'L') ? bbLox : bbFuel;
    ctrl.configure(sp, db, (uint32_t)wt);
    bbSaveEeprom(bbLox, bbFuel);

    char ack[64];
    snprintf(ack, sizeof(ack), "BB_CFG:%c:%.1f:%.1f:%lu", side, sp, db, wt);
    out.sendLine(ack);
    break;
  }

  case 'b': {
    if (strlen(packet) < 3) {
      out.sendLine("BB_ERROR:short");
      break;
    }
    char side = packet[1];
    char stCh = packet[2];
    if ((side != 'L' && side != 'F') || (stCh != '0' && stCh != '1')) {
      out.sendLine("BB_ERROR:bad_arg");
      break;
    }

    BBController &ctrl = (side == 'L') ? bbLox : bbFuel;

    if (stCh == '1') {
      if (!arming.isArmed()) {
        out.sendLine("BB_ERROR:not_armed");
        break;
      }
      ctrl.enable();
      char ack[32];
      snprintf(ack, sizeof(ack), "BB:%c:ON", side);
      out.sendLine(ack);
    } else {
      ctrl.disableNow(bbSetChannel);
      char ack[32];
      snprintf(ack, sizeof(ack), "BB:%c:OFF", side);
      out.sendLine(ack);
    }
    break;
  }

  default:
    out.sendLine("CMD_ERROR:unknown");
    break;
  }
}

// ── Sensor conversion ───────────────────────────────────────────────

static elapsedMillis boardTempTimer;
static constexpr uint32_t BOARD_TEMP_INTERVAL_MS = 2000;

static void updateConversions() {
  if (boardTempTimer >= BOARD_TEMP_INTERVAL_MS) {
    boardTempTimer = 0;
    float t = scanner2.readBoardTemp();
    if (t > -900.0f)
      boardTemp = t;
  }

  for (uint8_t i = 0; i < NUM_PT_CH; i++)
    ptData[i] = convertPT(muxA_data[i], i);

  for (uint8_t i = 0; i < NUM_MUX_B_CH; i++)
    curData[i] = convertCurrent(muxB_data[i]);

  for (uint8_t i = 0; i < NUM_LC_CH; i++)
    lcData[i] = convertLC(muxC_data[MUX_C_LC_START + i], i);

  for (uint8_t i = 0; i < NUM_TC_CH; i++)
    tcData[i] = convertTC(muxC_data[MUX_C_TC_START + i], i, boardTemp);
}

// ── Telemetry output ────────────────────────────────────────────────

static elapsedMillis telemetryTimer;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;

static void sendTelemetry(CommsHandler &out) {
  char buf[512];

  CommsHandler::toCSVRow(ptData, 'p', NUM_PT_CH, buf, sizeof(buf));
  out.send(buf);

  CommsHandler::toCSVRow(curData, ID_SOLENOID_CURRENT, NUM_MUX_B_CH, buf,
                         sizeof(buf));
  out.send(buf);

  float lctcCombined[NUM_LC_CH + NUM_TC_CH];
  memcpy(lctcCombined, lcData, sizeof(float) * NUM_LC_CH);
  memcpy(lctcCombined + NUM_LC_CH, tcData, sizeof(float) * NUM_TC_CH);
  CommsHandler::toCSVRow(lctcCombined, ID_LCTC, NUM_LC_CH + NUM_TC_CH, buf,
                         sizeof(buf));
  out.send(buf);

  snprintf(buf, sizeof(buf), "BB:L:%d:%d:%.1f", bbLox.isEnabled() ? 1 : 0,
           bbLox.isValveOpen() ? 1 : 0, bbLox.lastPressure());
  out.sendLine(buf);

  snprintf(buf, sizeof(buf), "BB:F:%d:%d:%.1f", bbFuel.isEnabled() ? 1 : 0,
           bbFuel.isValveOpen() ? 1 : 0, bbFuel.lastPressure());
  out.sendLine(buf);
}

// ── Power monitoring telemetry ──────────────────────────────────────

static elapsedMillis powerTimer;
static constexpr uint32_t POWER_INTERVAL_MS = 500;

static void sendPowerTelemetry(CommsHandler &out) {
  char buf[128];
  INA230* pmons[3] = {&pmon0, &pmon1, &pmon2};
  float voltages[3] = {0};
  for (int i = 0; i < 3; i++) {
    if (pmonOk[i]) voltages[i] = pmons[i]->busVoltage_V();
  }
  CommsHandler::toCSVRow(voltages, ID_POWER, 3, buf, sizeof(buf));
  out.send(buf);
}

// ── Sequence completion reporting ───────────────────────────────────

static bool seqWasActive = false;

static void checkSequenceComplete(CommsHandler &out) {
  bool active = seq.isActive();
  if (seqWasActive && !active) {
    out.sendLine("SEQ_EXEC_COMPLETE");
    if (seq.hasSequence()) {
      char buf[280];
      snprintf(buf, sizeof(buf), "SEQ_READY:%s", seq.getLastCommand());
      out.sendLine(buf);
    }
  }
  seqWasActive = active;
}

// ── Arduino entry points ────────────────────────────────────────────

void setup() {
  Serial.begin(DEBUG_BAUD);

  if (CrashReport) {
    Serial.print(CrashReport);
    // Give time for USB serial to flush before continuing
    delay(2000);
  }

  SPI.begin();
  SPI1.begin();
  Wire.begin();
  Wire.setTimeout(2500); // 2.5ms max per I2C transaction (default is infinite)

  // Bring up both RS-485 buses immediately — any TX problem shows up on BOOT
  comms.begin(RS485_BAUD);
  comms2.begin(RS485_BAUD);

  // V2 PCB has TX differential pair (Y/Z) swapped on the RJ45.
  // RX pair (A/B) is correct — do NOT set RXINV.
  // TODO: fix Y/Z routing in next board revision
  //
  // Bus 1 (Serial7 → LPUART7)
  LPUART7_CTRL |= LPUART_CTRL_TXINV;
  // Bus 2 (Serial6 → LPUART1)
  LPUART1_CTRL |= LPUART_CTRL_TXINV;

  comms.sendLine("BOOT");
  comms2.sendLine("BOOT");

  if (CrashReport) {
    comms.sendLine("WARN:CRASH_DETECTED");
    comms2.sendLine("WARN:CRASH_DETECTED");
  }

  bool adc1_ok = adc1.begin();
  bool adc2_ok = adc2.begin();

  seq.begin();

  scanner1.begin();
  scanner2.begin();

  pmonOk[0] = pmon0.begin(0.1f, 5.0f);
  pmonOk[1] = pmon1.begin(0.1f, 5.0f);
  pmonOk[2] = pmon2.begin(0.1f, 5.0f);

  arming.begin();

  bbLoadEeprom(bbLox, bbFuel);

  comms.sendLine("PANDA_V2_INIT");
  comms2.sendLine("PANDA_V2_INIT");

  if (!adc1_ok) {
    comms.sendLine("WARN:ADC1_INIT_FAIL");
    comms2.sendLine("WARN:ADC1_INIT_FAIL");
  }
  if (!adc2_ok) {
    comms.sendLine("WARN:ADC2_INIT_FAIL");
    comms2.sendLine("WARN:ADC2_INIT_FAIL");
  }
  if (!pmonOk[0]) {
    comms.sendLine("WARN:PMON0_INIT_FAIL");
    comms2.sendLine("WARN:PMON0_INIT_FAIL");
  }
  if (!pmonOk[1]) {
    comms.sendLine("WARN:PMON1_INIT_FAIL");
    comms2.sendLine("WARN:PMON1_INIT_FAIL");
  }
  if (!pmonOk[2]) {
    comms.sendLine("WARN:PMON2_INIT_FAIL");
    comms2.sendLine("WARN:PMON2_INIT_FAIL");
  }

  Serial.println("PandaV2 ready");
}

void loop() {
  // 1. Drain all queued commands
  for (int i = 0; i < NUM_MAX_COMMANDS; i++) {
    comms.poll();
    if (!comms.isPacketReady()) break;
    handleCommand(comms.takePacket(), comms);
  }
  for (int i = 0; i < NUM_MAX_COMMANDS; i++) {
    comms2.poll();
    if (!comms2.isPacketReady()) break;
    handleCommand(comms2.takePacket(), comms2);
  }

  // 2. State machines
  arming.update();
  seq.update();

  // 3. ADC scanning
  scanner1.update();
  scanner2.update();

  // 4. Sensor conversions
  updateConversions();

  // 5. Bang-bang
  bbLox.update(arming.isArmed(), bbSetChannel);
  bbFuel.update(arming.isArmed(), bbSetChannel);

  // 6. Telemetry
  if (telemetryTimer >= TELEMETRY_INTERVAL_MS) {
    telemetryTimer = 0;
    sendTelemetry(comms);
    sendTelemetry(comms2);
  }

  if (powerTimer >= POWER_INTERVAL_MS) {
    powerTimer = 0;
    sendPowerTelemetry(comms);
    sendPowerTelemetry(comms2);
  }

  // 7. Sequence completion
  checkSequenceComplete(comms);
  checkSequenceComplete(comms2);

  // Rate cap: ~10kHz is plenty for 50ms telemetry + 500µs ADC settle.
  // Avoids hammering ISRs and power rail at 600kHz.
  delayMicroseconds(100);
}
