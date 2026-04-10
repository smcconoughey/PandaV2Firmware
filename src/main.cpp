#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "pins.h"
#include "BoardConfig.h"

#include <MCP3561RT.h>
#include <INA230.h>

#include "Scanner.h"
#include "SensorConfig.h"
#include "CommsHandler.h"
#include "SequenceHandler.h"
#include "ArmingController.h"
#include "BangBang.h"

// ── Hardware instances ──────────────────────────────────────────────

// ADCs (24-bit, SPI)
MCP3561RT adc1(PIN_ADC1_CS, PIN_ADC1_IRQ, SPI, SPI_ADC_SETTINGS, 1.25f);
MCP3561RT adc2(PIN_ADC2_CS, PIN_ADC2_IRQ, SPI1, SPI_ADC_SETTINGS, 1.25f);

// Power monitors (I2C)
INA230 pmon0(Wire, INA230_ADDR_U8);
INA230 pmon1(Wire, INA230_ADDR_U10);
INA230 pmon2(Wire, INA230_ADDR_U12);

// RS-485 comms (Serial7 = bus 1, Serial8 = bus 2)
CommsHandler comms(Serial7, PIN_RS485_1_DE);
CommsHandler comms2(Serial6, PIN_RS485_2_DE);


// Arming — pin assignments are placeholders, verify on V2 hardware
ArmingController arming(PIN_ARM, PIN_DISARM);

// Sequence handler (drives solenoids via DC_PINS GPIO)
SequenceHandler seq;

// ── Scanner output buffers ──────────────────────────────────────────

float muxA_data[NUM_MUX_A_CH] = {0};
float muxB_data[NUM_MUX_B_CH] = {0};
float muxC_data[NUM_MUX_C_CH] = {0};

// Converted sensor output buffers
float ptData[NUM_PT_CH]   = {0};   // PTs (converted from muxA)
float lcData[NUM_LC_CH]   = {0};   // load cells (converted from muxC 0..7)
float tcData[NUM_TC_CH]   = {0};   // thermocouples (converted from muxC 8..15)
float curData[NUM_MUX_B_CH] = {0}; // solenoid current (converted from muxB)
float boardTemp = 25.0f;           // CJC reference from ADC internal sensor

// Mux banks for ADC1 (mux A = PTs, mux B = current sense)
MuxBank adc1Banks[] = {
    {{PIN_MUX_A_S0, PIN_MUX_A_S1, PIN_MUX_A_S2, PIN_MUX_A_S3},
     NUM_MUX_A_CH, MCP3561RT::Mux::CH0, muxA_data, 1.25f},
    {{PIN_MUX_B_S0, PIN_MUX_B_S1, PIN_MUX_B_S2, PIN_MUX_B_S3},
     NUM_MUX_B_CH, MCP3561RT::Mux::CH1, muxB_data, 3.3f},
};

// Mux bank for ADC2 (mux C = LCs + TCs)
MuxBank adc2Banks[] = {
    {{PIN_MUX_C_S0, PIN_MUX_C_S1, PIN_MUX_C_S2, PIN_MUX_C_S3},
     NUM_MUX_C_CH, MCP3561RT::Mux::CH0, muxC_data, 1.25f},
};

Scanner scanner1(adc1, adc1Banks, 2);
Scanner scanner2(adc2, adc2Banks, 1);

// ── Bang-bang controllers ───────────────────────────────────────────
// Hardwired: LOX reads ptData[BB_LOX_PT_CH], drives DC BB_LOX_DC_CH.
//            Fuel reads ptData[BB_FUEL_PT_CH], drives DC BB_FUEL_DC_CH.
// Config is loaded from EEPROM on boot; both start DISABLED.
// Do not command BB solenoid channels via 'S' while BB is enabled.

BBController bbLox (&ptData[BB_LOX_PT_CH],  BB_LOX_DC_CH,  'L');
BBController bbFuel(&ptData[BB_FUEL_PT_CH], BB_FUEL_DC_CH, 'F');

// Thin wrapper so BBController can drive solenoids without a SequenceHandler dep
static bool bbSetChannel(uint8_t ch, bool state) {
    return seq.setChannel(ch, state);
}

// ── Command handling ────────────────────────────────────────────────

static void handleCommand(char* packet) {
    if (!packet || packet[0] == '\0') return;

    char id = packet[0];

    switch (id) {

    case 'a': {
        auto result = arming.arm();
        if (result == ArmingController::Result::ACCEPTED)
            comms.sendLine("ARM_ACK");
        else if (result == ArmingController::Result::IGNORED)
            comms.sendLine("ARM_BUSY");
        else
            comms.sendLine("ARM_ALREADY");
        break;
    }

    case 'r': {
        // Disarm: always safe. Cancel sequence, kill all outputs, disable BB.
        seq.cancelExecution();
        seq.setAllOff();
        bbLox.disableNow(bbSetChannel);
        bbFuel.disableNow(bbSetChannel);
        auto result = arming.disarm();
        comms.sendLine("DISARM_ACK");
        if (seq.hasSequence()) {
            char buf[280];
            snprintf(buf, sizeof(buf), "SEQ_READY:%s", seq.getLastCommand());
            comms.sendLine(buf);
        }
        (void)result;
        break;
    }

    case 's': {
        // Sequence load: "s11.01000,s10.00000,..."
        if (seq.setCommand(packet)) {
            char buf[300];
            snprintf(buf, sizeof(buf), "SEQ_ACK:count=%u,raw=%s",
                     seq.getNumSteps(), seq.getLastCommand());
            comms.sendLine(buf);
        } else {
            comms.sendLine("SEQ_ERROR:parse_failed");
        }
        break;
    }

    case 'S': {
        // Direct actuator: "S<chan_hex><state>"
        if (!arming.isArmed()) {
            comms.sendLine("CMD_ERROR:not_armed");
            break;
        }
        if (strlen(packet) < 3) {
            comms.sendLine("CMD_ERROR:short_packet");
            break;
        }

        unsigned chan = 0, state = 0;
        char ch = packet[1];
        if (ch >= '0' && ch <= '9') chan = ch - '0';
        else if (ch >= 'A' && ch <= 'F') chan = 10 + (ch - 'A');
        else if (ch >= 'a' && ch <= 'f') chan = 10 + (ch - 'a');
        else {
            comms.sendLine("CMD_ERROR:bad_channel");
            break;
        }

        state = packet[2] - '0';
        if (state > 1) {
            comms.sendLine("CMD_ERROR:bad_state");
            break;
        }

        if (chan < 1 || chan > NUM_ACTUATORS) {
            comms.sendLine("CMD_ERROR:chan_range");
            break;
        }

        seq.setChannel(chan, state);
        char ack[64];
        snprintf(ack, sizeof(ack), "ACT:%u:%s", chan, state ? "ON" : "OFF");
        comms.sendLine(ack);
        break;
    }

    case 'f': {
        // Fire/execute loaded sequence
        if (!seq.hasSequence()) {
            comms.sendLine("SEQ_ERROR:no_sequence");
        } else if (!arming.isArmed()) {
            comms.sendLine("SEQ_ERROR:not_armed");
        } else if (seq.execute()) {
            char buf[300];
            snprintf(buf, sizeof(buf), "SEQ_EXEC_START:count=%u,raw=%s",
                     seq.getNumSteps(), seq.getLastCommand());
            comms.sendLine(buf);
        } else {
            comms.sendLine("SEQ_ERROR:exec_failed");
        }
        break;
    }

    case 'B': {
        // Configure bang-bang: "BL<setpoint>,<deadband>,<wait_ms>"
        //                      "BF<setpoint>,<deadband>,<wait_ms>"
        if (strlen(packet) < 4) {
            comms.sendLine("BB_ERROR:short");
            break;
        }
        char side = packet[1];
        if (side != 'L' && side != 'F') {
            comms.sendLine("BB_ERROR:bad_bus");
            break;
        }

        float sp, db;
        unsigned long wt;
        if (sscanf(packet + 2, "%f,%f,%lu", &sp, &db, &wt) != 3
                || sp < 0.0f || db <= 0.0f || wt > 60000UL) {
            comms.sendLine("BB_ERROR:parse");
            break;
        }

        BBController& ctrl = (side == 'L') ? bbLox : bbFuel;
        ctrl.configure(sp, db, (uint32_t)wt);
        bbSaveEeprom(bbLox, bbFuel);

        char ack[64];
        snprintf(ack, sizeof(ack), "BB_CFG:%c:%.1f:%.1f:%lu", side, sp, db, wt);
        comms.sendLine(ack);
        break;
    }

    case 'b': {
        // Enable/disable bang-bang: "bL1", "bL0", "bF1", "bF0"
        if (strlen(packet) < 3) {
            comms.sendLine("BB_ERROR:short");
            break;
        }
        char side  = packet[1];
        char stCh  = packet[2];
        if ((side != 'L' && side != 'F') || (stCh != '0' && stCh != '1')) {
            comms.sendLine("BB_ERROR:bad_arg");
            break;
        }

        BBController& ctrl = (side == 'L') ? bbLox : bbFuel;

        if (stCh == '1') {
            if (!arming.isArmed()) {
                comms.sendLine("BB_ERROR:not_armed");
                break;
            }
            ctrl.enable();
            char ack[32];
            snprintf(ack, sizeof(ack), "BB:%c:ON", side);
            comms.sendLine(ack);
        } else {
            ctrl.disableNow(bbSetChannel);
            char ack[32];
            snprintf(ack, sizeof(ack), "BB:%c:OFF", side);
            comms.sendLine(ack);
        }
        break;
    }

    default:
        comms.sendLine("CMD_ERROR:unknown");
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
        if (t > -900.0f) boardTemp = t;
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

static void sendTelemetry() {
    if (telemetryTimer < TELEMETRY_INTERVAL_MS) return;
    telemetryTimer = 0;

    char buf[512];

    CommsHandler::toCSVRow(ptData, 'p', NUM_PT_CH, buf, sizeof(buf));
    comms.send(buf);

    CommsHandler::toCSVRow(curData, ID_SOLENOID_CURRENT, NUM_MUX_B_CH, buf, sizeof(buf));
    comms.send(buf);

    float lctcCombined[NUM_LC_CH + NUM_TC_CH];
    memcpy(lctcCombined, lcData, sizeof(float) * NUM_LC_CH);
    memcpy(lctcCombined + NUM_LC_CH, tcData, sizeof(float) * NUM_TC_CH);
    CommsHandler::toCSVRow(lctcCombined, ID_LCTC, NUM_LC_CH + NUM_TC_CH, buf, sizeof(buf));
    comms.send(buf);

    // BB status: "BB:<bus>:<enabled>:<valve_open>:<pressure_psi>"
    snprintf(buf, sizeof(buf), "BB:L:%d:%d:%.1f",
        bbLox.isEnabled()   ? 1 : 0,
        bbLox.isValveOpen() ? 1 : 0,
        bbLox.lastPressure());
    comms.sendLine(buf);

    snprintf(buf, sizeof(buf), "BB:F:%d:%d:%.1f",
        bbFuel.isEnabled()   ? 1 : 0,
        bbFuel.isValveOpen() ? 1 : 0,
        bbFuel.lastPressure());
    comms.sendLine(buf);
}

// ── Power monitoring telemetry ──────────────────────────────────────

static elapsedMillis powerTimer;
static constexpr uint32_t POWER_INTERVAL_MS = 500;

static void sendPowerTelemetry() {
    if (powerTimer < POWER_INTERVAL_MS) return;
    powerTimer = 0;

    char buf[128];
    float voltages[3] = {
        pmon0.busVoltage_V(),
        pmon1.busVoltage_V(),
        pmon2.busVoltage_V()
    };
    CommsHandler::toCSVRow(voltages, ID_POWER, 3, buf, sizeof(buf));
    comms.send(buf);
}

// ── Sequence completion reporting ───────────────────────────────────

static bool seqWasActive = false;

static void checkSequenceComplete() {
    bool active = seq.isActive();
    if (seqWasActive && !active) {
        comms.sendLine("SEQ_EXEC_COMPLETE");
        if (seq.hasSequence()) {
            char buf[280];
            snprintf(buf, sizeof(buf), "SEQ_READY:%s", seq.getLastCommand());
            comms.sendLine(buf);
        }
    }
    seqWasActive = active;
}

// ── Arduino entry points ────────────────────────────────────────────

void setup() {
    Serial.begin(DEBUG_BAUD);

    SPI.begin();
    SPI1.begin();
    Wire.begin();

    comms.begin(RS485_BAUD);

    bool adc1_ok = adc1.begin();
    bool adc2_ok = adc2.begin();

    seq.begin();

    scanner1.begin();
    scanner2.begin();

    bool pm0_ok = pmon0.begin(0.1f, 5.0f);
    bool pm1_ok = pmon1.begin(0.1f, 5.0f);
    bool pm2_ok = pmon2.begin(0.1f, 5.0f);

    arming.begin();

    // Load BB config from EEPROM (both controllers start disabled regardless)
    bbLoadEeprom(bbLox, bbFuel);

    comms.sendLine("PANDA_V2_INIT");

    if (!adc1_ok) comms.sendLine("WARN:ADC1_INIT_FAIL");
    if (!adc2_ok) comms.sendLine("WARN:ADC2_INIT_FAIL");
    if (!pm0_ok)  comms.sendLine("WARN:PMON0_INIT_FAIL");
    if (!pm1_ok)  comms.sendLine("WARN:PMON1_INIT_FAIL");
    if (!pm2_ok)  comms.sendLine("WARN:PMON2_INIT_FAIL");

    Serial.println("PandaV2 ready");
}

void loop() {
    // 1. Receive commands
    comms.poll();
    if (comms.isPacketReady()) {
        char* pkt = comms.takePacket();
        handleCommand(pkt);
    }

    // 2. State machines
    arming.update();
    seq.update();

    // 3. ADC scanning
    scanner1.update();
    scanner2.update();

    // 4. Sensor conversions (raw voltage → engineering units)
    updateConversions();

    // 5. Bang-bang control loops (run after conversions so ptData is current)
    bbLox.update(arming.isArmed(), bbSetChannel);
    bbFuel.update(arming.isArmed(), bbSetChannel);

    // 6. Telemetry
    sendTelemetry();
    sendPowerTelemetry();

    // 7. Sequence completion detection
    checkSequenceComplete();
}
