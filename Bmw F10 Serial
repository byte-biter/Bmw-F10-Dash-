/*
 * BMW F10 2012 CAN Bus Controller + Physical Shifter Reader
 * ESP32-S3 + External MCP2515 (8MHz crystal)
 *
 * Wiring:
 * MCP2515 SCK  -> GPIO 36
 * MCP2515 MISO -> GPIO 37
 * MCP2515 MOSI -> GPIO 35
 * MCP2515 CS   -> GPIO 39
 * MCP2515 INT  -> GPIO 40   (physical shifter interrupt)
 *
 * Diesel F10 — all warning lights OFF by default
 * Rear seatbelt warning disabled
 *
 * Physical shifter state is read from CAN and synced into currentGear so
 * all cluster messages (0x3FD, 0x0F3, etc.) stay consistent automatically.
 *
 * NOTE: setShifterStatus() and sendTransmission() both write to 0x3FD but
 * with different payloads.  setShifterStatus() is the handshake the physical
 * shifter module expects; sendTransmission() is the cluster gear-display
 * message.  Both are needed on the same bus.
 *
 * KOMBI CODING: For M/S individual gear display (S1, S2, etc. instead of
 * just "DS"), the KOMBI must be coded via E-Sys or BimmerCode:
 *   SPA_SPORT_ENABLE  = aktiv
 *   SPA_IST_GANG_ENABLE = aktiv
 */

#include <SPI.h>
#include "mcp_can.h"

// ── Pin definitions ───────────────────────────────────────────────────────────
#define SPI_SCK   36
#define SPI_MISO  37
#define SPI_MOSI  35
#define SPI_CS    39
#define CAN_INT   40   // Physical shifter interrupt pin

MCP_CAN CAN(SPI_CS);

// ── Cluster CAN counters ──────────────────────────────────────────────────────
uint8_t counter4Bit = 0;
uint8_t count       = 0;
uint8_t accCounter  = 0;

// ── Vehicle state (controlled via Serial or physical shifter) ─────────────────
int     currentSpeed       = 0;
int     currentRPM         = 1000;
int     currentGear        = 10;   // 10=P 11=R 12=N 13=D  1-9=manual gears
int     manualGear         = 1;    // Current gear in M/S mode (1-8)
int     engineTemperature  = 100;
uint8_t driveMode          = 2;    // 2=Comfort
int     fuelLevel          = 100;

// ── Cruise control ────────────────────────────────────────────────────────────
bool cruiseControlActive = false;
int  cruiseSetSpeed      = 0;

// ── Shifter backlight refresh counter ─────────────────────────────────────────
uint8_t backlightCycleCount = 0;

// ── Ignition & brightness (from CarCluster) ──────────────────────────────────
bool    ignitionOn          = true;
uint8_t backlightBrightness = 100;   // 0-100%
uint8_t clockHours          = 12;
uint8_t clockMinutes        = 0;
uint8_t clockSeconds        = 0;
uint8_t dateDay             = 1;
uint8_t dateMonth           = 1;
uint16_t dateYear           = 2026;
unsigned long lastSlowUpdate = 0;
int overspeedThreshold       = 120;  // km/h, 0 = disabled

// ── UDS response buffer (global to avoid stack overflow) ────────────────────
uint8_t udsFullResp[64];
int     udsFullLen = 0;

// ── Lane assistant ──────────────────────────────────────────────────────────
bool    laneAssistActive    = false;
uint8_t laneLeftStatus      = 0x03;   // 0x00=off, 0x01=grey, 0x02=green, 0x03=yellow
uint8_t laneRightStatus     = 0x03;   // same encoding — needs bench testing

// ── Warning lights — all OFF ──────────────────────────────────────────────────
bool doorOpen         = false;
bool dscAlert         = false;
bool handbrake        = false;
bool checkEngine      = false;
bool dscOff           = false;
bool parkBrake        = false;
bool sosCall          = false;
bool chassisWarning   = false;
bool cruiseWarning    = false;
bool brakeFailure     = false;
bool dippedBeamFailure = false;
bool trailerReversing = false;
bool restraintRear    = false;
bool fastenSeatbelts  = false;
bool warning73        = false;
bool warning85        = false;
bool warning88        = false;

// ── Lights ────────────────────────────────────────────────────────────────────
bool mainLights    = true;
bool highBeam      = false;
bool rearFogLight  = false;
bool frontFogLight = false;
bool leftBlinker   = false;
bool rightBlinker  = false;

// ── Fuel map (F10 Diesel) ─────────────────────────────────────────────────────
uint8_t inFuelRange[3]  = {0, 50, 100};
uint8_t outFuelRange[3] = {37, 18, 4};

// ── Distance counter ──────────────────────────────────────────────────────────
uint32_t distanceTravelledCounter = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  CRC8 — cluster messages (computed table, poly 0x1D)
// ─────────────────────────────────────────────────────────────────────────────
class CRC8 {
  uint8_t crcTable[256];
public:
  CRC8() {
    for (int d = 0; d < 256; ++d) {
      uint8_t rem = (uint8_t)d;
      for (uint8_t b = 8; b > 0; --b)
        rem = (rem & 0x80) ? (rem << 1) ^ 0x1D : (rem << 1);
      crcTable[d] = rem;
    }
  }
  uint8_t get_crc8(uint8_t const msg[], int n, uint8_t fin) {
    uint8_t rem = 0xFF;
    for (int i = 0; i < n; ++i) rem = crcTable[msg[i] ^ rem];
    return rem ^ fin;
  }
} crc8Calculator;

// ─────────────────────────────────────────────────────────────────────────────
//  CRC8 — shifter module (lookup table, XOR 0x70)
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t shifter_crc8_lut[] = {
    0x00,0x1d,0x3a,0x27,0x74,0x69,0x4e,0x53,0xe8,0xf5,0xd2,0xcf,0x9c,0x81,0xa6,0xbb,
    0xcd,0xd0,0xf7,0xea,0xb9,0xa4,0x83,0x9e,0x25,0x38,0x1f,0x02,0x51,0x4c,0x6b,0x76,
    0x87,0x9a,0xbd,0xa0,0xf3,0xee,0xc9,0xd4,0x6f,0x72,0x55,0x48,0x1b,0x06,0x21,0x3c,
    0x4a,0x57,0x70,0x6d,0x3e,0x23,0x04,0x19,0xa2,0xbf,0x98,0x85,0xd6,0xcb,0xec,0xf1,
    0x13,0x0e,0x29,0x34,0x67,0x7a,0x5d,0x40,0xfb,0xe6,0xc1,0xdc,0x8f,0x92,0xb5,0xa8,
    0xde,0xc3,0xe4,0xf9,0xaa,0xb7,0x90,0x8d,0x36,0x2b,0x0c,0x11,0x42,0x5f,0x78,0x65,
    0x94,0x89,0xae,0xb3,0xe0,0xfd,0xda,0xc7,0x7c,0x61,0x46,0x5b,0x08,0x15,0x32,0x2f,
    0x59,0x44,0x63,0x7e,0x2d,0x30,0x17,0x0a,0xb1,0xac,0x8b,0x96,0xc5,0xd8,0xff,0xe2,
    0x26,0x3b,0x1c,0x01,0x52,0x4f,0x68,0x75,0xce,0xd3,0xf4,0xe9,0xba,0xa7,0x80,0x9d,
    0xeb,0xf6,0xd1,0xcc,0x9f,0x82,0xa5,0xb8,0x03,0x1e,0x39,0x24,0x77,0x6a,0x4d,0x50,
    0xa1,0xbc,0x9b,0x86,0xd5,0xc8,0xef,0xf2,0x49,0x54,0x73,0x6e,0x3d,0x20,0x07,0x1a,
    0x6c,0x71,0x56,0x4b,0x18,0x05,0x22,0x3f,0x84,0x99,0xbe,0xa3,0xf0,0xed,0xca,0xd7,
    0x35,0x28,0x0f,0x12,0x41,0x5c,0x7b,0x66,0xdd,0xc0,0xe7,0xfa,0xa9,0xb4,0x93,0x8e,
    0xf8,0xe5,0xc2,0xdf,0x8c,0x91,0xb6,0xab,0x10,0x0d,0x2a,0x37,0x64,0x79,0x5e,0x43,
    0xb2,0xaf,0x88,0x95,0xc6,0xdb,0xfc,0xe1,0x5a,0x47,0x60,0x7d,0x2e,0x33,0x14,0x09,
    0x7f,0x62,0x45,0x58,0x0b,0x16,0x31,0x2c,0x97,0x8a,0xad,0xb0,0xe3,0xfe,0xd9,0xc4
};

static uint8_t shifter_crc8(uint8_t crc, const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++)
        crc = shifter_crc8_lut[buf[i] ^ crc] ^ (crc << 8);
    return crc ^ 0x70;
}

// ── Utility ───────────────────────────────────────────────────────────────────
template <typename T>
T multiMap(T val, uint8_t *_in, uint8_t *_out, uint8_t size) {
    if (val <= _in[0])        return _out[0];
    if (val >= _in[size - 1]) return _out[size - 1];
    uint8_t pos = 1;
    while (val > _in[pos]) pos++;
    if (val == _in[pos]) return _out[pos];
    return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

// ─────────────────────────────────────────────────────────────────────────────
//  PHYSICAL SHIFTER — state & helpers
// ─────────────────────────────────────────────────────────────────────────────
const byte SHIFTER_PARKING = 0x20;
const byte SHIFTER_REVERSE = 0x40;
const byte SHIFTER_NEUTRAL = 0x60;
const byte SHIFTER_DRIVE   = 0x81;

const String POS_MIDDLE           = "0x0F";
const String POS_MIDDLE_MOVE_DOWN = "0x3F";
const String POS_MIDDLE_MOVE_UP   = "0x1F";
const String POS_SIDE             = "0x7F";
const String POS_SIDE_MOVE_DOWN   = "0x6F";
const String POS_SIDE_MOVE_UP     = "0x5F";
const String POS_PARKING_POS      = "0xD5";
const String POS_PARKING_RELEASED = "0xC0";

// Auto gear schema  R=0, N=1, D=2
const byte shifter_schema[3]      = {SHIFTER_REVERSE, SHIFTER_NEUTRAL, SHIFTER_DRIVE};
uint8_t    shifter_schema_pos     = 1;
byte       shifter_status_current = SHIFTER_NEUTRAL;

bool    shifter_init_done      = false;
bool    shifter_released       = true;
bool    parking_btn_released   = true;
bool    parking_engaged        = false;
bool    shifter_mode_auto      = true;
uint8_t shifter_status_counter = 0;

long unsigned int rxId  = 0;
unsigned char     rxLen = 0;
unsigned char     rxBuf[8];
char msgString[128];
char lever_pos_code[16];
char parking_pos_code[16];

// Translate physical shifter byte -> currentGear (file-1 encoding)
void syncGearFromShifter(byte s) {
    if      (s == SHIFTER_PARKING) currentGear = 10;
    else if (s == SHIFTER_REVERSE) currentGear = 11;
    else if (s == SHIFTER_NEUTRAL) currentGear = 12;
    else if (s == SHIFTER_DRIVE)   currentGear = 13;
}

// Shifter handshake on 0x3FD — shifter CRC, own counter, DLC=8
// Called ONLY from processShifterCANFrame(), never from main loop.
void setShifterStatus(byte status_code) {
    uint8_t buf[]   = {shifter_status_counter, status_code, 0x00, 0x00};
    uint8_t crc     = shifter_crc8(0, buf, 4);
    byte payload[5] = {crc, shifter_status_counter, status_code, 0x00, 0x00};
    CAN.sendMsgBuf(0x3FD, 0, 8, payload);
    shifter_status_counter++;
    if ((shifter_status_counter & 0xF) == 0xF) shifter_status_counter++;
}

// Send backlight CAN message for the shifter module
void turnShifterBacklight() {
    byte payload[] = {0xFF, 0x00, 0x00};
    CAN.sendMsgBuf(0x202, 0, 4, payload);
}

void shifterInitTest(long unsigned int id) {
    if ((id & 0x197) == 0x197) {
        setShifterStatus(SHIFTER_NEUTRAL); delay(400);
        setShifterStatus(SHIFTER_PARKING); delay(400);
        setShifterStatus(SHIFTER_NEUTRAL); delay(400);
        setShifterStatus(SHIFTER_PARKING); delay(400);
    }
    shifter_init_done = true;
}

void processShifterCANFrame() {
    sprintf(lever_pos_code,   "0x%.2X", rxBuf[2]);
    sprintf(parking_pos_code, "0x%.2X", rxBuf[3]);

    if (!shifter_init_done) shifterInitTest(rxId);

    // Debug print
    sprintf(msgString, "Shifter RX 0x%.3lX DLC:%d ", rxId, rxLen);
    Serial.print(msgString);
    for (byte i = 0; i < rxLen; i++) { sprintf(msgString, "0x%.2X ", rxBuf[i]); Serial.print(msgString); }
    Serial.println();

    // Always echo current status back to the module
    setShifterStatus(shifter_status_current);

    // ── Parking button ────────────────────────────────────────────────────────
    if (String(parking_pos_code) == POS_PARKING_POS && parking_btn_released) {
        parking_btn_released   = false;
        parking_engaged        = true;
        shifter_status_current = SHIFTER_PARKING;
        shifter_schema_pos     = 1;
        setShifterStatus(SHIFTER_PARKING);
        syncGearFromShifter(shifter_status_current);
    }
    if (String(parking_pos_code) == POS_PARKING_RELEASED) {
        parking_btn_released = true;
    }

    // ── Enter / exit M/S mode ─────────────────────────────────────────────────
    if (String(lever_pos_code) == POS_SIDE && shifter_mode_auto) {
        shifter_mode_auto = false;
        manualGear = 1;
        currentGear = manualGear;
    } else if (String(lever_pos_code) == POS_MIDDLE && !shifter_mode_auto) {
        shifter_mode_auto = true;
        currentGear = 13;  // Back to D
    }

    // ── Auto-mode lever movements ─────────────────────────────────────────────
    if (String(lever_pos_code) == POS_MIDDLE_MOVE_DOWN && shifter_released) {
        shifter_released = false;
        if (parking_engaged) {
            parking_engaged        = false;
            shifter_schema_pos     = 1;
            shifter_status_current = SHIFTER_NEUTRAL;
            setShifterStatus(SHIFTER_NEUTRAL);
        } else if (shifter_schema_pos + 1 <= 2) {
            shifter_schema_pos++;
            shifter_status_current = shifter_schema[shifter_schema_pos];
            setShifterStatus(shifter_status_current);
        }
        syncGearFromShifter(shifter_status_current);
    } else if (String(lever_pos_code) == POS_MIDDLE_MOVE_UP && shifter_released && !parking_engaged) {
        if (shifter_schema_pos - 1 >= 0) {
            shifter_released = false;
            shifter_schema_pos--;
            shifter_status_current = shifter_schema[shifter_schema_pos];
            setShifterStatus(shifter_status_current);
            syncGearFromShifter(shifter_status_current);
        }
    }

    // ── M/S mode lever movements ─────────────────────────────────────────────
    if (String(lever_pos_code) == POS_SIDE_MOVE_DOWN && shifter_released) {
        shifter_released = false;
        if (manualGear < 8) manualGear++;
        currentGear = manualGear;
        Serial.println("SEMI:GEAR_UP:" + String(manualGear));
    } else if (String(lever_pos_code) == POS_SIDE_MOVE_UP && shifter_released) {
        shifter_released = false;
        if (manualGear > 1) manualGear--;
        currentGear = manualGear;
        Serial.println("SEMI:GEAR_DOWN:" + String(manualGear));
    }

    // Release latch when lever returns to rest
    if (String(lever_pos_code) == POS_MIDDLE || String(lever_pos_code) == POS_SIDE) {
        shifter_released = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  CLUSTER CAN MESSAGES
// ─────────────────────────────────────────────────────────────────────────────

// 0x12F — Ignition (from CarCluster: supports ON/OFF)
void sendIgnitionStatus() {
    uint8_t ignStatus = ignitionOn ? 0x8A : 0x08;
    unsigned char d[] = {0x80 | counter4Bit, ignStatus, 0xDD, 0xF1, 0x01, 0x30, 0x06};
    uint8_t crc = crc8Calculator.get_crc8(d, 7, 0x44);
    unsigned char p[] = {crc, d[0],d[1],d[2],d[3],d[4],d[5],d[6]};
    CAN.sendMsgBuf(0x12F, 0, 8, p);
}

// 0x202 — Backlight (from CarCluster: variable brightness)
void sendBacklight() {
    uint8_t mapped = map(backlightBrightness, 0, 100, 0, 253);
    unsigned char p[] = {mapped, 0xFF};
    CAN.sendMsgBuf(0x202, 0, 2, p);
}

// 0x1A1 — Speed
void sendSpeed() {
    uint16_t cs = (double)currentSpeed * 64.01;
    unsigned char d[] = {0xC0 | counter4Bit, (uint8_t)(cs & 0xFF), (uint8_t)(cs >> 8),
                         (currentSpeed == 0 ? 0x81 : 0x91)};
    uint8_t crc = crc8Calculator.get_crc8(d, 4, 0xA9);
    unsigned char p[] = {crc, d[0],d[1],d[2],d[3]};
    CAN.sendMsgBuf(0x1A1, 0, 5, p);
}

// 0x0F3 — RPM
void sendRPM() {
    int cg = 0;
    switch (currentGear) {
        case 1 ... 9: cg = currentGear + 4; break;
        case 11: cg = 2; break;   // R
        case 12: cg = 1; break;   // N
    }
    int rv = map(currentRPM, 0, 6900, 0x00, 0x2B);
    unsigned char d[] = {0x60 | counter4Bit, (uint8_t)rv, 0xC0, 0xF0, (uint8_t)cg, 0xFF, 0xFF};
    uint8_t crc = crc8Calculator.get_crc8(d, 7, 0x7A);
    unsigned char p[] = {crc, d[0],d[1],d[2],d[3],d[4],d[5],d[6]};
    CAN.sendMsgBuf(0x0F3, 0, 8, p);
}

// 0x3FD — Cluster gear display (cluster CRC, counter4Bit, DLC=5)
// Runs every loop cycle. Shifter module ignores these (wrong CRC for it).
void sendTransmission() {
    uint8_t sg = 0x20;
    switch (currentGear) {
        case 1 ... 9: sg = 0x81; break;  // DS (manual gears)
        case 10:      sg = 0x20; break;  // P
        case 11:      sg = 0x40; break;  // R
        case 12:      sg = 0x60; break;  // N
        case 13:      sg = 0x80; break;  // D
    }
    unsigned char d[] = {counter4Bit, sg, 0xFC, 0xFF};
    uint8_t crc = crc8Calculator.get_crc8(d, 4, 0xD6);
    unsigned char p[] = {crc, d[0], d[1], d[2], d[3]};
    CAN.sendMsgBuf(0x3FD, 0, 5, p);
}

// 0x5C0 — Alerts
void sendAlerts() {
    auto alert = [](uint8_t id, bool on) {
        uint8_t m[] = {0x40, id, 0x00, (uint8_t)(on ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
        CAN.sendMsgBuf(0x5C0, 0, 8, m);
    };
    alert(0x0F, doorOpen);
    alert(215,  dscAlert);
    alert(71,   handbrake);
    alert(34,   checkEngine);
    alert(36,   dscOff);
    alert(24,   parkBrake);
    alert(77,   false);   // seatbelt — always clear
    alert(73,   warning73);
    alert(85,   cruiseWarning);
    alert(88,   dippedBeamFailure);
    // dipped beam secondary IDs
    alert(98,   dippedBeamFailure);
    alert(99,   dippedBeamFailure);
    alert(100,  dippedBeamFailure);
    // fasten seatbelts
    alert(86,   fastenSeatbelts);
    alert(87,   fastenSeatbelts);
    alert(91,   fastenSeatbelts);
    // brake failure
    alert(72,   brakeFailure);
    alert(74,   brakeFailure);
    // trailer reversing
    alert(76,   trailerReversing);
    alert(89,   trailerReversing);
    alert(90,   trailerReversing);
    // restraint rear
    alert(92,   restraintRear);
    alert(93,   restraintRear);
    alert(94,   restraintRear);
    // chassis
    { uint8_t m[] = {0x40, (uint8_t)(299 & 0xFF), 0x00, (uint8_t)(chassisWarning ? 0x29 : 0x28), 0xFF, 0xF7, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, m); }
    // SOS
    { uint8_t m[] = {0x40, 196, 0x00, (uint8_t)(sosCall ? 0x29 : 0x28), 0xFF, 0xF7, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, m); }
    alert(95, sosCall);
    alert(96, sosCall);
}

// 0x21A — Lights (from CarCluster: high beam forces main lights ON)
void sendLights() {
    if (highBeam) mainLights = true;
    uint8_t ls = (highBeam << 1) | (mainLights << 2) | (frontFogLight << 5) | (rearFogLight << 6);
    unsigned char p[] = {ls, 0xC0, 0xF7};
    CAN.sendMsgBuf(0x21A, 0, 3, p);
}

// 0x1F6 — Blinkers
void sendBlinkers() {
    uint8_t bs = (!leftBlinker && !rightBlinker)
                 ? 0x80 : (0x81 | (leftBlinker << 4) | (rightBlinker << 5));
    unsigned char p[] = {bs, 0xF0};
    CAN.sendMsgBuf(0x1F6, 0, 2, p);
}

// 0x3A7 — Drive mode
void sendDriveMode() {
    unsigned char d[] = {0xF0 | counter4Bit, 0, 0, driveMode, 0x11, 0xC0};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 6, 0x4A), d[0],d[1],d[2],d[3],d[4],d[5]};
    CAN.sendMsgBuf(0x3A7, 0, 7, p);
}

// 0x349 — Fuel
void sendFuel() {
    uint16_t fq = multiMap<uint8_t>(fuelLevel, inFuelRange, outFuelRange, 3);
    uint8_t hi  = (fq >> 8) & 0xFF, lo = fq & 0xFF;
    unsigned char p[] = {hi, lo, hi, lo, 0x00};
    CAN.sendMsgBuf(0x349, 0, 5, p);
}

// 0x36E — ABS
void sendABS() {
    unsigned char d[] = {0xF0 | counter4Bit, 0xFE, 0xFF, 0x14};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 4, 0xD8), d[0],d[1],d[2],d[3]};
    CAN.sendMsgBuf(0x36E, 0, 5, p);
}

// 0xD7 — Safety counter
void sendSafetyCounter() {
    unsigned char p[] = {count, 0xFF};
    CAN.sendMsgBuf(0xD7, 0, 2, p);
}

// 0x2A7 — Power steering
void sendPowerSteering() {
    unsigned char d[] = {0xF0 | counter4Bit, 0xFE, 0xFF, 0x14};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 4, 0x9E), d[0],d[1],d[2],d[3]};
    CAN.sendMsgBuf(0x2A7, 0, 5, p);
}

// 0x289 — Cruise control (from CarCluster)
void sendCruiseControl() {
    uint8_t cruiseState = cruiseControlActive ? 0xE3 : 0xE0;
    unsigned char d[] = {0xF0 | counter4Bit, 0xE0, 0xE0, cruiseState, 0x00, 0xEC, 0x01};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 7, 0x82), d[0],d[1],d[2],d[3],d[4],d[5],d[6]};
    CAN.sendMsgBuf(0x289, 0, 8, p);
}

// 0x19B — Airbag
void sendAirbag() {
    unsigned char d[] = {0x40 | counter4Bit, 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 7, 0xFF), d[0],d[1],d[2],d[3],d[4],d[5],d[6]};
    CAN.sendMsgBuf(0x19B, 0, 8, p);
}

// 0x297 — Seatbelt system
void sendSeatbeltSystem() {
    unsigned char d[] = {0xE0 | counter4Bit, 0xF1, 0xF0, 0xF2, 0xF2, 0xFE};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 6, 0x28), d[0],d[1],d[2],d[3],d[4],d[5]};
    CAN.sendMsgBuf(0x297, 0, 7, p);
}

// 0x2C4 — Engine temperature
void sendEngineTemperature() {
    unsigned char d[] = {0x3E, (uint8_t)engineTemperature, 0x64, 0x64, 0x64, 0x01, 0xF1};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 7, 0xB2), d[0],d[1],d[2],d[3],d[4],d[5],d[6]};
    CAN.sendMsgBuf(0x2C4, 0, 8, p);
}

// 0x3F9 — Oil temperature (drives gauge needle)
void sendOilTemperature() {
    unsigned char d[] = {0x10 | counter4Bit, 0x82, 0x4E, 0x7E,
                         (uint8_t)(engineTemperature + 50), 0x05, 0x89};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 7, 0xF1), d[0],d[1],d[2],d[3],d[4],d[5],d[6]};
    CAN.sendMsgBuf(0x3F9, 0, 8, p);
}

// 0x369 — TPMS
void sendTPMS() {
    unsigned char d[] = {0xF0 | counter4Bit, 0xA2, 0xA0, 0xA0};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 4, 0xC5), d[0],d[1],d[2],d[3]};
    CAN.sendMsgBuf(0x369, 0, 5, p);
}

// 0x36F — Park brake status
void sendParkBrakeStatus() {
    unsigned char d[] = {0xF0 | counter4Bit, 0x38, 0x00, (uint8_t)(handbrake ? 0x15 : 0x14)};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 4, 0x17), d[0],d[1],d[2],d[3]};
    CAN.sendMsgBuf(0x36F, 0, 5, p);
}

// 0x2BB / 0x2C4 — Distance & MPG
void sendDistanceTravelled() {
    unsigned char d1[] = {count, 0xFF, 0x64, 0x64, 0x64, 0x01, 0xF1};
    unsigned char p1[] = {crc8Calculator.get_crc8(d1, 7, 0xC6), d1[0],d1[1],d1[2],d1[3],d1[4],d1[5],d1[6]};
    CAN.sendMsgBuf(0x2C4, 0, 8, p1);

    unsigned char d2[] = {0xF0 | counter4Bit,
                          (uint8_t)(distanceTravelledCounter & 0xFF),
                          (uint8_t)(distanceTravelledCounter >> 8), 0xF2};
    unsigned char p2[] = {crc8Calculator.get_crc8(d2, 4, 0xDE), d2[0],d2[1],d2[2],d2[3]};
    CAN.sendMsgBuf(0x2BB, 0, 5, p2);
    distanceTravelledCounter += currentSpeed * 2.9;
}

// Required housekeeping messages
void send2CAMessage() { unsigned char p[] = {0x64, 0x65}; CAN.sendMsgBuf(0x2CA, 0, 2, p); }
void send393Message()  { unsigned char p[] = {0x3E, 0x32, 0x05, 0xFE}; CAN.sendMsgBuf(0x393, 0, 4, p); }

void send1B3Message() {
    unsigned char d[] = {0x1E, 0x01, 0xF0 | counter4Bit, 0x00, 0xFF, 0x80, 0x50};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 7, 0xFF), d[0],d[1],d[2],d[3],d[4],d[5],d[6]};
    CAN.sendMsgBuf(0x1B3, 0, 8, p);
}

void send2C5Message() {
    unsigned char d[] = {0xF0 | counter4Bit, 0x00, 0xF0, 0x04, 0x70, 0x08};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 6, 0xD8), d[0],d[1],d[2],d[3],d[4],d[5]};
    CAN.sendMsgBuf(0x2C5, 0, 7, p);
}

void sendACC() {
    unsigned char d[] = {0xF0 | accCounter, 0x5C, 0x70, 0x00, 0x00};
    unsigned char p[] = {crc8Calculator.get_crc8(d, 5, 0x6B), d[0],d[1],d[2],d[3],d[4]};
    CAN.sendMsgBuf(0x33B, 0, 6, p);
    accCounter += 4;
    if (accCounter > 0x0E) accCounter -= 0x0F;
}

// ─────────────────────────────────────────────────────────────────────────────
//  CarCluster features — smart alerts, time, steering button
// ─────────────────────────────────────────────────────────────────────────────

// Helper: send a single CC-ID alert on/off
void sendCC(uint8_t id, bool on) {
    uint8_t m[] = {0x40, id, 0x00, (uint8_t)(on ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, m);
}

// Automatic alerts from CarCluster — engine start, overspeed, overheat, etc.
void sendSmartAlerts() {
    static unsigned long engineStartTime = 0;
    static bool engineWasRunning = false;
    static bool startupDone = false;

    // Engine start sequence — after 10s of ignition, pulse CC-IDs 91/53, set 181
    if (ignitionOn) {
        if (!engineWasRunning) {
            engineStartTime = millis();
            engineWasRunning = true;
            startupDone = false;
        }
        if (!startupDone && millis() - engineStartTime >= 10000) {
            sendCC(91, true); delay(50); sendCC(91, false);
            sendCC(53, true); delay(50); sendCC(53, false);
            sendCC(181, true);
            startupDone = true;
        }
    } else {
        engineWasRunning = false;
        startupDone = false;
    }

    // Engine not started warnings (ignition ON, RPM < 10)
    bool notStarted = ignitionOn && currentRPM < 10;
    sendCC(40, notStarted);
    sendCC(41, notStarted);

    // Ignition ON but RPM=0 — show pre-start warnings
    bool noRPM = ignitionOn && currentRPM == 0;
    sendCC(213, noRPM);
    sendCC(220, noRPM);
    sendCC(21, noRPM);
    sendCC(30, noRPM);
    sendCC(175, noRPM);
    sendCC(206, noRPM);
    sendCC(255, noRPM);

    // Failed start (RPM 200-600)
    bool failedStart = ignitionOn && currentRPM >= 200 && currentRPM < 600;
    sendCC(186, failedStart);
    sendCC(22, failedStart);

    // N gear CC-IDs
    sendCC(169, currentGear == 12);
    sendCC(203, currentGear == 12);

    // Handbrake + speed alert
    sendCC(48, handbrake && currentSpeed > 1);
    sendCC(55, handbrake);

    // Overspeed warning (adjustable threshold, 0 = disabled)
    sendCC(62, overspeedThreshold > 0 && currentSpeed > overspeedThreshold);

    // Oil/coolant overheat
    sendCC(39, engineTemperature > 130);

    // Gearbox overheat (from CarCluster)
    if (engineTemperature > 120 && currentRPM > 3500) sendCC(103, true);
    if (engineTemperature > 130 && currentSpeed > 80)  sendCC(104, true);
    if (engineTemperature > 140)                       sendCC(105, true);
}

// 0x2F8 — Time & date broadcast
// Format: {hours, minutes, seconds, day, (month<<4)|0x0F, year_lo, year_hi, 0xF5}
// Normally the KOMBI sends this out — on bench setups injecting it can set the display
void sendTimeFrame() {
    unsigned char p[] = {
        clockHours,
        clockMinutes,
        clockSeconds,
        dateDay,
        (uint8_t)((dateMonth << 4) | 0x0F),
        (uint8_t)(dateYear & 0xFF),
        (uint8_t)((dateYear >> 8) & 0xFF),
        0xF5
    };
    CAN.sendMsgBuf(0x2F8, 0, 8, p);
}

// 0x327 — Lane assistant display (KAFAS)
// DLC=4, CRC8, upper nibble 0x5_ = active, bytes 2-3 = lane marking data
// Cluster needs coding: TLC_VERBAUT=aktiviert, ST_TLC_ALIVE/TIMEOUT/APPL=aktiv
void sendLaneAssist() {
    if (!laneAssistActive) return;
    unsigned char d[] = {
        (uint8_t)(0x50 | counter4Bit),
        laneLeftStatus,
        laneRightStatus
    };
    unsigned char p[] = {crc8Calculator.get_crc8(d, 3, 0x27), d[0], d[1], d[2]};
    CAN.sendMsgBuf(0x327, 0, 4, p);
}

// 0x1EE — Steering wheel menu button (from CarCluster)
void sendSteeringWheelButton(bool press) {
    uint8_t p[] = {(uint8_t)(press ? 76 : 0x00), 0xFF};
    CAN.sendMsgBuf(0x1EE, 0, 2, p);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Serial command handler
// ─────────────────────────────────────────────────────────────────────────────
void processSerialCommand() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // No-argument commands
    if (cmd == "MENUBUTTON") { sendSteeringWheelButton(true); delay(100); sendSteeringWheelButton(false); Serial.println("OK:MENUBUTTON"); return; }
    // UDSBRUTE — brute-force: try ALL sessions, security levels, and direct writes
    if (cmd == "UDSBRUTE") {
        Serial.println("UDSBRUTE: Full brute-force attack on KOMBI coding...");
        Serial.println("UDSBRUTE: WARNING — may brick cluster!");
        uint8_t fc[8] = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};

        // Current 3003 data (known good): 47 61 10 69 84 FF FF FC 1E 00 00 00 00 00 00 00
        uint8_t origData[16] = {0x47, 0x61, 0x10, 0x69, 0x84, 0xFF, 0xFF, 0xFC,
                                0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        // Keep-alive helper — sends full loop cycle to prevent cluster sleep
        unsigned long lastKA = 0;
        auto keepAlive = [&]() {
            if (millis() - lastKA >= 90) {
                send2C5Message();
                send1B3Message();
                send2CAMessage();
                send393Message();
                sendACC();
                sendIgnitionStatus();
                sendBacklight();
                sendSpeed();
                sendRPM();
                sendTransmission();
                sendAlerts();
                sendLights();
                sendBlinkers();
                sendABS();
                sendSafetyCounter();
                sendPowerSteering();
                sendCruiseControl();
                sendAirbag();
                sendSeatbeltSystem();
                sendTPMS();
                sendEngineTemperature();
                sendOilTemperature();
                sendParkBrakeStatus();
                sendDriveMode();
                sendFuel();
                sendDistanceTravelled();
                sendLaneAssist();
                counter4Bit++;
                if (counter4Bit >= 14) counter4Bit = 0;
                count++;
                if (count >= 254) count = 0;
                lastKA = millis();
            }
        };

        // Wait for UDS response with keep-alive (replaces bare delay loops)
        auto waitResp = [&](unsigned long ms) -> bool {
            unsigned long t = millis() + ms;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) return true;
                yield();
            }
            return false;
        };

        // Drain one CAN frame from 0x686 (helper)
        auto drainResp = [&](unsigned long ms, uint8_t* outBuf, uint8_t* outLen) -> bool {
            unsigned long t = millis() + ms;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        if (outBuf) memcpy(outBuf, rBuf, rLen);
                        if (outLen) *outLen = rLen;
                        return true;
                    }
                }
                yield();
            }
            return false;
        };

        // Helper: send ISO-TP multi-frame write for section 3003
        // Returns: 0=no response, 1=accepted (0x6E), 2=rejected with NRC
        auto tryWrite = [&](uint8_t* data, int dataLen) -> int {
            int writeLen = 3 + dataLen; // 2E 30 03 + data
            uint8_t ffBuf[8];
            ffBuf[0] = 0x86;
            ffBuf[1] = 0x10 | ((writeLen >> 8) & 0x0F);
            ffBuf[2] = writeLen & 0xFF;
            ffBuf[3] = 0x2E; ffBuf[4] = 0x30; ffBuf[5] = 0x03;
            int payIdx = 0;
            for (int i = 6; i < 8 && payIdx < dataLen; i++)
                ffBuf[i] = data[payIdx++];
            CAN.sendMsgBuf(0x6F0, 0, 8, ffBuf);

            // Wait for FC or negative response
            unsigned long t = millis() + 1500;
            bool gotFC = false;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        if ((rBuf[ds] & 0xF0) == 0x30) { gotFC = true; break; }
                        if ((rBuf[ds] & 0xF0) == 0x00 && rBuf[ds+1] == 0x7F) {
                            Serial.printf("NRC:0x%02X ", rBuf[ds+3]);
                            return 2;
                        }
                    }
                }
                yield();
            }
            if (!gotFC) return 0;

            // Send CFs
            uint8_t seq = 1;
            while (payIdx < dataLen) {
                uint8_t cf[8]; cf[0] = 0x86; cf[1] = 0x20 | (seq & 0x0F);
                for (int i = 2; i < 8; i++) {
                    cf[i] = (payIdx < dataLen) ? data[payIdx++] : 0xAA;
                }
                keepAlive();
                unsigned long d = millis() + 5;
                while (millis() < d) yield();
                CAN.sendMsgBuf(0x6F0, 0, 8, cf);
                seq++;
            }

            // Wait for response
            t = millis() + 2000;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        if (rBuf[ds+1] == 0x6E) return 1; // ACCEPTED!
                        if (rBuf[ds+1] == 0x7F) {
                            Serial.printf("NRC:0x%02X ", rBuf[ds+3]);
                            return 2;
                        }
                    }
                }
                yield();
            }
            return 0;
        };

        // Helper: try security access at given level
        auto trySecurity = [&](uint8_t level) -> bool {
            uint8_t req[8] = {0x86, 0x02, 0x27, level, 0xAA, 0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F0, 0, 8, req);
            unsigned long t = millis() + 500;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        uint8_t pci = rBuf[ds];
                        int sfLen = pci & 0x0F;
                        if (sfLen >= 2 && rBuf[ds+1] == 0x67) {
                            Serial.printf("SEED(lvl 0x%02X):", level);
                            for (int i = ds+1; i < ds+1+sfLen; i++) Serial.printf(" %02X", rBuf[i]);
                            Serial.println();
                            return true;
                        }
                    }
                }
                yield();
            }
            return false;
        };

        // ═══════════════════════════════════════════════════════
        // PHASE 1: Try ALL session types (0x01-0x7F)
        // ═══════════════════════════════════════════════════════
        Serial.println("\n=== PHASE 1: Session scan ===");
        uint8_t workingSessions[16];
        int numWorking = 0;

        for (uint8_t s = 0x01; s <= 0x7F && numWorking < 16; s++) {
            keepAlive();
            uint8_t req[8] = {0x86, 0x02, 0x10, s, 0xAA, 0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F0, 0, 8, req);
            unsigned long t = millis() + 300;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        if (rBuf[ds+1] == 0x50) {
                            Serial.printf("SESSION 0x%02X ACCEPTED! resp: %02X %02X\n", s, rBuf[ds+1], rBuf[ds+2]);
                            workingSessions[numWorking++] = s;
                        }
                        break;
                    }
                }
                yield();
            }
            if (s % 16 == 0) Serial.printf("  scanned to 0x%02X...\n", s);
        }
        Serial.printf("Found %d working sessions\n", numWorking);

        // ═══════════════════════════════════════════════════════
        // PHASE 2: For each working session, try security access
        // ═══════════════════════════════════════════════════════
        Serial.println("\n=== PHASE 2: Security access per session ===");
        uint8_t secLevels[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x09,0x0B,0x0D,
                               0x11,0x13,0x19,0x21,0x27,0x29,0x41,0x43,0x61,0x63,0x65,0x67,0x69,0x6B};
        bool gotSeed = false;

        for (int si = 0; si < numWorking && !gotSeed; si++) {
            keepAlive();
            uint8_t sessReq[8] = {0x86, 0x02, 0x10, workingSessions[si], 0xAA, 0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F0, 0, 8, sessReq);
            uint8_t dummy[8]; uint8_t dLen;
            drainResp(200, dummy, &dLen);

            Serial.printf("In session 0x%02X, trying %d security levels...\n",
                           workingSessions[si], (int)(sizeof(secLevels)));
            for (int l = 0; l < (int)sizeof(secLevels) && !gotSeed; l++) {
                keepAlive();
                if (trySecurity(secLevels[l])) {
                    gotSeed = true;
                    Serial.printf("*** GOT SEED in session 0x%02X level 0x%02X! ***\n",
                                   workingSessions[si], secLevels[l]);
                }
            }
        }

        // ═══════════════════════════════════════════════════════
        // PHASE 3: Try direct write in each working session
        // ═══════════════════════════════════════════════════════
        Serial.println("\n=== PHASE 3: Direct write attempts ===");
        bool writeOK = false;

        for (int si = 0; si < numWorking && !writeOK; si++) {
            keepAlive();
            uint8_t sessReq[8] = {0x86, 0x02, 0x10, workingSessions[si], 0xAA, 0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F0, 0, 8, sessReq);
            uint8_t dummy[8]; uint8_t dLen;
            drainResp(200, dummy, &dLen);

            Serial.printf("Session 0x%02X: write test... ", workingSessions[si]);
            int res = tryWrite(origData, 16);
            if (res == 1) {
                writeOK = true;
                Serial.printf("*** WRITE ACCEPTED in session 0x%02X! ***\n", workingSessions[si]);
            } else if (res == 2) {
                Serial.println("rejected");
            } else {
                Serial.println("no response");
            }
        }

        // ═══════════════════════════════════════════════════════
        // PHASE 4: Try writing via different services
        // ═══════════════════════════════════════════════════════
        Serial.println("\n=== PHASE 4: Alternative write services ===");
        keepAlive();
        uint8_t extS[8] = {0x86, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, extS);
        { uint8_t dummy[8]; uint8_t dLen; drainResp(200, dummy, &dLen); }

        // Try RoutineControl (0x31)
        uint8_t routines[][8] = {
            {0x86, 0x04, 0x31, 0x01, 0x02, 0x09, 0xAA, 0xAA},
            {0x86, 0x04, 0x31, 0x01, 0xFF, 0x00, 0xAA, 0xAA},
            {0x86, 0x04, 0x31, 0x01, 0x02, 0x03, 0xAA, 0xAA},
            {0x86, 0x04, 0x31, 0x01, 0x02, 0x05, 0xAA, 0xAA},
        };
        for (int r = 0; r < 4; r++) {
            keepAlive();
            CAN.sendMsgBuf(0x6F0, 0, 8, routines[r]);
            Serial.printf("Routine 0x%02X%02X: ", routines[r][4], routines[r][5]);
            unsigned long t = millis() + 500;
            bool got = false;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        got = true;
                        for (int i = ds; i < rLen; i++) Serial.printf("%02X ", rBuf[i]);
                        Serial.println();
                        break;
                    }
                }
                yield();
            }
            if (!got) Serial.println("no response");
        }

        // Try IOControlByIdentifier (0x2F) on DID 3003
        keepAlive();
        Serial.print("IOControl 0x2F on 3003: ");
        uint8_t ioCtrl[8] = {0x86, 0x04, 0x2F, 0x30, 0x03, 0x03, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, ioCtrl);
        {
            unsigned long t = millis() + 500;
            bool got = false;
            while (millis() < t) {
                keepAlive();
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        got = true;
                        for (int i = ds; i < rLen; i++) Serial.printf("%02X ", rBuf[i]);
                        Serial.println();
                        break;
                    }
                }
                yield();
            }
            if (!got) Serial.println("no response");
        }

        Serial.println("\n=== UDSBRUTE: Complete ===");
        if (writeOK) Serial.println("*** WRITE ACCESS FOUND! Use UDSTESTWRITE to write coding ***");
        else if (gotSeed) Serial.println("*** SEED FOUND! Key algorithm needed ***");
        else Serial.println("No write access found. Cluster requires gateway for coding.");
        Serial.println("UDSBRUTE: Done");
        return;
    }
    // UDSBOOTPROG — power-cycle cluster then try programming session + security during boot
    if (cmd == "UDSBOOTPROG") {
        Serial.println("UDSBOOTPROG: Power-cycling cluster, then trying prog session...");
        Serial.println("UDSBOOTPROG: Stopping CAN for 2s...");
        delay(2000);
        yield();

        uint8_t fc[8] = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};
        uint8_t defSess[8]  = {0x86, 0x02, 0x10, 0x01, 0xAA, 0xAA, 0xAA, 0xAA};
        uint8_t extSess[8]  = {0x86, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA};
        uint8_t progSess[8] = {0x86, 0x02, 0x10, 0x02, 0xAA, 0xAA, 0xAA, 0xAA};
        uint8_t secLevels[] = {0x01, 0x03, 0x05, 0x11, 0x61};

        // Wake-up burst first (like setup does)
        Serial.println("UDSBOOTPROG: Wake-up burst...");
        for (int i = 0; i < 15; i++) {
            sendIgnitionStatus();
            sendBacklight();
            sendSafetyCounter();
            sendLights();
            delay(20);
            yield();
        }

        Serial.println("UDSBOOTPROG: Trying programming session...");
        unsigned long endTime = millis() + 15000;
        int phase = 1;  // skip phase 0, go straight to session attempts
        int attempt = 0;
        bool progOK = false;
        bool seedGot = false;

        while (millis() < endTime) {
            // Keep cluster alive (ignition needed or it shuts down)
            sendIgnitionStatus();
            sendBacklight();
            sendSafetyCounter();
            counter4Bit++;
            if (counter4Bit >= 14) counter4Bit = 0;

            if (phase == 0 && attempt >= 2) {
                // After 200ms of wake-up, start trying sessions immediately
                phase = 1;
                attempt = 0;
            }

            if (phase == 1) {
                // Try different session entry sequences
                uint8_t* sessReq;
                if (attempt % 3 == 0) sessReq = progSess;      // direct prog
                else if (attempt % 3 == 1) sessReq = extSess;   // ext first
                else sessReq = defSess;                          // default first

                CAN.sendMsgBuf(0x6F0, 0, 8, sessReq);
                Serial.printf("UDSBOOTPROG: Try session 0x%02X (attempt %d)... ", sessReq[3], attempt);

                unsigned long t = millis() + 200;
                while (millis() < t) {
                    if (!digitalRead(CAN_INT)) {
                        unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                        CAN.readMsgBuf(&rId, &rLen, rBuf);
                        if (rId == 0x686) {
                            int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                            Serial.printf("resp:");
                            for (int i = ds; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                            Serial.println();
                            if (rBuf[ds + 1] == 0x50 && rBuf[ds + 1 + 1] == 0x02) {
                                // Programming session accepted!
                                Serial.println("UDSBOOTPROG: *** PROGRAMMING SESSION ACCEPTED! ***");
                                progOK = true;
                                phase = 2;
                                attempt = 0;
                            } else if ((rBuf[ds] & 0xF0) == 0x00 && rBuf[ds+1] == 0x50 && rBuf[ds+2] == 0x02) {
                                Serial.println("UDSBOOTPROG: *** PROGRAMMING SESSION ACCEPTED! ***");
                                progOK = true;
                                phase = 2;
                                attempt = 0;
                            }
                            break;
                        }
                    }
                    yield();
                }
                if (phase == 1) {
                    // If ext session worked, try prog right after
                    if (attempt % 3 == 1) {
                        delay(50);
                        CAN.sendMsgBuf(0x6F0, 0, 8, progSess);
                        delay(150);
                        if (!digitalRead(CAN_INT)) {
                            unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                            CAN.readMsgBuf(&rId, &rLen, rBuf);
                            if (rId == 0x686) {
                                int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                                Serial.printf("  prog after ext:");
                                for (int i = ds; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                                Serial.println();
                                if (rBuf[ds+1] == 0x50 || (rBuf[ds] < 0x10 && rBuf[ds+1] == 0x50)) {
                                    Serial.println("UDSBOOTPROG: *** PROG SESSION VIA EXT! ***");
                                    progOK = true;
                                    phase = 2;
                                    attempt = 0;
                                }
                            }
                        }
                    }
                    Serial.println();
                }
                attempt++;
                if (attempt > 30) {
                    Serial.println("UDSBOOTPROG: Giving up on prog session");
                    break;
                }
            }

            if (phase == 2 && progOK) {
                // Try security access
                int lvl = attempt % 5;
                uint8_t seedReq[8] = {0x86, 0x02, 0x27, secLevels[lvl], 0xAA, 0xAA, 0xAA, 0xAA};
                CAN.sendMsgBuf(0x6F0, 0, 8, seedReq);
                Serial.printf("UDSBOOTPROG: Security level 0x%02X... ", secLevels[lvl]);

                udsFullLen = 0;
                int expLen = 0;
                unsigned long st = millis() + 500;
                bool got = false;
                while (millis() < st) {
                    if (!digitalRead(CAN_INT)) {
                        unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                        CAN.readMsgBuf(&rId, &rLen, rBuf);
                        if (rId == 0x686) {
                            got = true;
                            int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                            uint8_t pci = rBuf[ds];
                            if ((pci & 0xF0) == 0x10) {
                                udsFullLen = 0;
                                expLen = ((pci & 0x0F) << 8) | rBuf[ds+1];
                                for (int i = ds+2; i < rLen && udsFullLen < expLen; i++)
                                    udsFullResp[udsFullLen++] = rBuf[i];
                                CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                            } else if ((pci & 0xF0) == 0x00) {
                                udsFullLen = 0;
                                expLen = pci & 0x0F;
                                for (int i = ds+1; i < ds+1+expLen && i < rLen; i++)
                                    udsFullResp[udsFullLen++] = rBuf[i];
                            }
                            if (expLen > 0 && udsFullLen >= expLen) break;
                        }
                    }
                    yield();
                }
                if (!got) {
                    Serial.println("no response");
                } else if (udsFullLen >= 2 && udsFullResp[0] == 0x67) {
                    Serial.printf("*** SEED: ");
                    for (int i = 2; i < udsFullLen; i++) Serial.printf("%02X ", udsFullResp[i]);
                    Serial.println("***");
                    seedGot = true;
                    break;
                } else if (udsFullLen >= 3 && udsFullResp[0] == 0x7F) {
                    uint8_t nrc = udsFullResp[2];
                    Serial.printf("NRC 0x%02X", nrc);
                    switch (nrc) {
                        case 0x12: Serial.print(" (subFunctionNotSupported)"); break;
                        case 0x22: Serial.print(" (conditionsNotCorrect)"); break;
                        case 0x35: Serial.print(" (invalidKey)"); break;
                        case 0x36: Serial.print(" (exceededAttempts)"); break;
                        case 0x37: Serial.print(" (timeDelayNotExpired)"); break;
                        case 0x7F: Serial.print(" (notSupportedInSession)"); break;
                    }
                    Serial.println();
                } else {
                    for (int i = 0; i < udsFullLen; i++) Serial.printf("%02X ", udsFullResp[i]);
                    Serial.println();
                }
                attempt++;
                if (attempt > 10) break;
            }

            delay(100);
            yield();
        }

        if (!progOK) Serial.println("UDSBOOTPROG: Could not enter programming session");
        if (progOK && !seedGot) Serial.println("UDSBOOTPROG: Prog session OK but no security seed");
        Serial.println("UDSBOOTPROG: Done");
        return;
    }
    // UDSTESTWRITE — safe test: write identical 3003 data back to check if write works
    if (cmd == "UDSTESTWRITE") {
        Serial.println("UDSTESTWRITE: Testing if coding write is possible...");
        uint8_t fc[8] = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};

        // Step 1: Extended session
        uint8_t sess[8] = {0x86, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, sess);
        delay(150);
        if (!digitalRead(CAN_INT)) {
            unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
            CAN.readMsgBuf(&rId, &rLen, rBuf);
            Serial.printf("UDSTESTWRITE: Session:");
            for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
            Serial.println();
        }

        // Step 2: Read current 3003 — cap at expectedLen to exclude padding
        uint8_t rd[8] = {0x86, 0x03, 0x22, 0x30, 0x03, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, rd);
        udsFullLen = 0;
        int expectedLen = 0;
        unsigned long timeout = millis() + 2000;
        while (millis() < timeout) {
            if (!digitalRead(CAN_INT)) {
                unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                CAN.readMsgBuf(&rId, &rLen, rBuf);
                if (rId == 0x686) {
                    int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                    uint8_t pci = rBuf[ds];
                    if ((pci & 0xF0) == 0x10) {
                        udsFullLen = 0;
                        expectedLen = ((pci & 0x0F) << 8) | rBuf[ds + 1];
                        for (int i = ds + 2; i < rLen && udsFullLen < expectedLen; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                    } else if ((pci & 0xF0) == 0x20) {
                        for (int i = ds + 1; i < rLen && udsFullLen < expectedLen; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                    } else if ((pci & 0xF0) == 0x00) {
                        udsFullLen = 0;
                        expectedLen = pci & 0x0F;
                        for (int i = ds + 1; i < ds + 1 + expectedLen && i < rLen; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                    }
                    if (expectedLen > 0 && udsFullLen >= expectedLen) break;
                }
            }
            yield();
        }
        if (udsFullLen < 4 || udsFullResp[0] != 0x62) {
            Serial.println("UDSTESTWRITE: Failed to read 3003");
            Serial.println("UDSTESTWRITE: Done");
            return;
        }
        // Data bytes after 62 30 03 header — use expectedLen not udsFullLen
        int dataLen = expectedLen - 3;
        Serial.printf("UDSTESTWRITE: Read %d data bytes (expectedLen=%d):", dataLen, expectedLen);
        for (int i = 3; i < expectedLen; i++) Serial.printf(" %02X", udsFullResp[i]);
        Serial.println();

        // Step 3: Try writing using Single Frame first if small enough
        // Also try multi-frame for larger payloads
        int writeLen = 3 + dataLen;  // 2E 30 03 + data
        Serial.printf("UDSTESTWRITE: Write payload = %d bytes\n", writeLen);

        // If writeLen <= 6, we can use Single Frame (with ext addr, max 6 payload bytes)
        if (writeLen <= 6) {
            uint8_t wr[8] = {0x86, (uint8_t)writeLen, 0x2E, 0x30, 0x03, 0xAA, 0xAA, 0xAA};
            for (int i = 0; i < dataLen && (5 + i) < 8; i++)
                wr[5 + i] = udsFullResp[3 + i];
            CAN.sendMsgBuf(0x6F0, 0, 8, wr);
            Serial.printf("UDSTESTWRITE: SF:");
            for (int i = 0; i < 8; i++) Serial.printf(" %02X", wr[i]);
            Serial.println();
        } else {
            // Multi-frame: First Frame
            uint8_t ffBuf[8];
            ffBuf[0] = 0x86;
            ffBuf[1] = 0x10 | ((writeLen >> 8) & 0x0F);
            ffBuf[2] = writeLen & 0xFF;
            ffBuf[3] = 0x2E;
            ffBuf[4] = 0x30;
            ffBuf[5] = 0x03;
            int payIdx = 0;
            for (int i = 6; i < 8 && payIdx < dataLen; i++)
                ffBuf[i] = udsFullResp[3 + payIdx++];
            CAN.sendMsgBuf(0x6F0, 0, 8, ffBuf);
            Serial.printf("UDSTESTWRITE: FF:");
            for (int i = 0; i < 8; i++) Serial.printf(" %02X", ffBuf[i]);
            Serial.println();

            // Wait for Flow Control
            bool gotFC = false;
            unsigned long fcTimeout = millis() + 2000;
            while (millis() < fcTimeout) {
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        Serial.printf("UDSTESTWRITE: RX:");
                        for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                        Serial.println();
                        if ((rBuf[ds] & 0xF0) == 0x30) {
                            gotFC = true;
                            break;
                        }
                        // Check for negative response SF
                        if ((rBuf[ds] & 0xF0) == 0x00) {
                            int sfLen = rBuf[ds] & 0x0F;
                            if (sfLen >= 3 && rBuf[ds+1] == 0x7F) {
                                uint8_t nrc = rBuf[ds+3];
                                Serial.printf("UDSTESTWRITE: REJECTED NRC 0x%02X", nrc);
                                switch (nrc) {
                                    case 0x11: Serial.print(" (serviceNotSupported)"); break;
                                    case 0x22: Serial.print(" (conditionsNotCorrect)"); break;
                                    case 0x31: Serial.print(" (requestOutOfRange)"); break;
                                    case 0x33: Serial.print(" (securityAccessDenied)"); break;
                                    case 0x72: Serial.print(" (generalProgrammingFailure)"); break;
                                    case 0x7F: Serial.print(" (serviceNotSupportedInActiveSession)"); break;
                                }
                                Serial.println();
                                Serial.println("UDSTESTWRITE: Done");
                                return;
                            }
                        }
                    }
                }
                yield();
            }
            if (!gotFC) {
                Serial.println("UDSTESTWRITE: No FC — cluster rejected or ignored the write");
                Serial.println("UDSTESTWRITE: Done");
                return;
            }

            // Send Consecutive Frames
            uint8_t seqNum = 1;
            while (payIdx < dataLen) {
                uint8_t cfBuf[8];
                cfBuf[0] = 0x86;
                cfBuf[1] = 0x20 | (seqNum & 0x0F);
                for (int i = 2; i < 8; i++) {
                    if (payIdx < dataLen) cfBuf[i] = udsFullResp[3 + payIdx++];
                    else cfBuf[i] = 0xAA;
                }
                delay(10);
                CAN.sendMsgBuf(0x6F0, 0, 8, cfBuf);
                Serial.printf("UDSTESTWRITE: CF%d:", seqNum);
                for (int i = 0; i < 8; i++) Serial.printf(" %02X", cfBuf[i]);
                Serial.println();
                seqNum++;
            }
        }

        // Wait for write response
        unsigned long wrTimeout = millis() + 3000;
        bool gotResp = false;
        while (millis() < wrTimeout) {
            if (!digitalRead(CAN_INT)) {
                unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                CAN.readMsgBuf(&rId, &rLen, rBuf);
                if (rId == 0x686) {
                    gotResp = true;
                    int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                    Serial.printf("UDSTESTWRITE: Response:");
                    for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                    Serial.println();
                    uint8_t resp = rBuf[ds + 1];
                    if (resp == 0x6E) {
                        Serial.println("UDSTESTWRITE: *** WRITE ACCEPTED! ***");
                    } else if (resp == 0x7F) {
                        uint8_t nrc = rBuf[ds + 3];
                        Serial.printf("UDSTESTWRITE: REJECTED NRC 0x%02X", nrc);
                        switch (nrc) {
                            case 0x11: Serial.print(" (serviceNotSupported)"); break;
                            case 0x13: Serial.print(" (incorrectMessageLength)"); break;
                            case 0x22: Serial.print(" (conditionsNotCorrect)"); break;
                            case 0x31: Serial.print(" (requestOutOfRange)"); break;
                            case 0x33: Serial.print(" (securityAccessDenied)"); break;
                            case 0x72: Serial.print(" (generalProgrammingFailure)"); break;
                            case 0x7F: Serial.print(" (serviceNotSupportedInActiveSession)"); break;
                        }
                        Serial.println();
                    }
                    break;
                }
            }
            yield();
        }
        if (!gotResp) Serial.println("UDSTESTWRITE: No response to write");
        Serial.println("UDSTESTWRITE: Done");
        return;
    }
    // UDSSEED — request security access seed from KOMBI
    if (cmd == "UDSSEED") {
        Serial.println("UDSSEED: Requesting security access...");
        uint8_t fc[8] = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};

        // Step 1: Extended session
        uint8_t sess[8] = {0x86, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, sess);
        delay(150);
        // Drain response
        if (!digitalRead(CAN_INT)) {
            unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
            CAN.readMsgBuf(&rId, &rLen, rBuf);
            if (rId == 0x686) {
                int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                Serial.printf("UDSSEED: Session resp:");
                for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                Serial.println();
            }
        }

        // Step 2: Try programming session (10 02) — some modules need this for security
        uint8_t prog[8] = {0x86, 0x02, 0x10, 0x02, 0xAA, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, prog);
        delay(150);
        if (!digitalRead(CAN_INT)) {
            unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
            CAN.readMsgBuf(&rId, &rLen, rBuf);
            if (rId == 0x686) {
                Serial.printf("UDSSEED: ProgSess resp:");
                for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                Serial.println();
            }
        }

        // Step 3: Try multiple security access levels
        // BMW typically uses subfunction 0x01, 0x03, 0x05, 0x11, 0x61
        uint8_t levels[] = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x11, 0x61, 0x63};
        int numLevels = sizeof(levels) / sizeof(levels[0]);

        for (int l = 0; l < numLevels; l++) {
            uint8_t seedReq[8] = {0x86, 0x02, 0x27, levels[l], 0xAA, 0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F0, 0, 8, seedReq);
            Serial.printf("UDSSEED: Trying level 0x%02X... ", levels[l]);

            udsFullLen = 0;
            int expectedLen = 0;
            unsigned long timeout = millis() + 1000;
            bool got = false;
            while (millis() < timeout) {
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        got = true;
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        uint8_t pci = rBuf[ds];
                        if ((pci & 0xF0) == 0x10) {
                            udsFullLen = 0;
                            expectedLen = ((pci & 0x0F) << 8) | rBuf[ds + 1];
                            for (int i = ds + 2; i < rLen && udsFullLen < 64; i++)
                                udsFullResp[udsFullLen++] = rBuf[i];
                            CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                        } else if ((pci & 0xF0) == 0x20) {
                            for (int i = ds + 1; i < rLen && udsFullLen < 64; i++)
                                udsFullResp[udsFullLen++] = rBuf[i];
                        } else if ((pci & 0xF0) == 0x00) {
                            udsFullLen = 0;
                            int sfLen = pci & 0x0F;
                            for (int i = ds + 1; i < ds + 1 + sfLen && i < rLen; i++)
                                udsFullResp[udsFullLen++] = rBuf[i];
                            expectedLen = sfLen;
                        }
                        if (expectedLen > 0 && udsFullLen >= expectedLen) break;
                    }
                }
                yield();
            }
            if (!got) {
                Serial.println("NO RESPONSE");
            } else if (udsFullLen >= 1 && udsFullResp[0] == 0x7F) {
                uint8_t nrc = (udsFullLen >= 3) ? udsFullResp[2] : 0;
                Serial.printf("NRC 0x%02X", nrc);
                switch (nrc) {
                    case 0x12: Serial.print(" (subFunctionNotSupported)"); break;
                    case 0x13: Serial.print(" (incorrectMessageLength)"); break;
                    case 0x22: Serial.print(" (conditionsNotCorrect)"); break;
                    case 0x24: Serial.print(" (requestSequenceError)"); break;
                    case 0x31: Serial.print(" (requestOutOfRange)"); break;
                    case 0x35: Serial.print(" (invalidKey)"); break;
                    case 0x36: Serial.print(" (exceededNumberOfAttempts)"); break;
                    case 0x37: Serial.print(" (requiredTimeDelayNotExpired)"); break;
                }
                Serial.println();
            } else if (udsFullLen >= 2 && udsFullResp[0] == 0x67) {
                // Positive response! 0x67 = SecurityAccess response
                Serial.printf("SEED RECEIVED! Level 0x%02X, seed:", udsFullResp[1]);
                for (int i = 2; i < udsFullLen; i++) Serial.printf(" %02X", udsFullResp[i]);
                Serial.println();
                Serial.println("UDSSEED: *** SEED FOUND — security access is possible! ***");
            } else {
                for (int i = 0; i < udsFullLen; i++) Serial.printf("%02X ", udsFullResp[i]);
                Serial.println();
            }
            delay(200);
            yield();
        }

        // Step 4: Also try a test write to see if it works WITHOUT security
        Serial.println("UDSSEED: Testing write without security (read-back section 3003)...");
        // First re-read 3003 to confirm current value
        uint8_t rd[8] = {0x86, 0x03, 0x22, 0x30, 0x03, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, rd);
        udsFullLen = 0;
        int expectedLen2 = 0;
        unsigned long timeout2 = millis() + 1000;
        while (millis() < timeout2) {
            if (!digitalRead(CAN_INT)) {
                unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                CAN.readMsgBuf(&rId, &rLen, rBuf);
                if (rId == 0x686) {
                    int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                    uint8_t pci = rBuf[ds];
                    if ((pci & 0xF0) == 0x10) {
                        udsFullLen = 0;
                        expectedLen2 = ((pci & 0x0F) << 8) | rBuf[ds + 1];
                        for (int i = ds + 2; i < rLen && udsFullLen < 64; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                    } else if ((pci & 0xF0) == 0x20) {
                        for (int i = ds + 1; i < rLen && udsFullLen < 64; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                    } else if ((pci & 0xF0) == 0x00) {
                        udsFullLen = 0;
                        int sfLen = pci & 0x0F;
                        for (int i = ds + 1; i < ds + 1 + sfLen && i < rLen; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        expectedLen2 = sfLen;
                    }
                    if (expectedLen2 > 0 && udsFullLen >= expectedLen2) break;
                }
            }
            yield();
        }
        Serial.print("UDSSEED: Current 3003:");
        for (int i = 0; i < udsFullLen; i++) Serial.printf(" %02X", udsFullResp[i]);
        Serial.println();

        Serial.println("UDSSEED: Done");
        return;
    }
    // UDSINFO — read multiple DIDs: part#, HW ver, SW ver, coding data
    if (cmd == "UDSINFO") {
        Serial.println("UDSINFO: Reading KOMBI info (0x86)...");
        // Extended session first
        uint8_t sess[8] = {0x86, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA};
        uint8_t fc[8]   = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, sess);
        delay(150);
        // Drain session response
        while (!digitalRead(CAN_INT) == false) { yield(); }
        if (!digitalRead(CAN_INT)) {
            unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
            CAN.readMsgBuf(&rId, &rLen, rBuf);
        }

        // DIDs to read
        uint16_t dids[] = {
            0xF187,  // Part number
            0xF189,  // SW version
            0xF191,  // HW version
            0xF19E,  // Diagnostic variant
            0x1000,  // Coding data block
            0x1001,  // Coding data
            0x1700,  // CAFD header
            0x1704,  // CAFD data
            0x2000,  // Section 0
            0x3000,  // Coding section 3000
            0x3003,  // Coding section 3003 (TLC_VERBAUT lives here)
            0x3004,  // Coding section 3004
        };
        int numDids = sizeof(dids) / sizeof(dids[0]);

        for (int d = 0; d < numDids; d++) {
            uint8_t req[8] = {0x86, 0x03, 0x22,
                              (uint8_t)(dids[d] >> 8), (uint8_t)(dids[d] & 0xFF),
                              0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F0, 0, 8, req);
            Serial.printf("UDSINFO: DID 0x%04X → ", dids[d]);

            udsFullLen = 0;
            int expectedLen = 0;
            unsigned long timeout = millis() + 1000;
            bool got = false;
            while (millis() < timeout) {
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId == 0x686) {
                        got = true;
                        int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                        uint8_t pci = rBuf[ds];
                        if ((pci & 0xF0) == 0x10) {
                            udsFullLen = 0;
                            expectedLen = ((pci & 0x0F) << 8) | rBuf[ds + 1];
                            for (int i = ds + 2; i < rLen && udsFullLen < 64; i++)
                                udsFullResp[udsFullLen++] = rBuf[i];
                            CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                        } else if ((pci & 0xF0) == 0x20) {
                            for (int i = ds + 1; i < rLen && udsFullLen < 64; i++)
                                udsFullResp[udsFullLen++] = rBuf[i];
                        } else if ((pci & 0xF0) == 0x00) {
                            udsFullLen = 0;
                            int sfLen = pci & 0x0F;
                            for (int i = ds + 1; i < ds + 1 + sfLen && i < rLen; i++)
                                udsFullResp[udsFullLen++] = rBuf[i];
                            expectedLen = sfLen;
                        }
                        if (expectedLen > 0 && udsFullLen >= expectedLen) break;
                    }
                }
                yield();
            }
            if (!got) {
                Serial.println("NO RESPONSE");
            } else if (udsFullLen >= 1 && udsFullResp[0] == 0x7F) {
                // Negative response
                Serial.printf("NRC 0x%02X", udsFullLen >= 3 ? udsFullResp[2] : 0);
                if (udsFullLen >= 3) {
                    switch (udsFullResp[2]) {
                        case 0x13: Serial.print(" (incorrectMessageLength)"); break;
                        case 0x14: Serial.print(" (responseTooLong)"); break;
                        case 0x22: Serial.print(" (conditionsNotCorrect)"); break;
                        case 0x31: Serial.print(" (requestOutOfRange)"); break;
                        case 0x33: Serial.print(" (securityAccessDenied)"); break;
                        case 0x72: Serial.print(" (generalProgrammingFailure)"); break;
                    }
                }
                Serial.println();
            } else {
                // Print hex
                for (int i = 0; i < udsFullLen && i < expectedLen; i++) Serial.printf("%02X ", udsFullResp[i]);
                // Also try ASCII if it looks printable
                bool printable = true;
                int dataStart = (udsFullLen >= 3 && udsFullResp[0] == 0x62) ? 3 : 0;
                for (int i = dataStart; i < udsFullLen && i < expectedLen; i++) {
                    if (udsFullResp[i] < 0x20 || udsFullResp[i] > 0x7E) { printable = false; break; }
                }
                if (printable && dataStart < udsFullLen) {
                    Serial.print(" [");
                    for (int i = dataStart; i < udsFullLen && i < expectedLen; i++) Serial.printf("%c", udsFullResp[i]);
                    Serial.print("]");
                }
                Serial.println();
            }
            delay(100);
            yield();
        }
        Serial.println("UDSINFO: Done");
        return;
    }
    // UDSVIN — clean VIN read: extended session + ReadDataByIdentifier F190
    if (cmd == "UDSVIN") {
        Serial.println("UDSVIN: Reading VIN from KOMBI (0x86)...");
        // Step 1: Extended diagnostic session
        uint8_t sess[8] = {0x86, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, sess);
        delay(100);
        // Drain session response
        if (!digitalRead(CAN_INT)) {
            unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
            CAN.readMsgBuf(&rId, &rLen, rBuf);
            if (rId == 0x686) {
                Serial.printf("UDSVIN_SESS(0x686):");
                for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                Serial.println();
            }
        }
        // Step 2: Read VIN (DID F190) — 17 chars = 20 bytes with header = multi-frame
        uint8_t req[8] = {0x86, 0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA};
        uint8_t fc[8]  = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};
        CAN.sendMsgBuf(0x6F0, 0, 8, req);

        udsFullLen = 0;
        int expectedLen = 0;
        unsigned long timeout = millis() + 3000;
        while (millis() < timeout) {
            if (!digitalRead(CAN_INT)) {
                unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                CAN.readMsgBuf(&rId, &rLen, rBuf);
                if (rId == 0x686) {
                    Serial.printf("UDSVIN_RX:");
                    for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                    Serial.println();

                    int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                    uint8_t pci = rBuf[ds];
                    if ((pci & 0xF0) == 0x10) {
                        udsFullLen = 0;
                        expectedLen = ((pci & 0x0F) << 8) | rBuf[ds + 1];
                        for (int i = ds + 2; i < rLen && udsFullLen < 64; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                    } else if ((pci & 0xF0) == 0x20) {
                        for (int i = ds + 1; i < rLen && udsFullLen < 64; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                    } else if ((pci & 0xF0) == 0x00) {
                        udsFullLen = 0;
                        int sfLen = pci & 0x0F;
                        for (int i = ds + 1; i < ds + 1 + sfLen && i < rLen; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        expectedLen = sfLen;
                    }
                    if (expectedLen > 0 && udsFullLen >= expectedLen) {
                        Serial.print("UDSVIN_FULL:");
                        for (int i = 0; i < expectedLen; i++) Serial.printf(" %02X", udsFullResp[i]);
                        Serial.println();
                        if (expectedLen >= 3 && udsFullResp[0] == 0x62 && udsFullResp[1] == 0xF1 && udsFullResp[2] == 0x90) {
                            Serial.print("VIN: ");
                            for (int i = 3; i < expectedLen; i++) Serial.printf("%c", udsFullResp[i]);
                            Serial.println();
                        }
                        break;
                    }
                }
            }
            yield();
        }
        if (udsFullLen == 0) Serial.println("UDSVIN: NO RESPONSE");
        Serial.println("UDSVIN: Done");
        return;
    }
    // UDSSCAN — scan ALL module addresses to find who responds to UDS
    if (cmd == "UDSSCAN") {
        Serial.println("UDSSCAN: Scanning all module addresses...");
        Serial.println("UDSSCAN: Trying TesterPresent (3E 00) on every address");
        // Try every CAN ID from 0x600 to 0x6FE as direct addressing
        // Also try extended addressing on 0x6F0 and 0x6F1 with target byte
        for (int addr = 0; addr < 256; addr++) {
            sendIgnitionStatus();
            sendSafetyCounter();
            sendBacklight();
            counter4Bit++;
            if (counter4Bit >= 14) counter4Bit = 0;

            // Method 1: Normal addressing — send to 0x600+addr
            uint8_t norm[8] = {0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            CAN.sendMsgBuf(0x600 + addr, 0, 8, norm);

            // Method 2: Extended addressing on 0x6F0 with target byte
            uint8_t ext[8] = {(uint8_t)addr, 0x02, 0x3E, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F0, 0, 8, ext);

            // Method 3: Extended addressing on 0x6F1 with target byte
            uint8_t ext2[8] = {(uint8_t)addr, 0x02, 0x3E, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};
            CAN.sendMsgBuf(0x6F1, 0, 8, ext2);

            delay(30);

            // Check for any response in diagnostic range
            for (int check = 0; check < 5; check++) {
                if (!digitalRead(CAN_INT)) {
                    unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                    CAN.readMsgBuf(&rId, &rLen, rBuf);
                    if (rId >= 0x600 && rId <= 0x6FF) {
                        Serial.printf("UDSSCAN: FOUND! addr=0x%02X → response on 0x%03lX:", addr, rId);
                        for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                        Serial.println();
                    }
                }
            }

            if (addr % 32 == 0) Serial.printf("UDSSCAN: scanning 0x%02X...\n", addr);
            yield();
        }
        Serial.println("UDSSCAN: Done.");
        return;
    }
    // UDSBOOT — soft power-cycle + read VIN during wake-up window
    if (cmd == "UDSBOOT") {
        Serial.println("UDSBOOT: Stopping CAN for 3s...");
        delay(3000);
        yield();
        Serial.println("UDSBOOT: Waking cluster + reading VIN...");
        // Only use 0x6F0→0x86 (confirmed working address)
        uint8_t sessExt[8] = {0x86, 0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA};
        uint8_t reqExt[8]  = {0x86, 0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA};
        uint8_t fc[8]      = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};

        unsigned long endTime = millis() + 15000;
        int cycle = 0;
        bool gotUDS = false;
        bool waitingCF = false;  // true after FF received, waiting for CFs
        int expectedLen = 0;
        udsFullLen = 0;

        while (millis() < endTime) {
            sendIgnitionStatus();
            sendSafetyCounter();
            sendBacklight();
            counter4Bit++;
            if (counter4Bit >= 14) counter4Bit = 0;

            // Send UDS request every 500ms, but STOP once we got a First Frame
            if (!waitingCF && cycle % 5 == 0) {
                CAN.sendMsgBuf(0x6F0, 0, 8, sessExt);
                delay(50);
                CAN.sendMsgBuf(0x6F0, 0, 8, reqExt);
            }

            // Check for responses
            if (!digitalRead(CAN_INT)) {
                unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                CAN.readMsgBuf(&rId, &rLen, rBuf);
                if (rId == 0x686) {
                    Serial.printf("UDSBOOT_RX(0x686):");
                    for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                    Serial.println();
                    gotUDS = true;

                    int ds = (rBuf[0] == 0xF0) ? 1 : 0;
                    uint8_t pci = rBuf[ds];

                    if ((pci & 0xF0) == 0x10) {
                        // First Frame — RESET buffer, start fresh
                        udsFullLen = 0;
                        expectedLen = ((pci & 0x0F) << 8) | rBuf[ds + 1];
                        for (int i = ds + 2; i < rLen && udsFullLen < 64; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                        waitingCF = true;
                        Serial.printf("UDSBOOT: FF len=%d, got %d, sent FC\n", expectedLen, udsFullLen);
                    } else if ((pci & 0xF0) == 0x20) {
                        for (int i = ds + 1; i < rLen && udsFullLen < 64; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        Serial.printf("UDSBOOT: CF seq=%d, total %d/%d\n", pci & 0x0F, udsFullLen, expectedLen);
                    } else if ((pci & 0xF0) == 0x00) {
                        udsFullLen = 0;
                        int sfLen = pci & 0x0F;
                        for (int i = ds + 1; i < ds + 1 + sfLen && i < rLen; i++)
                            udsFullResp[udsFullLen++] = rBuf[i];
                        expectedLen = sfLen;
                    }

                    if (expectedLen > 0 && udsFullLen >= expectedLen) {
                        Serial.print("UDSBOOT_FULL:");
                        for (int i = 0; i < expectedLen; i++) Serial.printf(" %02X", udsFullResp[i]);
                        Serial.println();
                        if (expectedLen >= 3 && udsFullResp[0] == 0x62 && udsFullResp[1] == 0xF1 && udsFullResp[2] == 0x90) {
                            Serial.print("VIN: ");
                            for (int i = 3; i < expectedLen; i++) Serial.printf("%c", udsFullResp[i]);
                            Serial.println();
                        }
                        break;
                    }
                }
            }
            delay(100);
            yield();
            cycle++;
        }
        if (!gotUDS) Serial.println("UDSBOOT: No UDS response during boot window");
        Serial.println("UDSBOOT: Done");
        return;
    }
    // CLUSTERTEST — stop all CAN for 5s (cluster sleeps), then restart = lamp test
    if (cmd == "CLUSTERTEST") {
        Serial.println("CLUSTERTEST: Stopping CAN for 5s (cluster will sleep)...");
        delay(5000);
        yield();
        Serial.println("CLUSTERTEST: Restarting — WATCH THE CLUSTER NOW!");
        for (int i = 0; i < 10; i++) {
            sendIgnitionStatus();
            sendBacklight();
            sendSafetyCounter();
            sendLights();
            delay(20);
            yield();
        }
        Serial.println("CLUSTERTEST: Lamp test should have fired. Did you see lane markings?");
        return;
    }
    // BCSTALK — simulate BC stalk long-press to enter cluster test menu
    // Tries CAN IDs 0x5C1 (BC stalk) and 0x1EE with long-press patterns
    if (cmd == "BCSTALK") {
        Serial.println("BCSTALK: Simulating trip reset long-press for test mode...");
        // Try various button codes on 0x1EE (steering wheel)
        // and 0x5C1 (BC stalk) with held state
        uint16_t ids[] = {0x1EE, 0x5C1};
        uint8_t btnCodes[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x4C};
        for (int id = 0; id < 2; id++) {
            for (int b = 0; b < 9; b++) {
                Serial.printf("BCSTALK: ID 0x%03X btn 0x%02X (hold 3s)...\n", ids[id], btnCodes[b]);
                unsigned long holdEnd = millis() + 3000;
                while (millis() < holdEnd) {
                    uint8_t p[] = {btnCodes[b], 0xFF};
                    CAN.sendMsgBuf(ids[id], 0, 2, p);
                    sendIgnitionStatus();
                    sendBacklight();
                    sendSafetyCounter();
                    delay(100);
                    yield();
                }
                // Release
                uint8_t rel[] = {0x00, 0xFF};
                CAN.sendMsgBuf(ids[id], 0, 2, rel);
                delay(500);
                yield();
            }
        }
        Serial.println("BCSTALK: Done. Did anything happen?");
        return;
    }
    // Brute-force scan CRC XOR for 0x327 lane assist — watch the cluster!
    // LANESCAN — scan CAN IDs 0x300-0x3FF with all 256 CRC XOR values
    // Also tries 0x18A, 0x239, 0x1A6 (other known ADAS IDs)
    // Watch the cluster for lane markings!
    if (cmd == "LANESCAN") {
        uint16_t canIds[] = {0x327, 0x345, 0x18A, 0x239, 0x1A6, 0x31B, 0x317, 0x337, 0x347};
        int numIds = sizeof(canIds) / sizeof(canIds[0]);
        Serial.println("LANESCAN: Scanning multiple CAN IDs x 256 XOR values...");
        for (int idIdx = 0; idIdx < numIds; idIdx++) {
            Serial.printf("LANESCAN: === CAN ID 0x%03X ===\n", canIds[idIdx]);
            for (int xorVal = 0; xorVal < 256; xorVal++) {
                uint8_t patterns[][2] = {{0xFF, 0xFF}, {0x03, 0x03}};
                for (int p = 0; p < 2; p++) {
                    // Try DLC=4 (CRC + counter + 2 data)
                    unsigned char d[] = {(uint8_t)(0x50 | counter4Bit), patterns[p][0], patterns[p][1]};
                    unsigned char msg4[] = {crc8Calculator.get_crc8(d, 3, (uint8_t)xorVal), d[0], d[1], d[2]};
                    CAN.sendMsgBuf(canIds[idIdx], 0, 4, msg4);
                    sendIgnitionStatus();
                    sendSafetyCounter();
                    counter4Bit++;
                    if (counter4Bit >= 14) counter4Bit = 0;
                    delay(20);
                    yield();
                }
                // Also try DLC=5 and DLC=8 with same XOR
                unsigned char d5[] = {(uint8_t)(0x50 | counter4Bit), 0xFF, 0xFF, 0xFF};
                unsigned char msg5[] = {crc8Calculator.get_crc8(d5, 4, (uint8_t)xorVal), d5[0], d5[1], d5[2], d5[3]};
                CAN.sendMsgBuf(canIds[idIdx], 0, 5, msg5);
                sendBacklight();
                counter4Bit++;
                if (counter4Bit >= 14) counter4Bit = 0;
                delay(20);
                yield();
            }
        }
        Serial.println("LANESCAN: Done.");
        return;
    }
    // LANESCAN:0x327 — scan only one specific CAN ID with all XOR values
    if (cmd.startsWith("LANESCAN:")) {
        uint16_t scanId = (uint16_t)strtoul(cmd.substring(9).c_str(), 0, 16);
        Serial.printf("LANESCAN: CAN ID 0x%03X, all 256 XOR values...\n", scanId);
        for (int xorVal = 0; xorVal < 256; xorVal++) {
            uint8_t patterns[][2] = {{0xFF, 0xFF}, {0x03, 0x03}, {0x0F, 0x0F}, {0xF0, 0xF0}};
            for (int p = 0; p < 4; p++) {
                unsigned char d[] = {(uint8_t)(0x50 | counter4Bit), patterns[p][0], patterns[p][1]};
                unsigned char msg[] = {crc8Calculator.get_crc8(d, 3, (uint8_t)xorVal), d[0], d[1], d[2]};
                CAN.sendMsgBuf(scanId, 0, 4, msg);
                sendIgnitionStatus();
                sendBacklight();
                sendSafetyCounter();
                counter4Bit++;
                if (counter4Bit >= 14) counter4Bit = 0;
                delay(25);
                yield();
            }
            if (xorVal % 32 == 0) Serial.printf("LANESCAN: XOR 0x%02X...\n", xorVal);
            yield();
        }
        Serial.printf("LANESCAN: Done 0x%03X.\n", scanId);
        return;
    }
    // LANEXOR:0x4A or LANEXOR:0x4A:0x327 — test one XOR value (optionally on specific CAN ID)
    if (cmd.startsWith("LANEXOR:")) {
        String params = cmd.substring(8);
        int col = params.indexOf(':');
        uint8_t xorVal;
        uint16_t testId = 0x327;
        if (col > 0) {
            xorVal = (uint8_t)strtoul(params.substring(0, col).c_str(), 0, 16);
            testId = (uint16_t)strtoul(params.substring(col + 1).c_str(), 0, 16);
        } else {
            xorVal = (uint8_t)strtoul(params.c_str(), 0, 16);
        }
        Serial.printf("LANEXOR: XOR 0x%02X on CAN 0x%03X for 10s...\n", xorVal, testId);
        unsigned long endTime = millis() + 10000;
        while (millis() < endTime) {
            uint8_t patterns[][2] = {{0xFF, 0xFF}, {0x03, 0x03}, {0x30, 0x30}, {0xF0, 0xF0}, {0x0F, 0x0F}, {0xCC, 0xCC}};
            for (int p = 0; p < 6; p++) {
                unsigned char d[] = {(uint8_t)(0x50 | counter4Bit), patterns[p][0], patterns[p][1]};
                unsigned char msg[] = {crc8Calculator.get_crc8(d, 3, xorVal), d[0], d[1], d[2]};
                CAN.sendMsgBuf(testId, 0, 4, msg);
                sendIgnitionStatus();
                sendBacklight();
                sendSafetyCounter();
                counter4Bit++;
                if (counter4Bit >= 14) counter4Bit = 0;
                delay(80);
                yield();
            }
            yield();
        }
        Serial.printf("LANEXOR: Done XOR 0x%02X on 0x%03X\n", xorVal, testId);
        return;
    }
    if (cmd == "CCSET")    { if (currentSpeed > 0) { cruiseControlActive = true; cruiseSetSpeed = currentSpeed; Serial.println("OK:CCSET:ACTIVE@" + String(cruiseSetSpeed) + "km/h"); } else Serial.println("ERROR:CCSET:SPEED_ZERO"); return; }
    if (cmd == "CCRESUME") { if (cruiseSetSpeed > 0) { cruiseControlActive = true; Serial.println("OK:CCRESUME:ACTIVE@" + String(cruiseSetSpeed) + "km/h"); } else Serial.println("ERROR:CCRESUME:NO_SET_SPEED"); return; }
    if (cmd == "CCCANCEL") { cruiseControlActive = false; Serial.println("OK:CCCANCEL:SPEED_KEPT@" + String(cruiseSetSpeed) + "km/h"); return; }
    if (cmd == "CCOFF")    { cruiseControlActive = false; cruiseSetSpeed = 0; Serial.println("OK:CCOFF"); return; }
    if (cmd == "CCPLUS")   { if (cruiseControlActive) { cruiseSetSpeed++; Serial.println("OK:CCPLUS:" + String(cruiseSetSpeed) + "km/h"); } return; }
    if (cmd == "CCMINUS")  { if (cruiseControlActive && cruiseSetSpeed > 0) { cruiseSetSpeed--; Serial.println("OK:CCMINUS:" + String(cruiseSetSpeed) + "km/h"); } return; }

    int sep = cmd.indexOf(':');
    if (sep <= 0) { Serial.println("ERROR:INVALID_FORMAT"); return; }
    String key = cmd.substring(0, sep);
    String val = cmd.substring(sep + 1);
    bool on = (val == "1" || val == "ON");

    if      (key == "SPEED")     { currentSpeed = constrain(val.toInt(), 0, 260); }
    else if (key == "RPM")       { currentRPM   = constrain(val.toInt(), 0, 6900); }
    else if (key == "GEAR")      {
        if      (val == "P") { currentGear = 10; shifter_status_current = SHIFTER_PARKING; }
        else if (val == "R") { currentGear = 11; shifter_status_current = SHIFTER_REVERSE; }
        else if (val == "N") { currentGear = 12; shifter_status_current = SHIFTER_NEUTRAL; }
        else if (val == "D") { currentGear = 13; shifter_status_current = SHIFTER_DRIVE; }
        else                 { currentGear = val.toInt(); shifter_status_current = SHIFTER_DRIVE; }
    }
    else if (key == "TEMP")      { engineTemperature = constrain(val.toInt(), 0, 200); }
    else if (key == "FUEL")      { fuelLevel = constrain(val.toInt(), 0, 100); }
    else if (key == "MODE")      { driveMode = val.toInt(); }
    else if (key == "DOOR")      { doorOpen = on; }
    else if (key == "DSC")       { dscAlert = on; }
    else if (key == "HANDBRAKE") { handbrake = on; }
    else if (key == "CHECKENG")  { checkEngine = on; }
    else if (key == "DSCOFF")    { dscOff = on; }
    else if (key == "PARKBRAKE") { parkBrake = on; }
    else if (key == "HIGHBEAM")  { highBeam = on; }
    else if (key == "FRONTFOG")  { frontFogLight = on; }
    else if (key == "REARFOG")   { rearFogLight = on; }
    else if (key == "LEFTBLINK") { leftBlinker = on; }
    else if (key == "RIGHTBLINK"){ rightBlinker = on; }
    else if (key == "SOSCALL")   { sosCall = on; }
    else if (key == "CHASSIS")   { chassisWarning = on; }
    else if (key == "CRUISE")    { cruiseWarning = on; }
    else if (key == "BRAKE")     { brakeFailure = on; }
    else if (key == "DIPPED")    { dippedBeamFailure = on; }
    else if (key == "TRAILER")   { trailerReversing = on; }
    else if (key == "RESTRAINT") { restraintRear = on; }
    else if (key == "FASTEN")    { fastenSeatbelts = on; }
    else if (key == "TEST73")    { warning73 = on; }
    else if (key == "TEST85")    { warning85 = on; }
    else if (key == "TEST88")    { warning88 = on; }
    else if (key == "IGN")       { ignitionOn = on; }
    else if (key == "BRIGHT")    { backlightBrightness = constrain(val.toInt(), 0, 100); }
    else if (key == "ALERT")     { sendCC(val.toInt(), true); }
    else if (key == "CLEARALERT"){ sendCC(val.toInt(), false); }
    else if (key == "TIME") {
        int colon = val.indexOf(':');
        if (colon > 0) {
            clockHours   = val.substring(0, colon).toInt() % 24;
            clockMinutes = val.substring(colon + 1).toInt() % 60;
        }
    }
    // DATE:DD.MM.YYYY  e.g. DATE:26.02.2026
    else if (key == "DATE") {
        int dot1 = val.indexOf('.');
        int dot2 = val.indexOf('.', dot1 + 1);
        if (dot1 > 0 && dot2 > dot1) {
            dateDay   = constrain(val.substring(0, dot1).toInt(), 1, 31);
            dateMonth = constrain(val.substring(dot1 + 1, dot2).toInt(), 1, 12);
            dateYear  = constrain(val.substring(dot2 + 1).toInt(), 2000, 2099);
        }
    }
    // OVERSPEED:120  — set overspeed warning threshold (0 = off)
    else if (key == "OVERSPEED") { overspeedThreshold = constrain(val.toInt(), 0, 260); }
    // LANE:ON / LANE:0  — toggle lane assist display
    else if (key == "LANE") { laneAssistActive = on; }
    // LANELEFT:0-3  — left lane marking (0=off 1=grey 2=green 3=yellow)
    else if (key == "LANELEFT")  { laneLeftStatus  = constrain(val.toInt(), 0, 0xFF); }
    // LANERIGHT:0-3 — right lane marking
    else if (key == "LANERIGHT") { laneRightStatus = constrain(val.toInt(), 0, 0xFF); }
    // UDS diagnostic commands — talk directly to KOMBI (module 0x86)
    // UDSRAW:hex bytes  — send raw UDS to KOMBI via ISO-TP
    // Uses extended addressing: TX on 0x6F0, first byte=0x86 (target)
    // Listens on 0x686, handles multi-frame with Flow Control
    else if (key == "UDSRAW") {
        // Parse UDS payload hex bytes
        uint8_t udsPayload[6];
        int payloadLen = 0;
        String hex = val;
        hex.trim();
        while (hex.length() > 0 && payloadLen < 6) {
            int sp = hex.indexOf(' ');
            String byteStr = (sp > 0) ? hex.substring(0, sp) : hex;
            udsPayload[payloadLen++] = (uint8_t)strtoul(byteStr.c_str(), 0, 16);
            hex = (sp > 0) ? hex.substring(sp + 1) : "";
            hex.trim();
        }

        // Build ISO-TP Single Frame with extended addressing
        // CAN ID 0x6F0 (tester F0), byte 0 = 0x86 (KOMBI target)
        uint8_t d[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        d[0] = 0x86;                       // target module (KOMBI)
        d[1] = (uint8_t)payloadLen;         // ISO-TP SF PCI
        for (int i = 0; i < payloadLen; i++) d[2 + i] = udsPayload[i];
        CAN.sendMsgBuf(0x6F0, 0, 8, d);
        Serial.print("UDS_TX(0x6F0→86):");
        for (int i = 0; i < 8; i++) Serial.printf(" %02X", d[i]);
        Serial.println();

        // Also try functional broadcast (0x6FF) as backup
        uint8_t df[8] = {0};
        df[0] = (uint8_t)payloadLen;
        for (int i = 0; i < payloadLen; i++) df[1 + i] = udsPayload[i];
        CAN.sendMsgBuf(0x6FF, 0, 8, df);

        // Collect full response with ISO-TP multi-frame support
        udsFullLen = 0;
        int expectedLen = 0;
        bool gotResponse = false;
        unsigned long udsTimeout = millis() + 3000;

        while (millis() < udsTimeout) {
            if (!digitalRead(CAN_INT)) {
                unsigned long rId; unsigned char rLen; unsigned char rBuf[8];
                CAN.readMsgBuf(&rId, &rLen, rBuf);

                if (rId >= 0x600 && rId <= 0x6FF) {
                    Serial.printf("UDS_RX(0x%03lX):", rId);
                    for (int i = 0; i < rLen; i++) Serial.printf(" %02X", rBuf[i]);
                    Serial.println();
                    gotResponse = true;

                    // Check if extended addressing (first byte = F0 = tester addr)
                    int dataStart = 0;
                    if (rBuf[0] == 0xF0) dataStart = 1;  // skip ext addr byte

                    uint8_t pci = rBuf[dataStart];

                    // First Frame (0x1X) — reset buffer
                    if ((pci & 0xF0) == 0x10) {
                        udsFullLen = 0;
                        expectedLen = ((pci & 0x0F) << 8) | rBuf[dataStart + 1];
                        for (int i = dataStart + 2; i < rLen && udsFullLen < expectedLen; i++) {
                            udsFullResp[udsFullLen++] = rBuf[i];
                        }
                        // Send Flow Control: CTS, BS=0, STmin=10ms
                        uint8_t fc[8] = {0x86, 0x30, 0x00, 0x0A, 0xAA, 0xAA, 0xAA, 0xAA};
                        CAN.sendMsgBuf(0x6F0, 0, 8, fc);
                        Serial.println("UDS_FC: Sent Flow Control");
                    }
                    // Consecutive Frame (0x2X)
                    else if ((pci & 0xF0) == 0x20) {
                        for (int i = dataStart + 1; i < rLen && udsFullLen < expectedLen; i++) {
                            udsFullResp[udsFullLen++] = rBuf[i];
                        }
                    }
                    // Single Frame (0x0X) — short response, reset buffer
                    else if ((pci & 0xF0) == 0x00) {
                        udsFullLen = 0;
                        int sfLen = pci & 0x0F;
                        for (int i = dataStart + 1; i < dataStart + 1 + sfLen && i < rLen; i++) {
                            udsFullResp[udsFullLen++] = rBuf[i];
                        }
                        expectedLen = sfLen;
                    }

                    // Check if we have the complete message
                    if (expectedLen > 0 && udsFullLen >= expectedLen) {
                        Serial.print("UDS_FULL:");
                        for (int i = 0; i < udsFullLen; i++) Serial.printf(" %02X", udsFullResp[i]);
                        Serial.println();
                        Serial.printf("UDS_FULL: %d bytes received\n", udsFullLen);
                        break;
                    }
                }
            }
            yield();
        }
        if (!gotResponse) Serial.println("UDS_RX: NO RESPONSE");
    }
    else if (key == "STATUS") {
        Serial.println("STATUS:");
        Serial.println("SPEED:"         + String(currentSpeed));
        Serial.println("RPM:"           + String(currentRPM));
        Serial.println("GEAR:"          + String(currentGear));
        Serial.println("TEMP:"          + String(engineTemperature));
        Serial.println("FUEL:"          + String(fuelLevel));
        Serial.println("MODE:"          + String(driveMode));
        Serial.println("CRUISE_ACTIVE:" + String(cruiseControlActive));
        Serial.println("CRUISE_SET:"    + String(cruiseSetSpeed));
        Serial.println("SHIFTER_AUTO:"  + String(shifter_mode_auto));
        Serial.println("PARKING_ENG:"   + String(parking_engaged));
        Serial.println("IGN:"           + String(ignitionOn));
        Serial.println("BRIGHT:"        + String(backlightBrightness));
        Serial.println("CLOCK:"         + String(clockHours) + ":" + String(clockMinutes));
        Serial.println("DATE:"          + String(dateDay) + "." + String(dateMonth) + "." + String(dateYear));
        Serial.println("OVERSPEED:"     + String(overspeedThreshold));
        Serial.println("LANE:"          + String(laneAssistActive));
        Serial.println("LANELEFT:"      + String(laneLeftStatus));
        Serial.println("LANERIGHT:"     + String(laneRightStatus));
        Serial.println("END");
        return;
    }
    else { Serial.println("ERROR:UNKNOWN_CMD:" + key); return; }

    Serial.println("OK:" + key + ":" + val);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("BMW F10 Diesel + Physical Shifter CAN v4.0");

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

    if      (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("CAN: 500kbps OK");
    else if (CAN.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) Serial.println("CAN: 100kbps OK");
    else { Serial.println("CAN: FAILED"); while (1) delay(1000); }

    CAN.setMode(MCP_NORMAL);
    pinMode(CAN_INT, INPUT_PULLUP);

    // Wake-up burst
    for (int i = 0; i < 10; i++) {
        sendIgnitionStatus();
        sendBacklight();
        sendSafetyCounter();
        sendLights();
        delay(20);
    }

    turnShifterBacklight();
    Serial.println("READY");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    // ── Read physical shifter if a frame is waiting ───────────────────────────
    if (!digitalRead(CAN_INT)) {
        CAN.readMsgBuf(&rxId, &rxLen, rxBuf);
        processShifterCANFrame();
    }

    // ── Serial commands ───────────────────────────────────────────────────────
    processSerialCommand();

    // ── Cluster messages ──────────────────────────────────────────────────────
    send2C5Message();
    send1B3Message();
    send2CAMessage();
    send393Message();
    sendACC();
    sendIgnitionStatus();
    sendBacklight();
    // Periodically refresh shifter backlight (every ~1s)
    if (++backlightCycleCount >= 10) {
        turnShifterBacklight();
        backlightCycleCount = 0;
    }
    sendSpeed();
    sendRPM();
    sendTransmission();
    // N gear cyclic frame (from CarCluster) — must be sent continuously while in N
    if (currentGear == 12) {
        unsigned char nd[] = {0xF0 | counter4Bit, 0x60, 0xFC, 0xFF};
        unsigned char np[] = {crc8Calculator.get_crc8(nd, 4, 0x5A), nd[0], nd[1], nd[2], nd[3]};
        CAN.sendMsgBuf(0x178, 0, 5, np);
    }
    sendAlerts();
    sendSmartAlerts();
    sendLights();
    sendBlinkers();
    sendABS();
    sendSafetyCounter();
    sendPowerSteering();
    sendCruiseControl();
    sendAirbag();
    sendSeatbeltSystem();
    sendTPMS();
    sendEngineTemperature();
    sendOilTemperature();
    sendParkBrakeStatus();
    sendDriveMode();
    sendFuel();
    sendDistanceTravelled();
    sendLaneAssist();

    // ── Slow-cycle messages (every ~1s) ──────────────────────────────────────
    if (millis() - lastSlowUpdate >= 1000) {
        sendTimeFrame();
        sendSteeringWheelButton(false);  // idle keepalive
        lastSlowUpdate = millis();
    }

    counter4Bit++;
    if (counter4Bit >= 14) counter4Bit = 0;
    count++;
    if (count >= 254) count = 0;

    delay(100);
}
