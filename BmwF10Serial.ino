/*
 * BMW F10 2012 CAN Bus Controller for ESP32-S3
 * Using External MCP2515 CAN Module (8MHz crystal)
 * 
 * Wiring:
 * MCP2515 SCK  -> GPIO 36
 * MCP2515 MISO -> GPIO 37
 * MCP2515 MOSI -> GPIO 35
 * MCP2515 CS   -> GPIO 39
 * 
 * Diesel F10 - All warning lights OFF by default
 * REAR SEATBELT WARNING DISABLED
 */

#include <SPI.h>
#include "mcp_can.h"

// SPI Pins
#define SPI_SCK   36
#define SPI_MISO  37
#define SPI_MOSI  35
#define SPI_CS    39

MCP_CAN CAN(SPI_CS);

// Counters
uint8_t counter4Bit = 0;
uint8_t count = 0;
uint8_t accCounter = 0;

// Vehicle state
int currentSpeed = 0;
int currentRPM = 1000;
int currentGear = 10; // 10=P, 11=R, 12=N, 13=D
int engineTemperature = 100; // 100 = normal operating temp
uint8_t driveMode = 2; // 2=Comfort
int fuelLevel = 100; // 100%

// Cruise control state
bool cruiseControlActive = false;
int cruiseSetSpeed = 0;
// Manual test values for 0x289 message
uint8_t test289_b1 = 0xE0;
uint8_t test289_b2 = 0xE0;
uint8_t test289_b3 = 0xE1;
uint8_t test289_b4 = 0x00;
uint8_t test289_b5 = 0xEC;
uint8_t test289_b6 = 0x01;

// Warning lights - ALL OFF
bool doorOpen = false;
bool dscAlert = false;
bool handbrake = false;
bool checkEngine = false;
bool dscOff = false;
bool parkBrake = false;
bool sosCall = false; // SOS call active/inactive
bool chassisWarning = false; // Chassis warning
bool cruiseWarning = false; // Cruise control keep distance warning
bool brakeFailure = false; // Brake failure warning
bool dippedBeamFailure = false; // Dipped beam failure warning
bool trailerReversing = false; // Trailer reversing light failure
bool restraintRear = false; // Restraint system rear failure
bool fastenSeatbelts = false; // Fasten seatbelts warning
bool warning73 = false; // Test ID 73
bool warning85 = false; // Test ID 85
bool warning88 = false; // Test ID 88

// Seatbelt status - all buckled to prevent warnings
bool driverSeatbelt = true;
bool passengerSeatbelt = true;
bool rearLeftSeatbelt = true;
bool rearCenterSeatbelt = true;
bool rearRightSeatbelt = true;

// Lights - Main lights ON
bool mainLights = true;
bool highBeam = false;
bool rearFogLight = false;
bool frontFogLight = false;
bool leftBlinker = false;
bool rightBlinker = false;

// Fuel ranges for F10 Diesel (not Mini) - from original code
uint8_t inFuelRange[3] = {0, 50, 100};
uint8_t outFuelRange[3] = {37, 18, 4};

// Distance counter
uint32_t distanceTravelledCounter = 0;

// CRC8 Calculator
class CRC8 {
private:
  uint8_t crcTable[256];
  
public:
  CRC8() {
    // Initialize CRC table
    for (int dividend = 0; dividend < 256; ++dividend) {
      uint8_t remainder = dividend << 0; // WIDTH - 8 where WIDTH = 8
      for (uint8_t bit = 8; bit > 0; --bit) {  
        if (remainder & 0x80) { // TOPBIT
          remainder = (remainder << 1) ^ 0x1D; // POLYNOMIAL
        } else {
          remainder = (remainder << 1);
        }
      }
      crcTable[dividend] = remainder;
    }
  }
  
  uint8_t get_crc8(uint8_t const message[], int nBytes, uint8_t final) {
    uint8_t data;
    uint8_t remainder = 0xFF;
    for (int byte = 0; byte < nBytes; ++byte) {
      data = message[byte] ^ remainder;
      remainder = crcTable[data];
    }
    return remainder ^ final;
  }
};

CRC8 crc8Calculator;

template <typename T>
T multiMap(T val, uint8_t* _in, uint8_t* _out, uint8_t size) {
  if (val <= _in[0]) return _out[0];
  if (val >= _in[size-1]) return _out[size-1];
  uint8_t pos = 1;
  while(val > _in[pos]) pos++;
  if (val == _in[pos]) return _out[pos];
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

// 0x12F - Ignition Status (ALWAYS ON)
void sendIgnitionStatus() {
  unsigned char ignitionWithoutCRC[] = { 
    0x80 | counter4Bit, 0x8A, 0xDD, 0xF1, 0x01, 0x30, 0x06 
  };
  uint8_t crc = crc8Calculator.get_crc8(ignitionWithoutCRC, 7, 0x44);
  unsigned char ignitionWithCRC[] = { 
    crc, ignitionWithoutCRC[0], ignitionWithoutCRC[1], ignitionWithoutCRC[2],
    ignitionWithoutCRC[3], ignitionWithoutCRC[4], ignitionWithoutCRC[5], ignitionWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x12F, 0, 8, ignitionWithCRC);
}

// 0x202 - Backlight brightness
void sendBacklight() {
  unsigned char msg202[] = { 0xFD, 0xFF };
  CAN.sendMsgBuf(0x202, 0, 2, msg202);
}

// 0x1A1 - Speed
void sendSpeed() {
  uint16_t calculatedSpeed = (double)currentSpeed * 64.01;
  uint8_t lowByte = calculatedSpeed & 0xFF;
  uint8_t highByte = (calculatedSpeed >> 8) & 0xFF;
  unsigned char speedWithoutCRC[] = { 
    0xC0 | counter4Bit, lowByte, highByte, (currentSpeed == 0 ? 0x81 : 0x91)
  };
  uint8_t crc = crc8Calculator.get_crc8(speedWithoutCRC, 4, 0xA9);
  unsigned char speedWithCRC[] = { 
    crc, speedWithoutCRC[0], speedWithoutCRC[1], speedWithoutCRC[2], speedWithoutCRC[3]
  };
  CAN.sendMsgBuf(0x1A1, 0, 5, speedWithCRC);
}

// 0x0F3 - RPM
void sendRPM() {
  int calculatedGear = 0;
  switch (currentGear) {
    case 0: calculatedGear = 0; break;
    case 1 ... 9: calculatedGear = currentGear + 4; break;
    case 11: calculatedGear = 2; break; // R
    case 12: calculatedGear = 1; break; // N
  }
  int rpmValue = map(currentRPM, 0, 6900, 0x00, 0x2B);
  unsigned char rpmWithoutCRC[] = { 
    0x60 | counter4Bit, rpmValue, 0xC0, 0xF0, calculatedGear, 0xFF, 0xFF
  };
  uint8_t crc = crc8Calculator.get_crc8(rpmWithoutCRC, 7, 0x7A);
  unsigned char rpmWithCRC[] = { 
    crc, rpmWithoutCRC[0], rpmWithoutCRC[1], rpmWithoutCRC[2],
    rpmWithoutCRC[3], rpmWithoutCRC[4], rpmWithoutCRC[5], rpmWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x0F3, 0, 8, rpmWithCRC);
}

// 0x3FD - Transmission
void sendTransmission() {
  uint8_t selectedGear = 0;
  switch (currentGear) {
    case 1 ... 9: selectedGear = 0x81; break;
    case 10: selectedGear = 0x20; break; // P
    case 11: selectedGear = 0x40; break; // R
    case 12: selectedGear = 0x60; break; // N
    case 13: selectedGear = 0x80; break; // D
    default: selectedGear = 0x20; break;
  }
  unsigned char transmissionWithoutCRC[] = { counter4Bit, selectedGear, 0xFC, 0xFF };
  uint8_t crc = crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6);
  unsigned char transmissionWithCRC[] = { 
    crc, transmissionWithoutCRC[0], transmissionWithoutCRC[1],
    transmissionWithoutCRC[2], transmissionWithoutCRC[3]
  };
  CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
}

// 0x5C0 - Alerts (using reference implementation approach)
void sendAlerts() {
  // Check control messages - same CAN ID 0x5C0 used for variety of messages
  // Sending 0x29 on byte 3 sets the alert, 0x28 clears it
  // Known IDs: 34=check engine, 35/215=DSC, 36=DSC OFF, 24=park brake yellow, 71=park brake red, 77=seatbelt, 299=SOS call system
  
  // Door alert
  if (doorOpen) {
    uint8_t message[] = { 0x40, 0x0F, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 0x0F, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  }
  
  // DSC alert
  if (dscAlert) {
    uint8_t message[] = { 0x40, 215, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 215, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  }
  
  // Handbrake (red) - only for diesel F10, not Mini
  if (handbrake) {
    uint8_t message[] = { 0x40, 71, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 71, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  }
  
  // Check Engine
  if (checkEngine) {
    uint8_t message[] = { 0x40, 34, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 34, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  }
  
  // DSC OFF indicator
  if (dscOff) {
    uint8_t message[] = { 0x40, 36, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 36, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  }
  
  // Park Brake (yellow)
  if (parkBrake) {
    uint8_t message[] = { 0x40, 24, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 24, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, message);
  }
  
  // Seatbelt indicator - ALWAYS CLEAR (ID 77)
  uint8_t seatBeltMsg[] = { 0x40, 77, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
  CAN.sendMsgBuf(0x5C0, 0, 8, seatBeltMsg);
  
  // Test ID 73 individually (still unknown - keeping for testing)
  if (warning73) {
    uint8_t msg73[] = { 0x40, 73, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg73);
  } else {
    uint8_t msg73[] = { 0x40, 73, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg73);
  }
  
  // Cruise control keep distance (ID 85 - CONFIRMED!)
  if (cruiseWarning) {
    uint8_t cruiseMsg[] = { 0x40, 85, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, cruiseMsg);
  } else {
    uint8_t cruiseMsg[] = { 0x40, 85, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, cruiseMsg);
  }
  
  // Dipped beam left (ID 88 - CONFIRMED!)
  if (dippedBeamFailure) {
    uint8_t dippedMsg[] = { 0x40, 88, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg);
  } else {
    uint8_t dippedMsg[] = { 0x40, 88, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg);
  }
  
  // Fasten seatbelts warning (IDs 86, 87, 91)
  if (dippedBeamFailure) {
    uint8_t dippedMsg1[] = { 0x40, 98, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg1);
    
    uint8_t dippedMsg2[] = { 0x40, 99, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg2);
    
    uint8_t dippedMsg3[] = { 0x40, 100, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg3);
  } else {
    uint8_t dippedMsg1[] = { 0x40, 98, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg1);
    
    uint8_t dippedMsg2[] = { 0x40, 99, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg2);
    
    uint8_t dippedMsg3[] = { 0x40, 100, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, dippedMsg3);
  }
  
  // Fasten seatbelts warning (IDs 86, 87, 91)
  if (fastenSeatbelts) {
    uint8_t seatbeltMsg1[] = { 0x40, 86, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltMsg1);
    
    uint8_t seatbeltMsg2[] = { 0x40, 87, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltMsg2);
    
    uint8_t seatbeltMsg3[] = { 0x40, 91, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltMsg3);
  } else {
    uint8_t seatbeltMsg1[] = { 0x40, 86, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltMsg1);
    
    uint8_t seatbeltMsg2[] = { 0x40, 87, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltMsg2);
    
    uint8_t seatbeltMsg3[] = { 0x40, 91, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltMsg3);
  }
  
  // Brake failure warning (ID 72, 74)
  if (brakeFailure) {
    uint8_t brakeMsg1[] = { 0x40, 72, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, brakeMsg1);
    
    uint8_t brakeMsg2[] = { 0x40, 74, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, brakeMsg2);
  } else {
    uint8_t brakeMsg1[] = { 0x40, 72, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, brakeMsg1);
    
    uint8_t brakeMsg2[] = { 0x40, 74, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, brakeMsg2);
  }
  
  // Trailer reversing light failure (IDs 76, 89, 90)
  if (trailerReversing) {
    uint8_t trailerMsg1[] = { 0x40, 76, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, trailerMsg1);
    
    uint8_t trailerMsg2[] = { 0x40, 89, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, trailerMsg2);
    
    uint8_t trailerMsg3[] = { 0x40, 90, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, trailerMsg3);
  } else {
    uint8_t trailerMsg1[] = { 0x40, 76, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, trailerMsg1);
    
    uint8_t trailerMsg2[] = { 0x40, 89, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, trailerMsg2);
    
    uint8_t trailerMsg3[] = { 0x40, 90, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, trailerMsg3);
  }
  
  // Restraint system rear failure (IDs 92, 93, 94)
  if (restraintRear) {
    uint8_t restraintMsg1[] = { 0x40, 92, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, restraintMsg1);
    
    uint8_t restraintMsg2[] = { 0x40, 93, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, restraintMsg2);
    
    uint8_t restraintMsg3[] = { 0x40, 94, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, restraintMsg3);
  } else {
    uint8_t restraintMsg1[] = { 0x40, 92, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, restraintMsg1);
    
    uint8_t restraintMsg2[] = { 0x40, 93, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, restraintMsg2);
    
    uint8_t restraintMsg3[] = { 0x40, 94, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, restraintMsg3);
  }
  
  // Chassis warning (ID 299 = 0x12B)
  // This is the actual chassis warning
  if (chassisWarning) {
    uint8_t chassisMsg[] = { 0x40, 299 & 0xFF, 0x00, 0x29, 0xFF, 0xF7, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, chassisMsg);
  } else {
    uint8_t chassisMsg[] = { 0x40, 299 & 0xFF, 0x00, 0x28, 0xFF, 0xF7, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, chassisMsg);
  }
  
  // SOS Call System (trying IDs 95, 96, 97, 196)
  if (sosCall) {
    uint8_t sosMsg1[] = { 0x40, 196, 0x00, 0x29, 0xFF, 0xF7, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, sosMsg1);
    
    uint8_t sosMsg2[] = { 0x40, 95, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, sosMsg2);
    
    uint8_t sosMsg3[] = { 0x40, 96, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, sosMsg3);
  } else {
    uint8_t sosMsg1[] = { 0x40, 196, 0x00, 0x28, 0xFF, 0xF7, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, sosMsg1);
    
    uint8_t sosMsg2[] = { 0x40, 95, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, sosMsg2);
    
    uint8_t sosMsg3[] = { 0x40, 96, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, sosMsg3);
  }
}

// 0x21A - Lights
void sendLights() {
  uint8_t lightStatus = (highBeam << 1) | (mainLights << 2) | (frontFogLight << 5) | (rearFogLight << 6);
  unsigned char lightsWithoutCRC[] = { lightStatus, 0xC0, 0xF7 };
  CAN.sendMsgBuf(0x21A, 0, 3, lightsWithoutCRC);
}

// 0x1F6 - Blinkers
void sendBlinkers() {
  uint8_t blinkerStatus = (leftBlinker == 0 && rightBlinker == 0) 
                          ? 0x80 : (0x81 | leftBlinker << 4 | rightBlinker << 5);
  unsigned char blinkersWithoutCRC[] = { blinkerStatus, 0xF0 };
  CAN.sendMsgBuf(0x1F6, 0, 2, blinkersWithoutCRC);
}

// 0x3A7 - Drive Mode
void sendDriveMode() {
  unsigned char modeWithoutCRC[] = { 0xF0 | counter4Bit, 0, 0, driveMode, 0x11, 0xC0 };
  unsigned char modeWithCRC[] = { 
    crc8Calculator.get_crc8(modeWithoutCRC, 6, 0x4A), 
    modeWithoutCRC[0], modeWithoutCRC[1], modeWithoutCRC[2],
    modeWithoutCRC[3], modeWithoutCRC[4], modeWithoutCRC[5]
  };
  CAN.sendMsgBuf(0x3A7, 0, 7, modeWithCRC);
}

// 0x349 - Fuel Level (using original method with 16-bit value)
void sendFuel() {
  // MultiMap returns 8-bit, but we need to treat it as 16-bit for hi8/lo8
  uint16_t fuelQuantityLiters = multiMap<uint8_t>(fuelLevel, inFuelRange, outFuelRange, 3);
  
  // hi8 = high byte, lo8 = low byte
  uint8_t hi8_fuel = (fuelQuantityLiters >> 8) & 0xFF;
  uint8_t lo8_fuel = fuelQuantityLiters & 0xFF;
  
  // For F10 (not Mini): first 2 bytes get the value, last 2 bytes also get it
  unsigned char fuelWithoutCRC[] = { 
    hi8_fuel,    // Tank 1 high byte
    lo8_fuel,    // Tank 1 low byte
    hi8_fuel,    // Tank 2 high byte (same)
    lo8_fuel,    // Tank 2 low byte (same)
    0x00
  };
  CAN.sendMsgBuf(0x349, 0, 5, fuelWithoutCRC);
}

// 0x36E - ABS
void sendABS() {
  unsigned char abs1WithoutCRC[] = { 0xF0 | counter4Bit, 0xFE, 0xFF, 0x14 };
  unsigned char abs1WithCRC[] = { 
    crc8Calculator.get_crc8(abs1WithoutCRC, 4, 0xD8), 
    abs1WithoutCRC[0], abs1WithoutCRC[1], abs1WithoutCRC[2], abs1WithoutCRC[3]
  };
  CAN.sendMsgBuf(0x36E, 0, 5, abs1WithCRC);
}

// 0xD7 - Safety Counter
void sendSafetyCounter() {
  unsigned char aliveCounterSafetyWithoutCRC[] = { count, 0xFF };
  CAN.sendMsgBuf(0xD7, 0, 2, aliveCounterSafetyWithoutCRC);
}

// 0x2A7 - Power Steering
void sendPowerSteering() {
  unsigned char steeringColumnWithoutCRC[] = { 0xF0 | counter4Bit, 0xFE, 0xFF, 0x14 };
  unsigned char steeringColumnWithCRC[] = { 
    crc8Calculator.get_crc8(steeringColumnWithoutCRC, 4, 0x9E),
    steeringColumnWithoutCRC[0], steeringColumnWithoutCRC[1],
    steeringColumnWithoutCRC[2], steeringColumnWithoutCRC[3]
  };
  CAN.sendMsgBuf(0x2A7, 0, 5, steeringColumnWithCRC);
}

// 0x0AA - Rear Seatbelt Status (clears rear seatbelt warning)
void sendRearSeatbeltStatus() {
  // Try configuration 1: All zeros (buckled on some F10s)
  unsigned char rearSeatbeltMsg[] = { 
    0x00,  // Byte 0: Rear status
    0x00,  // Byte 1: Rear status
    0x00,  // Byte 2
    0x00,  // Byte 3
    0x00,  // Byte 4
    0x00,  // Byte 5
    0x00,  // Byte 6
    0x00   // Byte 7
  };
  CAN.sendMsgBuf(0x0AA, 0, 8, rearSeatbeltMsg);
}

// 0x0D0 - Alternative seatbelt status message
void sendAlternativeSeatbeltStatus() {
  // Some F10s use this message for seatbelt warnings
  unsigned char altSeatbeltMsg[] = { 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  CAN.sendMsgBuf(0x0D0, 0, 8, altSeatbeltMsg);
}

// 0x289 - Cruise Control
void sendCruiseControl() {
  // Use manual test values when cruise testing, otherwise use defaults
  unsigned char cruiseWithoutCRC[] = { 
    0xF0 | counter4Bit, 
    test289_b1,
    test289_b2,
    test289_b3,
    test289_b4,
    test289_b5,
    test289_b6
  };
  unsigned char cruiseWithCRC[] = { 
    crc8Calculator.get_crc8(cruiseWithoutCRC, 7, 0x82), 
    cruiseWithoutCRC[0], cruiseWithoutCRC[1], cruiseWithoutCRC[2],
    cruiseWithoutCRC[3], cruiseWithoutCRC[4], cruiseWithoutCRC[5], cruiseWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x289, 0, 8, cruiseWithCRC);
}

// 0x19B - Airbag
void sendAirbag() {
  unsigned char restraintWithoutCRC[] = { 0x40 | counter4Bit, 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF };
  unsigned char restraintWithCRC[] = { 
    crc8Calculator.get_crc8(restraintWithoutCRC, 7, 0xFF), 
    restraintWithoutCRC[0], restraintWithoutCRC[1], restraintWithoutCRC[2],
    restraintWithoutCRC[3], restraintWithoutCRC[4], restraintWithoutCRC[5], restraintWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x19B, 0, 8, restraintWithCRC);
}

// 0x297 - Seatbelt System (Enhanced with proper rear seatbelt status)
void sendSeatbeltSystem() {
  // Try different byte patterns for rear seatbelt status
  // 0xF0 might mean "not buckled/no occupant" on some models
  unsigned char restraint2WithoutCRC[] = { 
    0xE0 | counter4Bit, 
    0xF1,  // Driver buckled
    0xF0,  // Passenger buckled
    0xF0,  // Rear left - CHANGED to 0xF0 (no occupant/buckled)
    0xF0,  // Rear center/right - CHANGED to 0xF0
    0xFE   // Status OK
  };
  unsigned char restraint2WithCRC[] = { 
    crc8Calculator.get_crc8(restraint2WithoutCRC, 6, 0x28), 
    restraint2WithoutCRC[0], restraint2WithoutCRC[1], restraint2WithoutCRC[2],
    restraint2WithoutCRC[3], restraint2WithoutCRC[4], restraint2WithoutCRC[5]
  };
  CAN.sendMsgBuf(0x297, 0, 7, restraint2WithCRC);
}

// 0x2C4 - Engine Temperature
void sendEngineTemperature() {
  unsigned char engineTempWithoutCRC[] = { 0x3E, engineTemperature, 0x64, 0x64, 0x64, 0x01, 0xF1 };
  unsigned char engineTempWithCRC[] = { 
    crc8Calculator.get_crc8(engineTempWithoutCRC, 7, 0xB2), 
    engineTempWithoutCRC[0], engineTempWithoutCRC[1], engineTempWithoutCRC[2],
    engineTempWithoutCRC[3], engineTempWithoutCRC[4], engineTempWithoutCRC[5], engineTempWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x2C4, 0, 8, engineTempWithCRC);
}

// 0x3F9 - Oil Temperature (drives temp gauge)
void sendOilTemperature() {
  unsigned char oilWithoutCRC[] = { 
    0x10 | counter4Bit, 0x82, 0x4E, 0x7E, engineTemperature + 50, 0x05, 0x89 
  };
  unsigned char oilWithCRC[] = { 
    crc8Calculator.get_crc8(oilWithoutCRC, 7, 0xF1), 
    oilWithoutCRC[0], oilWithoutCRC[1], oilWithoutCRC[2],
    oilWithoutCRC[3], oilWithoutCRC[4], oilWithoutCRC[5], oilWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x3F9, 0, 8, oilWithCRC);
}

// 0x369 - TPMS
void sendTPMS() {
  unsigned char TPMSWithoutCRC[] = { 0xF0 | counter4Bit, 0xA2, 0xA0, 0xA0 };
  unsigned char TPMSWithCRC[] = { 
    crc8Calculator.get_crc8(TPMSWithoutCRC, 4, 0xC5), 
    TPMSWithoutCRC[0], TPMSWithoutCRC[1], TPMSWithoutCRC[2], TPMSWithoutCRC[3]
  };
  CAN.sendMsgBuf(0x369, 0, 5, TPMSWithCRC);
}

// 0x36F - Park Brake Status
void sendParkBrakeStatus() {
  unsigned char abs3WithoutCRC[] = { 0xF0 | counter4Bit, 0x38, 0, handbrake ? 0x15 : 0x14 };
  unsigned char abs3WithCRC[] = { 
    crc8Calculator.get_crc8(abs3WithoutCRC, 4, 0x17), 
    abs3WithoutCRC[0], abs3WithoutCRC[1], abs3WithoutCRC[2], abs3WithoutCRC[3]
  };
  CAN.sendMsgBuf(0x36F, 0, 5, abs3WithCRC);
}

// 0x2BB - Distance Travelled & MPG
void sendDistanceTravelled() {
  // MPG bar (first 0x2C4 message for MPG calculation)
  unsigned char mpgWithoutCRC[] = { count, 0xFF, 0x64, 0x64, 0x64, 0x01, 0xF1 };
  unsigned char mpgWithCRC[] = { 
    crc8Calculator.get_crc8(mpgWithoutCRC, 7, 0xC6), 
    mpgWithoutCRC[0], mpgWithoutCRC[1], mpgWithoutCRC[2], 
    mpgWithoutCRC[3], mpgWithoutCRC[4], mpgWithoutCRC[5], mpgWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x2C4, 0, 8, mpgWithCRC);
  
  // MPG bar 2 (this one actually moves the bar)
  unsigned char mpg2WithoutCRC[] = { 
    0xF0 | counter4Bit, 
    distanceTravelledCounter & 0xFF,
    (distanceTravelledCounter >> 8) & 0xFF,
    0xF2 
  };
  unsigned char mpg2WithCRC[] = { 
    crc8Calculator.get_crc8(mpg2WithoutCRC, 4, 0xDE), 
    mpg2WithoutCRC[0], mpg2WithoutCRC[1], mpg2WithoutCRC[2], mpg2WithoutCRC[3]
  };
  CAN.sendMsgBuf(0x2BB, 0, 5, mpg2WithCRC);
  distanceTravelledCounter += currentSpeed * 2.9;
}

// 0x2CA - Required
void send2CAMessage() {
  unsigned char msg2CA[] = { 0x64, 0x65 };
  CAN.sendMsgBuf(0x2CA, 0, 2, msg2CA);
}

// 0x393 - Required
void send393Message() {
  unsigned char msg393[] = { 0x3E, 0x32, 0x05, 0xFE };
  CAN.sendMsgBuf(0x393, 0, 4, msg393);
}

// 0x1B3 - Required
void send1B3Message() {
  unsigned char msg1B3[] = { 0x1E, 0x01, 0xF0 | counter4Bit, 0x00, 0xFF, 0x80, 0x50 };
  uint8_t crc = crc8Calculator.get_crc8(msg1B3, 7, 0xFF);
  unsigned char msg1B3WithCRC[] = { 
    crc, msg1B3[0], msg1B3[1], msg1B3[2], msg1B3[3], 
    msg1B3[4], msg1B3[5], msg1B3[6]
  };
  CAN.sendMsgBuf(0x1B3, 0, 8, msg1B3WithCRC);
}

// 0x2C5 - Required
void send2C5Message() {
  unsigned char msg2C5[] = { 0xF0 | counter4Bit, 0x00, 0xF0, 0x04, 0x70, 0x08 };
  uint8_t crc = crc8Calculator.get_crc8(msg2C5, 6, 0xD8);
  unsigned char msg2C5WithCRC[] = { 
    crc, msg2C5[0], msg2C5[1], msg2C5[2], 
    msg2C5[3], msg2C5[4], msg2C5[5]
  };
  CAN.sendMsgBuf(0x2C5, 0, 7, msg2C5WithCRC);
}

// 0x33B - ACC
void sendACC() {
  unsigned char accWithoutCRC[] = { 0xF0 | accCounter, 0x5C, 0x70, 0x00, 0x00 };
  unsigned char accWithCRC[] = { 
    crc8Calculator.get_crc8(accWithoutCRC, 5, 0x6B), 
    accWithoutCRC[0], accWithoutCRC[1], accWithoutCRC[2], 
    accWithoutCRC[3], accWithoutCRC[4]
  };
  CAN.sendMsgBuf(0x33B, 0, 6, accWithCRC);
  
  accCounter += 4;
  if (accCounter > 0x0E) accCounter = accCounter - 0x0F;
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Check for cruise control commands without colon
    if (command == "CCSET") {
      if (currentSpeed > 0) {
        cruiseControlActive = true;
        cruiseSetSpeed = currentSpeed;
        Serial.println("OK:CCSET:ACTIVE@" + String(cruiseSetSpeed) + "km/h");
      } else {
        Serial.println("ERROR:CCSET:SPEED_ZERO");
      }
      return;
    }
    else if (command == "CCRESUME") {
      if (cruiseSetSpeed > 0) {
        cruiseControlActive = true;
        Serial.println("OK:CCRESUME:ACTIVE@" + String(cruiseSetSpeed) + "km/h");
      } else {
        Serial.println("ERROR:CCRESUME:NO_SET_SPEED");
      }
      return;
    }
    else if (command == "CCCANCEL") {
      cruiseControlActive = false;
      Serial.println("OK:CCCANCEL:SPEED_KEPT@" + String(cruiseSetSpeed) + "km/h");
      return;
    }
    else if (command == "CCOFF") {
      cruiseControlActive = false;
      cruiseSetSpeed = 0;
      Serial.println("OK:CCOFF");
      return;
    }
    else if (command == "CCPLUS") {
      if (cruiseControlActive) {
        cruiseSetSpeed++;
        Serial.println("OK:CCPLUS:" + String(cruiseSetSpeed) + "km/h");
      }
      return;
    }
    else if (command == "CCMINUS") {
      if (cruiseControlActive && cruiseSetSpeed > 0) {
        cruiseSetSpeed--;
        Serial.println("OK:CCMINUS:" + String(cruiseSetSpeed) + "km/h");
      }
      return;
    }
    
    int separatorIndex = command.indexOf(':');
    if (separatorIndex > 0) {
      String cmd = command.substring(0, separatorIndex);
      String value = command.substring(separatorIndex + 1);
      
      if (cmd == "SPEED") {
        currentSpeed = constrain(value.toInt(), 0, 260);
        Serial.println("OK:SPEED:" + String(currentSpeed));
      }
      else if (cmd == "RPM") {
        currentRPM = constrain(value.toInt(), 0, 6900);
        Serial.println("OK:RPM:" + String(currentRPM));
      }
      else if (cmd == "GEAR") {
        if (value == "P") currentGear = 10;
        else if (value == "R") currentGear = 11;
        else if (value == "N") currentGear = 12;
        else if (value == "D") currentGear = 13;
        else currentGear = value.toInt();
        Serial.println("OK:GEAR:" + String(currentGear));
      }
      else if (cmd == "TEMP") {
        engineTemperature = constrain(value.toInt(), 0, 200);
        Serial.println("OK:TEMP:" + String(engineTemperature));
      }
      else if (cmd == "FUEL") {
        fuelLevel = constrain(value.toInt(), 0, 100);
        Serial.println("OK:FUEL:" + String(fuelLevel));
      }
      else if (cmd == "MODE") {
        driveMode = value.toInt();
        Serial.println("OK:MODE:" + String(driveMode));
      }
      else if (cmd == "DOOR") {
        doorOpen = (value == "1" || value == "ON");
        Serial.println("OK:DOOR:" + String(doorOpen));
      }
      else if (cmd == "DSC") {
        dscAlert = (value == "1" || value == "ON");
        Serial.println("OK:DSC:" + String(dscAlert));
      }
      else if (cmd == "HANDBRAKE") {
        handbrake = (value == "1" || value == "ON");
        Serial.println("OK:HANDBRAKE:" + String(handbrake));
      }
      else if (cmd == "CHECKENG") {
        checkEngine = (value == "1" || value == "ON");
        Serial.println("OK:CHECKENG:" + String(checkEngine));
      }
      else if (cmd == "DSCOFF") {
        dscOff = (value == "1" || value == "ON");
        Serial.println("OK:DSCOFF:" + String(dscOff));
      }
      else if (cmd == "PARKBRAKE") {
        parkBrake = (value == "1" || value == "ON");
        Serial.println("OK:PARKBRAKE:" + String(parkBrake));
      }
      else if (cmd == "HIGHBEAM") {
        highBeam = (value == "1" || value == "ON");
        Serial.println("OK:HIGHBEAM:" + String(highBeam));
      }
      else if (cmd == "FRONTFOG") {
        frontFogLight = (value == "1" || value == "ON");
        Serial.println("OK:FRONTFOG:" + String(frontFogLight));
      }
      else if (cmd == "REARFOG") {
        rearFogLight = (value == "1" || value == "ON");
        Serial.println("OK:REARFOG:" + String(rearFogLight));
      }
      else if (cmd == "LEFTBLINK") {
        leftBlinker = (value == "1" || value == "ON");
        Serial.println("OK:LEFTBLINK:" + String(leftBlinker));
      }
      else if (cmd == "RIGHTBLINK") {
        rightBlinker = (value == "1" || value == "ON");
        Serial.println("OK:RIGHTBLINK:" + String(rightBlinker));
      }
      else if (cmd == "SOSCALL") {
        sosCall = (value == "1" || value == "ON");
        Serial.println("OK:SOSCALL:" + String(sosCall));
      }
      else if (cmd == "CHASSIS") {
        chassisWarning = (value == "1" || value == "ON");
        Serial.println("OK:CHASSIS:" + String(chassisWarning));
      }
      else if (cmd == "CRUISE") {
        cruiseWarning = (value == "1" || value == "ON");
        Serial.println("OK:CRUISE:" + String(cruiseWarning));
      }
      else if (cmd == "BRAKE") {
        brakeFailure = (value == "1" || value == "ON");
        Serial.println("OK:BRAKE:" + String(brakeFailure));
      }
      else if (cmd == "DIPPED") {
        dippedBeamFailure = (value == "1" || value == "ON");
        Serial.println("OK:DIPPED:" + String(dippedBeamFailure));
      }
      else if (cmd == "TRAILER") {
        trailerReversing = (value == "1" || value == "ON");
        Serial.println("OK:TRAILER:" + String(trailerReversing));
      }
      else if (cmd == "RESTRAINT") {
        restraintRear = (value == "1" || value == "ON");
        Serial.println("OK:RESTRAINT:" + String(restraintRear));
      }
      else if (cmd == "FASTEN") {
        fastenSeatbelts = (value == "1" || value == "ON");
        Serial.println("OK:FASTEN:" + String(fastenSeatbelts));
      }
      else if (cmd == "TEST73") {
        warning73 = (value == "1" || value == "ON");
        Serial.println("OK:TEST73:" + String(warning73));
      }
      else if (cmd == "TEST85") {
        warning85 = (value == "1" || value == "ON");
        Serial.println("OK:TEST85:" + String(warning85));
      }
      else if (cmd == "TEST88") {
        warning88 = (value == "1" || value == "ON");
        Serial.println("OK:TEST88:" + String(warning88));
      }
      else if (cmd == "STATUS") {
        Serial.println("STATUS:");
        Serial.println("SPEED:" + String(currentSpeed));
        Serial.println("RPM:" + String(currentRPM));
        Serial.println("GEAR:" + String(currentGear));
        Serial.println("TEMP:" + String(engineTemperature));
        Serial.println("FUEL:" + String(fuelLevel));
        Serial.println("MODE:" + String(driveMode));
        Serial.println("CRUISE_ACTIVE:" + String(cruiseControlActive));
        Serial.println("CRUISE_SET:" + String(cruiseSetSpeed));
        Serial.println("END");
      }
      else {
        Serial.println("ERROR:UNKNOWN_CMD:" + cmd);
      }
    } else {
      Serial.println("ERROR:INVALID_FORMAT");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("BMW F10 Diesel CAN Controller v3.1 - Rear Seatbelt Fix");
  Serial.println("ESP32-S3 + MCP2515 8MHz");
  
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  
  if (CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN: 500kbps OK!");
  } else if (CAN.begin(MCP_STDEXT, CAN_100KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN: 100kbps OK!");
  } else {
    Serial.println("CAN: FAILED!");
    while(1) delay(1000);
  }
  
  CAN.setMode(MCP_NORMAL);
  
  // Wake-up sequence
  for(int i = 0; i < 10; i++) {
    sendIgnitionStatus();
    sendBacklight();
    sendSafetyCounter();
    sendLights();
    delay(20);
  }
  
  Serial.println("READY!");
  Serial.println("Rear seatbelt warning disabled");
}

void loop() {
  processSerialCommand();
  
  send2C5Message();        // Required system
  send1B3Message();        // Required system  
  send2CAMessage();        // Required system
  send393Message();        // Required system
  sendACC();               // ACC system
  sendIgnitionStatus();    // Ignition ON
  sendBacklight();         // Backlight keepalive
  sendSpeed();             // Speed
  sendRPM();               // RPM
  sendTransmission();      // Gear
  sendAlerts();            // Warnings (all OFF)
  sendLights();            // Lights
  sendBlinkers();          // Turn signals
  sendABS();               // ABS system OK
  sendSafetyCounter();     // Safety keepalive
  sendPowerSteering();     // Steering OK
  sendCruiseControl();     // Cruise control OK
  sendAirbag();            // Airbag OK
  sendSeatbeltSystem();    // Seatbelt system OK (0x297 - all buckled)
  sendTPMS();              // TPMS OK
  sendEngineTemperature(); // Engine temp
  sendOilTemperature();    // Oil temp gauge
  sendParkBrakeStatus();   // Park brake status OK
  sendDriveMode();         // Drive mode
  sendFuel();              // Fuel level
  sendDistanceTravelled(); // Odometer
  
  counter4Bit++;
  if (counter4Bit >= 14) counter4Bit = 0;
  
  count++;
  if (count >= 254) count = 0;
  
  delay(100);
}
