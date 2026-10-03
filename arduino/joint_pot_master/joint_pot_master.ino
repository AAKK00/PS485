// joint_pot_master.ino
//
// Arduino Uno as Dynamixel Protocol 2.0 master over MAX485,
// talking to the ATtiny202 JointPotentiometer slaves at ID 100 / 101.
//
// Wiring (Uno -> MAX485 module):
//   pin 10 (SS RX)  <- RO
//   pin 11 (SS TX)  -> DI
//   pin  8 (DIR)    -> DE + RE (tied)
//   5V/GND to VCC/GND, A/B to the RS-485 bus.
//
// USB Serial (pin 0/1) is used for debug output ONLY.
// The RS-485 line is fully on SoftwareSerial, so USB and upload are unaffected.
//
// Baud: 57600 (fixed on the slave side).

#include <SoftwareSerial.h>
#include <Dynamixel2Arduino.h>
#include "utility/port_handler.h"

constexpr uint8_t PIN_RS485_RX = 10;
constexpr uint8_t PIN_RS485_TX = 11;
constexpr int     DXL_DIR_PIN  = 8;

SoftwareSerial dxlSerial(PIN_RS485_RX, PIN_RS485_TX);

// Custom port handler that wraps SoftwareSerial + a DIR pin, so that
// Dynamixel2Arduino (which normally only accepts HardwareSerial) can drive
// a bit-banged UART.
class SoftSerialPortHandler : public DXLPortHandler {
 public:
  SoftSerialPortHandler(SoftwareSerial& port, int dir_pin)
      : port_(port), dir_pin_(dir_pin) {}

  void begin() override { beginWithBaud(57600); }
  void beginWithBaud(unsigned long baud) {
    port_.begin(baud);
    if (dir_pin_ != -1) {
      pinMode(dir_pin_, OUTPUT);
      digitalWrite(dir_pin_, LOW);
    }
    setOpenState(true);
  }
  void end() override {
    port_.end();
    setOpenState(false);
  }
  int available() override { return port_.available(); }
  int read() override { return port_.read(); }
  size_t write(uint8_t c) override {
    driveHigh();
    size_t ret = port_.write(c);
    releaseLow();
    return ret;
  }
  size_t write(uint8_t* buf, size_t len) override {
    driveHigh();
    size_t ret = port_.write(buf, len);
    releaseLow();
    return ret;
  }

 private:
  void driveHigh() {
    if (dir_pin_ != -1) digitalWrite(dir_pin_, HIGH);
  }
  void releaseLow() {
    // SoftwareSerial::write is fully blocking / bit-banged, so no flush() needed;
    // by the time we reach here the last stop bit has already been shifted out.
    if (dir_pin_ != -1) digitalWrite(dir_pin_, LOW);
  }

  SoftwareSerial& port_;
  int dir_pin_;
};

SoftSerialPortHandler dxlPort(dxlSerial, DXL_DIR_PIN);
Dynamixel2Arduino dxl;

constexpr uint8_t IDS[] = {100, 101};
constexpr uint8_t N_IDS = sizeof(IDS) / sizeof(IDS[0]);

constexpr uint16_t ADDR_LED              = 65;   // 1 byte, RW
constexpr uint16_t ADDR_PRESENT_POSITION = 132;  // 2 bytes, RO

uint8_t ledPhase = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  while (!Serial) {}
  Serial.println();
  Serial.println(F("=== joint_pot_master (SoftwareSerial) ==="));

  dxl.setPort(dxlPort);
  dxlPort.beginWithBaud(57600);
  dxl.setPortProtocolVersion(2.0);

  delay(200);

  for (uint8_t i = 0; i < N_IDS; i++) {
    Serial.print(F("ping ID "));
    Serial.print(IDS[i]);
    Serial.print(F(": "));
    // Our custom slave's model number (0x4B41) is not in Dynamixel2Arduino's
    // model table, so ping() reports UNKNOWN_MODEL_NUMBER even though the
    // slave answered. Treat that as success.
    if (dxl.ping(IDS[i]) ||
        dxl.getLastLibErrCode() == D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER) {
      Serial.println(F("OK"));
    } else {
      Serial.print(F("FAIL err=0x"));
      Serial.println(dxl.getLastLibErrCode(), HEX);
    }
  }
}

void loop() {
  ledPhase ^= 1;
  digitalWrite(LED_BUILTIN, ledPhase);

  for (uint8_t i = 0; i < N_IDS; i++) {
    dxl.write(IDS[i], ADDR_LED, &ledPhase, 1, 20);
  }

  for (uint8_t i = 0; i < N_IDS; i++) {
    uint16_t pos = 0;
    bool ok = dxl.read(IDS[i], ADDR_PRESENT_POSITION,
                       2, (uint8_t*)&pos, sizeof(pos), 30);
    Serial.print(F("ID "));
    Serial.print(IDS[i]);
    Serial.print(F(": "));
    if (ok) {
      Serial.print(F("pos="));
      Serial.println(pos);
    } else {
      Serial.print(F("read FAIL err=0x"));
      Serial.println(dxl.getLastLibErrCode(), HEX);
    }
  }

  Serial.println();
  delay(500);
}
