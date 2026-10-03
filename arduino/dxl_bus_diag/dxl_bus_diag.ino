// dxl_bus_diag.ino  (v2: RX on hardware UART)
//
// Raw RS-485 diagnostic. TX goes out via SoftwareSerial on pin 11,
// RX comes back on hardware Serial (pin 0), which can listen full-duplex
// even while we transmit -- so with the MAX485's RE tied to GND we can
// see our own bytes echoed on the bus, plus any slave reply.
//
// Wiring (Uno -> master MAX485):
//   pin 11  -> DI
//   pin  8  -> DE          (RE separated from DE, tied to GND)
//   pin  0  <- RO          *** connect AFTER upload, remove BEFORE upload ***
//   5V/GND, A/B to bus as usual.
//
// USB monitor at 57600 (shared with the RX uart).
//
// Reading the dump:
//   FF FF FD 00 64 03 00 01 xx xx           <- our own ping echo (id 0x64=100)
//   FF FF FD 00 64 07 00 55 00 ... crc crc  <- slave reply, if any
//   echo visible + no reply  -> master side OK, slave side dead
//   nothing at all           -> master MAX485 / DE wiring / RO wiring bad

#include <SoftwareSerial.h>

constexpr uint8_t PIN_TX_DUMMY_RX = 12;  // unused, SoftwareSerial needs an RX pin
constexpr uint8_t PIN_TX  = 11;
constexpr uint8_t PIN_DIR = 8;           // drives DE only

SoftwareSerial busTx(PIN_TX_DUMMY_RX, PIN_TX);

uint16_t crc_update(uint16_t crc, uint8_t b) {
  crc ^= ((uint16_t)b) << 8;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x8000) ? ((crc << 1) ^ 0x8005) : (crc << 1);
  return crc;
}

void sendPing(uint8_t id) {
  uint8_t pkt[10] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x03, 0x00, 0x01, 0, 0};
  uint16_t crc = 0;
  for (uint8_t i = 0; i < 8; i++) crc = crc_update(crc, pkt[i]);
  pkt[8] = crc & 0xFF;
  pkt[9] = crc >> 8;

  while (Serial.available()) Serial.read();   // drop stale bytes
  digitalWrite(PIN_DIR, HIGH);
  delayMicroseconds(10);
  busTx.write(pkt, sizeof(pkt));
  digitalWrite(PIN_DIR, LOW);
}

void dumpReplies(uint16_t window_ms) {
  uint32_t t0 = millis();
  uint8_t n = 0;
  while (millis() - t0 < window_ms) {
    if (Serial.available()) {
      uint8_t b = Serial.read();
      if (b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
      Serial.print(' ');
      n++;
    }
  }
  if (n == 0) Serial.print(F("(no bytes)"));
  Serial.println();
}

void setup() {
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);
  Serial.begin(57600);     // both USB debug out and RO listen in
  busTx.begin(57600);
  Serial.println(F("=== dxl_bus_diag v2 ==="));
  Serial.println(F("own ping echo = FF FF FD 00 <id> 03 00 01 cc cc"));
}

void loop() {
  Serial.print(F("ping 100 -> "));
  sendPing(100);
  dumpReplies(100);

  delay(200);

  Serial.print(F("ping 101 -> "));
  sendPing(101);
  dumpReplies(100);

  Serial.println();
  delay(1000);
}
