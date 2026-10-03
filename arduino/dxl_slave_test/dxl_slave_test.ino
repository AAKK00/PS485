// dxl_slave_test.ino
// Arduino Uno as a minimal Dynamixel Protocol 2.0 master to verify
// the DynamixelSlave library running on an ATtiny202.
//
// Wiring (Uno side):
//   D11 -> MAX485 DI    (TX)
//   D12 <- MAX485 RO    (RX)
//   D13 -> MAX485 DE+RE (tied, active HIGH = transmit)
//   A/B -> ATtiny202 side MAX485 A/B (same twisted pair)
//
// Open Serial Monitor at 115200 baud after flashing.

#include <SoftwareSerial.h>

constexpr uint8_t  PIN_DE_RE  = 13;
constexpr uint8_t  PIN_RX     = 12;
constexpr uint8_t  PIN_TX     = 11;
constexpr uint8_t  SLAVE_ID   = 100;
constexpr uint32_t DXL_BAUD   = 57600;

SoftwareSerial rs485(PIN_RX, PIN_TX);

static uint16_t crc_update(uint16_t crc, uint8_t b) {
  crc ^= ((uint16_t)b) << 8;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x8000) ? ((crc << 1) ^ 0x8005) : (crc << 1);
  }
  return crc;
}

static void txEnable()  { digitalWrite(PIN_DE_RE, HIGH); delayMicroseconds(20); }
static void txDisable() { delayMicroseconds(20); digitalWrite(PIN_DE_RE, LOW); }

static void printHex(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

// Send a packet (header through body, ex-CRC). Appends CRC and transmits.
static void sendPacket(uint8_t* buf, uint8_t len) {
  uint16_t crc = 0;
  for (uint8_t i = 0; i < len; i++) crc = crc_update(crc, buf[i]);
  buf[len++] = crc & 0xFF;
  buf[len++] = crc >> 8;

  Serial.print(F("  TX: "));
  for (uint8_t i = 0; i < len; i++) { printHex(buf[i]); Serial.write(' '); }
  Serial.println();

  txEnable();
  for (uint8_t i = 0; i < len; i++) rs485.write(buf[i]);
  txDisable();
  while (rs485.available()) rs485.read();  // drop junk seen during our own TX
}

static void sendPing(uint8_t id) {
  uint8_t buf[12] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x03, 0x00, 0x01};
  sendPacket(buf, 8);
}

static void sendRead(uint8_t id, uint16_t addr, uint16_t len) {
  uint8_t buf[16] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x07, 0x00, 0x02,
                     (uint8_t)addr, (uint8_t)(addr >> 8),
                     (uint8_t)len,  (uint8_t)(len  >> 8)};
  sendPacket(buf, 12);
}

static void sendWrite1(uint8_t id, uint16_t addr, uint8_t val) {
  uint8_t buf[14] = {0xFF, 0xFF, 0xFD, 0x00, id, 0x06, 0x00, 0x03,
                     (uint8_t)addr, (uint8_t)(addr >> 8), val};
  sendPacket(buf, 11);
}

static void sendSyncRead(uint16_t addr, uint16_t data_len, const uint8_t* ids, uint8_t id_count) {
  uint8_t buf[24];
  uint16_t len_field = (uint16_t)id_count + 7;       // inst + 4 fields + N + 2 CRC
  uint8_t p = 0;
  buf[p++] = 0xFF; buf[p++] = 0xFF; buf[p++] = 0xFD; buf[p++] = 0x00;
  buf[p++] = 0xFE;
  buf[p++] = (uint8_t)len_field; buf[p++] = (uint8_t)(len_field >> 8);
  buf[p++] = 0x82;
  buf[p++] = (uint8_t)addr;      buf[p++] = (uint8_t)(addr >> 8);
  buf[p++] = (uint8_t)data_len;  buf[p++] = (uint8_t)(data_len >> 8);
  for (uint8_t i = 0; i < id_count; i++) buf[p++] = ids[i];
  sendPacket(buf, p);
}

static void sendSyncWrite(uint16_t addr, uint16_t data_len, const uint8_t* entries, uint8_t entry_count) {
  uint8_t buf[40];
  uint16_t entries_size = (uint16_t)entry_count * (1 + data_len);
  uint16_t len_field = 7 + entries_size;             // inst + 4 fields + entries + 2 CRC
  uint8_t p = 0;
  buf[p++] = 0xFF; buf[p++] = 0xFF; buf[p++] = 0xFD; buf[p++] = 0x00;
  buf[p++] = 0xFE;
  buf[p++] = (uint8_t)len_field; buf[p++] = (uint8_t)(len_field >> 8);
  buf[p++] = 0x83;
  buf[p++] = (uint8_t)addr;      buf[p++] = (uint8_t)(addr >> 8);
  buf[p++] = (uint8_t)data_len;  buf[p++] = (uint8_t)(data_len >> 8);
  for (uint16_t i = 0; i < entries_size; i++) buf[p++] = entries[i];
  sendPacket(buf, p);
}

// Collect every byte that arrives during the listen window.
// No early-exit: we want to see everything, including garbled bytes.
static uint8_t receivePacket(uint8_t* out, uint8_t maxlen, uint16_t total_timeout_ms) {
  uint8_t  n        = 0;
  uint32_t deadline = millis() + total_timeout_ms;
  while (millis() < deadline && n < maxlen) {
    if (rs485.available()) out[n++] = rs485.read();
  }
  return n;
}

static void dump(const uint8_t* buf, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) { printHex(buf[i]); Serial.write(' '); }
  Serial.println();
}

static void describe(const uint8_t* buf, uint8_t n) {
  if (n == 0) { Serial.println(F("  (no response)")); return; }
  if (n < 11 || buf[0] != 0xFF || buf[1] != 0xFF || buf[2] != 0xFD || buf[3] != 0x00) {
    Serial.println(F("  (malformed)"));
    return;
  }
  Serial.print(F("  ID="));     Serial.print(buf[4]);
  uint16_t len = buf[5] | ((uint16_t)buf[6] << 8);
  Serial.print(F(" len="));     Serial.print(len);
  Serial.print(F(" inst=0x"));  printHex(buf[7]);
  Serial.print(F(" err=0x"));   printHex(buf[8]);
  if (n > 11) {
    Serial.print(F(" params="));
    for (uint8_t i = 9; i < n - 2; i++) { printHex(buf[i]); Serial.write(' '); }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);
  rs485.begin(DXL_BAUD);
  delay(800);  // give the ATtiny202 time to finish its boot blink

  Serial.println();
  Serial.println(F("==== Dynamixel slave tester ===="));
  Serial.print(F("target ID = ")); Serial.println(SLAVE_ID);
  Serial.print(F("baud      = ")); Serial.println(DXL_BAUD);
}

uint8_t led_val = 1;

void loop() {
  uint8_t rx[40];
  uint8_t n;

  Serial.println();
  Serial.println(F("[PING]"));
  sendPing(SLAVE_ID);
  n = receivePacket(rx, sizeof(rx), 500);
  Serial.print(F("  raw: ")); dump(rx, n);
  describe(rx, n);
  if (n >= 14) {
    uint16_t model = rx[9] | ((uint16_t)rx[10] << 8);
    Serial.print(F("  -> model=0x")); Serial.print(model, HEX);
    Serial.print(F(" fw=")); Serial.println(rx[11]);
  }

  delay(150);

  Serial.println(F("[READ Present Position addr=132 len=2]"));
  sendRead(SLAVE_ID, 132, 2);
  n = receivePacket(rx, sizeof(rx), 500);
  Serial.print(F("  raw: ")); dump(rx, n);
  describe(rx, n);
  if (n >= 13) {
    uint16_t pos = rx[9] | ((uint16_t)rx[10] << 8);
    Serial.print(F("  -> pos=")); Serial.println(pos);
  }

  delay(150);

  Serial.print(F("[WRITE LED addr=65 val=")); Serial.print(led_val); Serial.println(F("]"));
  sendWrite1(SLAVE_ID, 65, led_val);
  n = receivePacket(rx, sizeof(rx), 500);
  Serial.print(F("  raw: ")); dump(rx, n);
  describe(rx, n);
  led_val ^= 1;

  delay(150);

  Serial.println(F("[SYNC_READ pos addr=132 len=2 ids=100]"));
  const uint8_t sr_ids[] = {SLAVE_ID};
  sendSyncRead(132, 2, sr_ids, 1);
  n = receivePacket(rx, sizeof(rx), 500);
  Serial.print(F("  raw: ")); dump(rx, n);
  describe(rx, n);
  if (n >= 13) {
    uint16_t pos = rx[9] | ((uint16_t)rx[10] << 8);
    Serial.print(F("  -> pos=")); Serial.println(pos);
  }

  delay(150);

  Serial.print(F("[SYNC_WRITE LED addr=65 ids=100 data=")); Serial.print(led_val); Serial.println(F("] (no reply)"));
  const uint8_t sw_entries[] = {SLAVE_ID, led_val};
  sendSyncWrite(65, 1, sw_entries, 1);
  delay(50);

  delay(1500);
}
