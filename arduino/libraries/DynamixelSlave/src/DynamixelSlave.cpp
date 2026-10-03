#include "DynamixelSlave.h"

// ---- Compile-time tunables ----
// Computed for F_CPU = 20 MHz, 57600 baud (4 * 20e6 / 57600 = 1389).
// If you need a different baud, edit BAUD_REG and rebuild.
static constexpr uint16_t BAUD_REG        = 1389;
static constexpr uint16_t RETURN_DELAY_US = 50;
static constexpr uint16_t SYNC_SLOT_US    = 5000;

static uint16_t crc_update(uint16_t crc, uint8_t b) {
  crc ^= ((uint16_t)b) << 8;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x8000) ? ((crc << 1) ^ 0x8005) : (crc << 1);
  }
  return crc;
}

// Note: this slave does not de-stuff incoming 0xFF 0xFF 0xFD 0xFD sequences.
// The control table values typically exposed (small register addresses, low
// bytes of ADC results, IDs) cannot trigger byte stuffing, so we save flash
// by skipping that step. If you start exposing items whose serialized bytes
// can contain 0xFF 0xFF 0xFD, restore the unstuff helper from git history.

DynamixelSlave::DynamixelSlave(uint8_t id, uint16_t model_number, uint8_t firmware)
  : _id(id), _model(model_number), _firmware(firmware),
    _dirMask(0),
    _state(S_H1), _bodyLen(0), _bodyIdx(0), _packetId(0),
    _itemCount(0), _onWrite(nullptr) {}

void DynamixelSlave::setDirectionPin(uint8_t pin) {
  _dirMask = (pin == 255) ? 0 : digitalPinToBitMask(pin);
}

void DynamixelSlave::onWrite(void (*cb)(uint16_t, uint8_t)) { _onWrite = cb; }

void DynamixelSlave::begin(bool alt_pins) {
  // USART0 default routes to PA6 (TXD) / PA7 (RXD) on tinyAVR-1 8-pin parts;
  // PORTMUX moves it to the alternate position PA1 (TXD) / PA2 (RXD).
  if (alt_pins) {
    PORTMUX.CTRLB |= PORTMUX_USART0_bm;
    PORTA.DIRSET = PIN1_bm;
    PORTA.DIRCLR = PIN2_bm;
  } else {
    PORTMUX.CTRLB &= ~PORTMUX_USART0_bm;
    PORTA.DIRSET = PIN6_bm;
    PORTA.DIRCLR = PIN7_bm;
  }
  // MAX485 RO is push-pull, no internal pullup needed on RX.

  USART0.BAUD = BAUD_REG;
  USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

  PORTA.DIRSET = _dirMask;       // PORTA.DIRSET = 0 is a no-op when none set
  PORTA.OUTCLR = _dirMask;

  _state = S_H1;
  _bodyIdx = 0;
}

bool DynamixelSlave::addItem(uint16_t addr, void* data, uint8_t size, bool readonly) {
  if (_itemCount >= MAX_ITEMS) return false;
  if (size != 1 && size != 2 && size != 4) return false;
  _items[_itemCount].addr = addr;
  _items[_itemCount].data = data;
  _items[_itemCount].size = size;
  _items[_itemCount].readonly = readonly ? 1 : 0;
  _itemCount++;
  return true;
}

void DynamixelSlave::process() {
  while (USART0.STATUS & USART_RXCIF_bm) {
    uint8_t b = USART0.RXDATAL;

    switch (_state) {
      case S_H1:
        if (b == 0xFF) _state = S_H2;
        break;
      case S_H2:
        _state = (b == 0xFF) ? S_H3 : S_H1;
        break;
      case S_H3:
        if (b == 0xFD)      _state = S_RES;
        else if (b == 0xFF) ; // stay
        else                _state = S_H1;
        break;
      case S_RES:
        _state = (b == 0x00) ? S_ID : S_H1;
        break;
      case S_ID:
        _packetId = b;
        _state = S_LEN_L;
        break;
      case S_LEN_L:
        _bodyLen = b;
        _state = S_LEN_H;
        break;
      case S_LEN_H:
        _bodyLen |= (uint16_t)b << 8;
        if (_bodyLen < 3 || _bodyLen > BUF_SIZE) {
          _state = S_H1;
        } else {
          _bodyIdx = 0;
          _state = S_BODY;
        }
        break;
      case S_BODY:
        _buf[_bodyIdx++] = b;
        if (_bodyIdx >= _bodyLen) {
          onPacket();
          _state = S_H1;
        }
        break;
    }
  }
}

void DynamixelSlave::onPacket() {
  // Reject anything that's neither for us nor a broadcast. Staying silent
  // for unrelated traffic is what lets us coexist with real Dynamixel servos.
  if (_packetId != _id && _packetId != BROADCAST_ID) return;

  uint16_t crc = 0;
  crc = crc_update(crc, 0xFF);
  crc = crc_update(crc, 0xFF);
  crc = crc_update(crc, 0xFD);
  crc = crc_update(crc, 0x00);
  crc = crc_update(crc, _packetId);
  crc = crc_update(crc, _bodyLen & 0xFF);
  crc = crc_update(crc, _bodyLen >> 8);
  for (uint16_t i = 0; i < _bodyLen - 2; i++) crc = crc_update(crc, _buf[i]);

  uint16_t rx_crc = _buf[_bodyLen - 2] | ((uint16_t)_buf[_bodyLen - 1] << 8);
  if (crc != rx_crc) return;

  uint16_t body_len = _bodyLen - 2;   // inst + params (CRC already stripped)
  if (body_len < 1) return;

  uint8_t  inst       = _buf[0];
  uint8_t* params     = &_buf[1];
  uint16_t params_len = body_len - 1;

  // Broadcasts: only SYNC_READ / SYNC_WRITE are honored.
  if (_packetId == BROADCAST_ID) {
    if (inst == 0x82 && params_len >= 4) {
      uint16_t addr = params[0] | ((uint16_t)params[1] << 8);
      uint16_t len  = params[2] | ((uint16_t)params[3] << 8);
      handleSyncRead(addr, len, &params[4], params_len - 4);
    } else if (inst == 0x83 && params_len >= 4) {
      uint16_t addr     = params[0] | ((uint16_t)params[1] << 8);
      uint16_t data_len = params[2] | ((uint16_t)params[3] << 8);
      handleSyncWrite(addr, data_len, &params[4], params_len - 4);
    }
    return;
  }

  switch (inst) {
    case 0x01: // PING
      replyPing();
      break;
    case 0x02: // READ
      if (params_len >= 4) {
        uint16_t addr = params[0] | ((uint16_t)params[1] << 8);
        uint16_t len  = params[2] | ((uint16_t)params[3] << 8);
        replyRead(addr, len);
      }
      break;
    case 0x03: // WRITE
      if (params_len >= 2) {
        uint16_t addr = params[0] | ((uint16_t)params[1] << 8);
        replyWrite(addr, params_len - 2, &params[2]);
      }
      break;
    // Unknown instructions: silently drop. Skips ERR_INSTRUCTION reply to save flash.
  }
}

void DynamixelSlave::handleSyncRead(uint16_t addr, uint16_t len,
                                    const uint8_t* ids, uint16_t id_count) {
  uint8_t position = 0xFF;
  for (uint16_t i = 0; i < id_count; i++) {
    if (ids[i] == _id) { position = (uint8_t)i; break; }
  }
  if (position == 0xFF) return;                  // our ID is not in the list

  // Wait for our slot so we don't collide with earlier responders.
  // delayMicroseconds() takes up to ~16383 us; SYNC_SLOT_US must stay below.
  for (uint8_t i = 0; i < position; i++) delayMicroseconds(SYNC_SLOT_US);

  Item* it = findItem(addr, (uint8_t)len);
  if (!it) { sendStatus(ERR_ACCESS, nullptr, 0); return; }
  sendStatus(ERR_NONE, (const uint8_t*)it->data, (uint8_t)len);
}

void DynamixelSlave::handleSyncWrite(uint16_t addr, uint16_t data_len,
                                     const uint8_t* entries, uint16_t entries_len) {
  // entries = [id, data...][id, data...][...]   each entry is (1 + data_len) bytes
  uint16_t step = (uint16_t)1 + data_len;
  uint16_t off  = 0;
  while (off + step <= entries_len) {
    if (entries[off] == _id) {
      uint8_t err = writeItem(addr, data_len, &entries[off + 1]);
      if (err == ERR_NONE && _onWrite) _onWrite(addr, (uint8_t)data_len);
      return;                                    // SYNC_WRITE never replies
    }
    off += step;
  }
}

void DynamixelSlave::replyPing() {
  uint8_t buf[3];
  buf[0] = _model & 0xFF;
  buf[1] = _model >> 8;
  buf[2] = _firmware;
  sendStatus(ERR_NONE, buf, 3);
}

void DynamixelSlave::replyRead(uint16_t addr, uint16_t len) {
  // findItem only matches sizes 1/2/4 registered via addItem, so an oversize
  // request naturally falls through to ERR_ACCESS.
  Item* it = findItem(addr, (uint8_t)len);
  if (!it) { sendStatus(ERR_ACCESS, nullptr, 0); return; }
  sendStatus(ERR_NONE, (const uint8_t*)it->data, (uint8_t)len);
}

void DynamixelSlave::replyWrite(uint16_t addr, uint16_t len, const uint8_t* data) {
  uint8_t err = writeItem(addr, len, data);
  sendStatus(err, nullptr, 0);
  if (err == ERR_NONE && _onWrite) _onWrite(addr, (uint8_t)len);
}

DynamixelSlave::Item* DynamixelSlave::findItem(uint16_t addr, uint8_t size) {
  for (uint8_t i = 0; i < _itemCount; i++) {
    if (_items[i].addr == addr && _items[i].size == size) return &_items[i];
  }
  return nullptr;
}

uint8_t DynamixelSlave::writeItem(uint16_t addr, uint16_t len, const uint8_t* in) {
  Item* it = findItem(addr, (uint8_t)len);
  if (!it)           return ERR_ACCESS;
  if (it->readonly)  return ERR_ACCESS;
  // Caller (process loop) is not preempted by anything that touches item data,
  // so no SREG guard needed.
  memcpy(it->data, in, len);
  return ERR_NONE;
}

void DynamixelSlave::txEnable() {
  USART0.STATUS  = USART_TXCIF_bm;            // clear so we can see end of THIS frame
  USART0.CTRLB  &= ~USART_RXEN_bm;            // mute RX while we drive the bus
  PORTA.OUTSET   = _dirMask;                  // safe even when _dirMask == 0
}

void DynamixelSlave::txDisable() {
  PORTA.OUTCLR  = _dirMask;
  while (USART0.STATUS & USART_RXCIF_bm) (void)USART0.RXDATAL;
  USART0.CTRLB |= USART_RXEN_bm;
  _state = S_H1;
  _bodyIdx = 0;
}

void DynamixelSlave::txByte(uint8_t b) {
  while (!(USART0.STATUS & USART_DREIF_bm));
  USART0.TXDATAL = b;
}

// Note: TX-side byte stuffing is intentionally omitted. The FF FF FD pattern
// cannot appear in the Status header / error / typical small register values
// this library exposes, so we save ~50 bytes of flash and 24 bytes of stack
// by streaming bytes directly. If you start exposing items whose bytes can
// contain that pattern, add stuffing here.
void DynamixelSlave::sendStatus(uint8_t err, const uint8_t* params, uint8_t param_len) {
  uint16_t pkt_len = (uint16_t)param_len + 4;   // 0x55 + err + 2 CRC bytes

  delayMicroseconds(RETURN_DELAY_US);
  txEnable();

  uint16_t crc = 0;
  txByte(0xFF); crc = crc_update(crc, 0xFF);
  txByte(0xFF); crc = crc_update(crc, 0xFF);
  txByte(0xFD); crc = crc_update(crc, 0xFD);
  txByte(0x00); crc = crc_update(crc, 0x00);
  txByte(_id); crc = crc_update(crc, _id);
  uint8_t lo = pkt_len & 0xFF, hi = pkt_len >> 8;
  txByte(lo); crc = crc_update(crc, lo);
  txByte(hi); crc = crc_update(crc, hi);
  txByte(0x55); crc = crc_update(crc, 0x55);
  txByte(err);  crc = crc_update(crc, err);
  for (uint8_t i = 0; i < param_len; i++) {
    txByte(params[i]);
    crc = crc_update(crc, params[i]);
  }
  txByte(crc & 0xFF);
  txByte(crc >> 8);

  while (!(USART0.STATUS & USART_TXCIF_bm));
  USART0.STATUS = USART_TXCIF_bm;
  txDisable();
}
