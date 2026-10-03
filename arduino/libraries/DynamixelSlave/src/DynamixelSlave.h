#pragma once
#include <Arduino.h>

class DynamixelSlave {
public:
  enum Error : uint8_t {
    ERR_NONE        = 0x00,
    ERR_RESULT      = 0x01,
    ERR_INSTRUCTION = 0x02,
    ERR_CRC         = 0x03,
    ERR_RANGE       = 0x04,
    ERR_LENGTH      = 0x05,
    ERR_LIMIT       = 0x06,
    ERR_ACCESS      = 0x07,
  };

  static constexpr uint8_t BROADCAST_ID = 0xFE;
  static constexpr uint8_t MAX_ITEMS    = 4;
  static constexpr uint8_t BUF_SIZE     = 32;

  DynamixelSlave(uint8_t id, uint16_t model_number = 0xBEEF, uint8_t firmware = 0x01);

  // Pin used to enable the RS-485 driver (DE+RE tied together, active HIGH).
  // Pass 255 to disable direction control (TTL half-duplex setups).
  void setDirectionPin(uint8_t pin);

  // Configure USART0. alt_pins=true routes the UART to PA1 (TX) / PA2 (RX)
  // via PORTMUX; alt_pins=false keeps the default pins PA6 (TX) / PA7 (RX).
  // Baud is hardcoded to 57600 to save flash; recompile with a different
  // BAUD_REG below if you need another rate. Return-delay-time and SYNC_READ
  // slot timing are also compile-time constants (see RETURN_DELAY_US and
  // SYNC_SLOT_US in the .cpp).
  void begin(bool alt_pins = true);

  // Register a control table item. data must point to RAM that stays alive
  // for the life of the slave. size must be 1, 2, or 4.
  bool addItem(uint16_t addr, void* data, uint8_t size, bool readonly = false);

  // Optional write callback: invoked after a successful WRITE matching an
  // existing item. addr/size identify which item changed.
  void onWrite(void (*cb)(uint16_t addr, uint8_t size));

  // Pump the receiver. Call from loop().
  void process();

  uint8_t id() const { return _id; }
  void    setId(uint8_t id) { _id = id; }

private:
  enum State : uint8_t {
    S_H1, S_H2, S_H3, S_RES, S_ID, S_LEN_L, S_LEN_H, S_BODY,
  };

  struct Item {
    uint16_t addr;
    void*    data;
    uint8_t  size;
    uint8_t  readonly;
  };

  uint8_t  _id;
  uint16_t _model;
  uint8_t  _firmware;
  uint8_t  _dirMask;      // PORTA bit mask for DE/RE pin (0 = none)

  State    _state;
  uint16_t _bodyLen;
  uint16_t _bodyIdx;
  uint8_t  _packetId;
  uint8_t  _buf[BUF_SIZE];

  Item     _items[MAX_ITEMS];
  uint8_t  _itemCount;

  void (*_onWrite)(uint16_t, uint8_t);

  void onPacket();
  void replyPing();
  void replyRead(uint16_t addr, uint16_t len);
  void replyWrite(uint16_t addr, uint16_t len, const uint8_t* data);
  void handleSyncRead(uint16_t addr, uint16_t len, const uint8_t* ids, uint16_t id_count);
  void handleSyncWrite(uint16_t addr, uint16_t data_len, const uint8_t* entries, uint16_t entries_len);
  void sendStatus(uint8_t err, const uint8_t* params, uint8_t param_len);

  void txEnable();
  void txDisable();
  void txByte(uint8_t b);

  Item*   findItem(uint16_t addr, uint8_t size);
  uint8_t writeItem(uint16_t addr, uint16_t len, const uint8_t* in);
};
