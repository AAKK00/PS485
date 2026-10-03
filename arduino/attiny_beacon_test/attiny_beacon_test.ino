// attiny_beacon_test.ino  (v2: RX counter beacon)
//
// Slave-side self test for the elephant_robot_position_sensor board.
// Counts every byte received on USART0 RX (PA7 <- MAX485 RO) and, every
// 100 ms, drives the bus and transmits:  AA 55 <rx_byte_count>
//
// The master pings put 10 bytes each on the bus. So while the master diag
// is running (2 pings per ~1.2 s cycle):
//   count increases ~20 per cycle -> slave RX path works end to end
//   count stays constant          -> RO -> PA7 path (or RX config) is dead
//
// Tools: Chip = ATtiny202, Clock = 20 MHz internal.

static constexpr uint16_t BAUD_REG = 1389;   // 4 * 20 MHz / 57600

static void txByte(uint8_t b) {
  while (!(USART0.STATUS & USART_DREIF_bm));
  USART0.TXDATAL = b;
}

void setup() {
  PORTMUX.CTRLB &= ~PORTMUX_USART0_bm;   // USART0 on default pins PA6/PA7
  PORTA.DIRSET = PIN6_bm | PIN3_bm;      // PA6 = TXD out, PA3 = ena out
  PORTA.DIRCLR = PIN7_bm;                // PA7 = RXD in
  PORTA.OUTCLR = PIN3_bm;                // receiver mode when idle

  USART0.BAUD  = BAUD_REG;
  USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
}

uint8_t rx_count = 0;
uint32_t last_tx = 0;

void loop() {
  while (USART0.STATUS & USART_RXCIF_bm) {
    (void)USART0.RXDATAL;
    rx_count++;
  }

  if (millis() - last_tx >= 100) {
    last_tx = millis();

    USART0.CTRLB &= ~USART_RXEN_bm;   // don't count our own TX
    USART0.STATUS = USART_TXCIF_bm;
    PORTA.OUTSET  = PIN3_bm;          // DE on
    delayMicroseconds(10);

    txByte(0xAA);
    txByte(0x55);
    txByte(rx_count);

    while (!(USART0.STATUS & USART_TXCIF_bm));
    PORTA.OUTCLR = PIN3_bm;           // release the bus
    while (USART0.STATUS & USART_RXCIF_bm) (void)USART0.RXDATAL;
    USART0.CTRLB |= USART_RXEN_bm;
  }
}
