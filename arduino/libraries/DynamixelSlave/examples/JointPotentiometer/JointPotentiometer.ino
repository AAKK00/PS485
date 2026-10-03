// JointPotentiometer.ino
//
// ATtiny202 as a Dynamixel-Protocol-2.0 slave on the same RS-485 bus as
// real Dynamixel servos driven by an OpenRB-150.
// Reports the wiper voltage of a potentiometer as "Present Position".
//
// Wiring (ATtiny202 SOIC-8):
//   pin 1 (VDD)    +5V
//   pin 2 (PA6)    <- pot wiper
//   pin 3 (PA7)    -> status LED (optional)
//   pin 4 (PA1)    -> MAX485 DI    (UART TX)
//   pin 5 (PA2)    <- MAX485 RO    (UART RX)
//   pin 6 (PA0)    UPDI (programming only)
//   pin 7 (PA3)    -> MAX485 DE+RE (tied)
//   pin 8 (GND)    GND
//
// Pot ends to +5V and GND; wiper to PA6.
// MAX485 A/B to the same RS-485 pair the Dynamixel servos sit on.
//
// Tools settings: Chip = ATtiny202, Clock = 20 MHz internal.

#include <DynamixelSlave.h>

constexpr uint8_t  MY_ID    = 101;       // pick an ID no real servo on the bus uses
constexpr uint16_t MY_MODEL = 0x4B41;    // arbitrary; lets the master tell us apart
constexpr uint8_t  DIR_PIN  = PIN_PA3;
// Baud is fixed to 57600 in the library (compile-time constant).

// Control table backing storage (live in RAM, library reads/writes through pointers).
volatile uint16_t present_position = 0;  // addr 132, RO, 2 bytes
volatile uint8_t  led_state        = 0;  // addr 65,  RW, 1 byte

DynamixelSlave dxl(MY_ID, MY_MODEL);

static inline void led(bool on) {
  if (on) PORTA.OUTSET = PIN7_bm;
  else    PORTA.OUTCLR = PIN7_bm;
}

void onWrite(uint16_t addr, uint8_t /*size*/) {
  if (addr == 65) led(led_state != 0);
}

// Polled ADC: ~13.5 us at 20 MHz with DIV16 prescaler.
static void adcInit() {
  ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | (1 << ADC_SAMPCAP_bp);
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
  ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc;        // PA6
}

static uint16_t adcRead() {
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  return ADC0.RES;
}

void setup() {
  PORTA.DIRSET = PIN7_bm;          // PA7 = LED out
  PORTA.OUTCLR = PIN7_bm;
  // PA6 stays input (default after reset).

  adcInit();

  dxl.addItem(65,  (void*)&led_state,        1, /*readonly*/ false);
  dxl.addItem(132, (void*)&present_position, 2, /*readonly*/ true);
  dxl.onWrite(onWrite);

  dxl.setDirectionPin(DIR_PIN);
  dxl.begin();

  for (uint8_t i = 0; i < 3; i++) {
      PORTA.OUTSET = PIN7_bm; delay(100);
      PORTA.OUTCLR = PIN7_bm; delay(100);
    }
}

void loop() {
  uint16_t raw = adcRead();
  uint8_t sreg = SREG; cli();
  present_position = raw;
  SREG = sreg;

  dxl.process();
}
