// joint_pot_slave.ino
//
// ATtiny202 Dynamixel Protocol 2.0 slave for the elephant_robot_position_sensor
// board. Reports the potentiometer voltage as "Present Position".
//
// This board wires USART0 to its DEFAULT pins, unlike the older
// JointPotentiometer example (which used the ALT position PA1/PA2):
//   pin 2 (PA6)  tx1   -> MAX485 DI
//   pin 3 (PA7)  rx1   <- MAX485 RO
//   pin 5 (PA2)  V_out <- pot wiper (via 10R + 100n RC)
//   pin 7 (PA3)  ena   -> MAX485 DE+RE (tied)
//   pin 6 (PA0)  UPDI
//
// There is no LED on this board. Control table addr 65 is still writable so
// the master's write path can be exercised; it just lands in RAM.
//
// !!! Set MY_ID per board before burning: one board 100, the other 101 !!!
//
// Tools settings: Chip = ATtiny202, Clock = 20 MHz internal.
// (Library baud is fixed at 57600, computed for 20 MHz -- burn bootloader
//  after changing the clock setting so the fuses match.)

#include <DynamixelSlave.h>

constexpr uint8_t  MY_ID    = 101;       // <-- 100 or 101, per board
constexpr uint16_t MY_MODEL = 0x4B41;
constexpr uint8_t  DIR_PIN  = PIN_PA3;

volatile uint16_t present_position = 0;  // addr 132, RO, 2 bytes
volatile uint8_t  led_state        = 0;  // addr 65,  RW, 1 byte (no physical LED)

DynamixelSlave dxl(MY_ID, MY_MODEL);

// Polled ADC on PA2 (AIN2): ~13.5 us at 20 MHz with DIV16 prescaler.
static void adcInit() {
  ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | (1 << ADC_SAMPCAP_bp);
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
  ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;        // PA2 = V_out
}

static uint16_t adcRead() {
  ADC0.COMMAND = ADC_STCONV_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  ADC0.INTFLAGS = ADC_RESRDY_bm;
  return ADC0.RES;
}

void setup() {
  adcInit();

  dxl.addItem(65,  (void*)&led_state,        1, /*readonly*/ false);
  dxl.addItem(132, (void*)&present_position, 2, /*readonly*/ true);

  dxl.setDirectionPin(DIR_PIN);
  dxl.begin(/*alt_pins=*/false);           // this board uses PA6/PA7 (default)
}

void loop() {
  uint16_t raw = adcRead();
  uint8_t sreg = SREG; cli();
  present_position = raw;
  SREG = sreg;

  dxl.process();
}
