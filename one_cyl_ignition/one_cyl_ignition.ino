/* one_cyl_ignition.ino — single-cylinder inductive ignition, one Mega2560 per cylinder.
 * Self-sufficient: reads its own VR-conditioner output directly, blanks the
 * trailing-edge twin internally, fires a fixed-timing spark. No external cleaner.
 *
 *   VR sensor -> conditioner -> ICP5 (pin 48, Timer5 capture)
 *   spark fires AFTER_EDGE_DEG after the kept (leading) edge, using last rev's period:
 *     OCR5A (spark) = cap + fracTicks           -> coil LOW  = SPARK
 *     OCR5B (dwell) = cap + fracTicks - DWELL    -> coil HIGH = charge
 *   Smart coil (D514A): pin HIGH = charging, pin LOW = fire, idle LOW.
 *
 * Trigger edge = leading edge ~30 ATDC (provisional) = 330 BTDC of next TDC.
 * Same flash on all three cylinders (sensor + TDC both step 120 deg -> cancels).
 * The 120 deg phasing lives in each board's wiring to its own sensor/coil.
 * VERIFY the reference angle with a timing light before running on fuel.
 *
 * HARDWARE: 10k from pin 5 to GND at the pin (holds coil OFF through reset/brownout).
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/* ---- config ---- */
#define TRIGGER_ANGLE_BTDC   330      // leading edge 30 ATDC = 330 BTDC of next TDC
#define ADVANCE_BTDC         15       // fixed advance (stock idle, all cyls). Try 10 for first crank.
#define AFTER_EDGE_DEG       (TRIGGER_ANGLE_BTDC - ADVANCE_BTDC)  // edge->spark = 315

#define DWELL_US             3000UL   // D514A ~3 ms
#define MAX_DWELL_US         5000UL   // watchdog: never charge longer than this
#define STALL_US             300000UL // no edge this long -> force safe
#define CAPTURE_RISING       1        // match conditioner output polarity

/* ---- timebase (/64 @ 16 MHz -> 4 us/tick) ---- */
#define US_PER_TICK          4UL
#define US_TO_TICKS(us)      ((uint32_t)(us)/US_PER_TICK)
#define DWELL_TICKS          US_TO_TICKS(DWELL_US)
#define PERIOD_MIN_TICKS     US_TO_TICKS(6000UL)   // ~10000 rpm ceiling
#define PERIOD_MAX_TICKS     60000U                // ~250 rpm floor
#define FIXED_BLANK_TICKS    US_TO_TICKS(3000UL)   // startup blank floor (tune to magnet width)

/* ---- coil pin (D5 = PE3) ---- */
#define COIL_DDR   DDRE
#define COIL_PORT  PORTE
#define COIL_BIT   PE3
#define COIL_HIGH() (COIL_PORT |=  _BV(COIL_BIT))
#define COIL_LOW()  (COIL_PORT &= ~_BV(COIL_BIT))

/* ---- shared state ---- */
volatile uint16_t lastCapture    = 0;
volatile uint16_t lastPeriod     = 0;
volatile uint8_t  ovfSinceCap    = 0;
volatile bool     synced         = false;
volatile bool     coilCharging   = false;
volatile uint32_t coilHighMicros = 0;
volatile uint32_t lastEdgeMicros = 0;

ISR(TIMER5_OVF_vect){ if (ovfSinceCap < 255) ovfSinceCap++; }

/* ---- capture: kept leading edge, blank the trailing twin ---- */
ISR(TIMER5_CAPT_vect){
  uint16_t cap      = ICR5;
  uint16_t interval = cap - lastCapture;

  // Too slow / just started -> fresh sync point, don't fire.
  if (ovfSinceCap > 1){
    lastCapture=cap; ovfSinceCap=0; synced=false;
    lastEdgeMicros=micros(); return;
  }

  // Blank the trailing twin: ignore any edge closer than max(fixed floor,
  // 3/8 of last period) to the previous kept edge. Anchor stays on the kept edge.
  uint16_t blank = FIXED_BLANK_TICKS;
  if (synced){
    uint16_t dyn = (uint16_t)(((uint32_t)lastPeriod * 3U) / 8U);
    if (dyn > blank) blank = dyn;
  }
  if (interval < blank) return;   // trailing twin -> discard

  // Plausibility gate: reject noise / impossible speeds.
  if (interval < PERIOD_MIN_TICKS || interval > PERIOD_MAX_TICKS){
    lastCapture=cap; ovfSinceCap=0; synced=false;
    lastEdgeMicros=micros(); return;
  }

  // First good edge just establishes the period; fire from the second on.
  if (!synced){
    lastCapture=cap; lastPeriod=interval; ovfSinceCap=0;
    synced=true; lastEdgeMicros=micros(); return;
  }

  // ---- schedule this rev's spark (integer, no float) ----
  uint16_t period    = interval;
  uint32_t fracTicks = (uint32_t)period * AFTER_EDGE_DEG / 360UL;
  uint16_t sparkAt   = cap + (uint16_t)fracTicks;

  TIFR5 = _BV(OCF5A) | _BV(OCF5B);
  OCR5A = sparkAt;
  if (fracTicks > DWELL_TICKS){
    OCR5B = cap + (uint16_t)(fracTicks - DWELL_TICKS);
    TIMSK5 |= _BV(OCIE5A) | _BV(OCIE5B);
  } else {
    COIL_HIGH();
    coilCharging=true; coilHighMicros=micros();
    TIMSK5 |= _BV(OCIE5A);
  }

  lastCapture=cap; lastPeriod=period; ovfSinceCap=0;
  synced=true; lastEdgeMicros=micros();
}

/* ---- dwell start ---- */
ISR(TIMER5_COMPB_vect){
  TIMSK5 &= ~_BV(OCIE5B);
  COIL_HIGH();
  coilCharging=true; coilHighMicros=micros();
}

/* ---- spark ---- */
ISR(TIMER5_COMPA_vect){
  TIMSK5 &= ~_BV(OCIE5A);
  COIL_LOW();
  coilCharging=false;
}

void setup(){
  COIL_DDR |= _BV(COIL_BIT);
  COIL_LOW();                        // idle: not charging
  DDRL &= ~_BV(PL1);                 // ICP5 (pin 48) input

  cli();
  TCCR5A=0; TCCR5B=0;
#if CAPTURE_RISING
  TCCR5B |= _BV(ICES5);
#endif
  TCCR5B |= _BV(ICNC5);              // noise canceller
  TCCR5B |= _BV(CS51) | _BV(CS50);   // /64
  TCNT5=0;
  TIMSK5 = _BV(ICIE5) | _BV(TOIE5);
  sei();

  lastEdgeMicros = micros();
}

/* ---- safety watchdogs ---- */
void loop(){
  uint32_t now = micros();

  bool charging; uint32_t highAt;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ charging=coilCharging; highAt=coilHighMicros; }
  if (charging && (now - highAt) > MAX_DWELL_US){
    COIL_LOW();
    TIMSK5 &= ~_BV(OCIE5A);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; }
  }

  uint32_t lastEdge;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ lastEdge=lastEdgeMicros; }
  if ((now - lastEdge) > STALL_US){
    COIL_LOW();
    TIMSK5 &= ~(_BV(OCIE5A) | _BV(OCIE5B));
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; synced=false; }
  }
}
