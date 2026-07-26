/* one_cyl_ignition_debug.ino — bench-diagnostic build of one_cyl_ignition.ino.
 * Identical ignition logic to the production sketch, plus non-blocking serial
 * telemetry (115200 baud) so the capture ISR's internal decisions can be
 * observed live: every BLANKED/REJECTED/SYNCFIRST/FIRED/UNSCHED event, plus
 * watchdog trips, are logged from loop() without ever blocking the ISR path.
 *
 * Use this build when bringing up a new board, characterizing a real sensor's
 * twin-pulse gap, or diagnosing sync/timing issues on the bench (e.g. with
 * pulse_simulator). Flash the plain one_cyl_ignition.ino for actual engine use
 * - it has no Serial overhead and is otherwise byte-for-byte the same logic.
 *
 * single-cylinder inductive ignition, one Mega2560 per cylinder.
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
#define STALL_US             1500000UL // no edge this long -> force safe (must exceed the
                                        // period of the slowest speed we want to run at:
                                        // PERIOD_MAX_TICKS below implies an ~80rpm floor,
                                        // whose period is 750000us, so this needs real
                                        // margin above that or the watchdog force-drops
                                        // sync between every single legitimate revolution)
#define CAPTURE_RISING       1        // match conditioner output polarity

/* ---- timebase (/64 @ 16 MHz -> 4 us/tick) ---- */
#define US_PER_TICK          4UL
#define US_TO_TICKS(us)      ((uint32_t)(us)/US_PER_TICK)
#define DWELL_TICKS          US_TO_TICKS(DWELL_US)
#define PERIOD_MIN_TICKS     US_TO_TICKS(6000UL)     // ~10000 rpm ceiling
#define PERIOD_MAX_TICKS     US_TO_TICKS(750000UL)   // ~80rpm floor. Safe now that capture
                                                      // timestamps are 32-bit extended (see
                                                      // extendCapture()) instead of raw 16-bit
                                                      // ICR5 differences, which could only ever
                                                      // represent periods up to ~262ms (~229rpm)
                                                      // before silently aliasing.
#define FIXED_BLANK_TICKS    US_TO_TICKS(3000UL)   // startup blank floor (tune to magnet width)

/* ---- coil pin (D5 = PE3) ---- */
#define COIL_DDR   DDRE
#define COIL_PORT  PORTE
#define COIL_BIT   PE3
#define COIL_HIGH() (COIL_PORT |=  _BV(COIL_BIT))
#define COIL_LOW()  (COIL_PORT &= ~_BV(COIL_BIT))

/* ---- shared state ---- */
// lastCaptureExt/lastPeriod are 32-bit EXTENDED tick counts (see extendCapture()),
// not raw ICR5 values - this is what actually fixes the low-rpm aliasing, not a
// threshold tweak. A bare 16-bit ICR5 difference can only ever represent periods
// up to ~262ms (~229rpm) before silently wrapping to a wrong, too-small value.
volatile uint32_t timerHigh      = 0;   // software-extended high word of TCNT5
volatile uint32_t lastCaptureExt = 0;
volatile uint32_t lastPeriod     = 0;
volatile bool     synced         = false;
volatile bool     coilCharging   = false;
volatile uint32_t coilHighMicros = 0;
volatile uint32_t lastEdgeMicros = 0;

/* ---- debug telemetry (loop() prints, ISR only does cheap int copies) ---- */
#define DBG_NONE      0
#define DBG_BLANKED   1   // trailing twin discarded
#define DBG_REJECTED  2   // implausible interval
#define DBG_SYNCFIRST 3   // first good edge, period established, no fire yet
#define DBG_FIRED     4   // normal scheduled spark
#define DBG_UNSCHED   5   // period measured fine, but too slow to SCHEDULE (see below)
volatile uint8_t  dbgSeq      = 0;
volatile uint8_t  dbgEvent    = DBG_NONE;
volatile uint32_t dbgInterval = 0;
volatile uint32_t dbgBlank    = 0;
volatile uint32_t dbgPeriod   = 0;
volatile uint32_t dbgFracUs   = 0;

ISR(TIMER5_OVF_vect){ timerHigh++; }

// Combine the raw 16-bit ICR5 capture with the software-extended high word to
// get a full 32-bit tick count with no aliasing, up to ~4.7 hours before wrap.
// Handles the classic capture-vs-overflow race: TIMER5_CAPT has higher interrupt
// priority than TIMER5_OVF on this MCU, so if both fire together, CAPT runs first
// while TOV5 may already be set in hardware but not yet serviced. If the just-
// captured value is small (post-wrap) while TOV5 is pending, that overflow
// hasn't been counted into timerHigh yet, so we account for it here.
static inline uint32_t extendCapture(uint16_t icr){
  uint32_t high = timerHigh;
  if ((TIFR5 & _BV(TOV5)) && icr < 0x8000U) high++;
  return (high << 16) | icr;
}

/* ---- capture: kept leading edge, blank the trailing twin ---- */
ISR(TIMER5_CAPT_vect){
  uint16_t capRaw = ICR5;
  uint32_t capExt = extendCapture(capRaw);
  uint32_t interval = capExt - lastCaptureExt;

  // Blank the trailing twin: ignore any edge closer than max(fixed floor,
  // 3/8 of last period) to the previous kept edge. Anchor stays on the kept edge.
  uint32_t blank = FIXED_BLANK_TICKS;
  if (synced){
    uint32_t dyn = (lastPeriod * 3UL) / 8UL;
    if (dyn > blank) blank = dyn;
  }
  if (interval < blank){
    dbgEvent=DBG_BLANKED; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
    return;   // trailing twin -> discard
  }

  // Plausibility gate: reject noise / impossible speeds.
  if (interval < PERIOD_MIN_TICKS || interval > PERIOD_MAX_TICKS){
    lastCaptureExt=capExt; synced=false;
    lastEdgeMicros=micros();
    dbgEvent=DBG_REJECTED; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
    return;
  }

  // First good edge just establishes the period; fire from the second on.
  if (!synced){
    lastCaptureExt=capExt; lastPeriod=interval;
    synced=true; lastEdgeMicros=micros();
    dbgEvent=DBG_SYNCFIRST; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
    return;
  }

  // ---- schedule this rev's spark (integer, no float) ----
  // Scheduling still uses the RAW 16-bit capRaw + fracTicks with natural
  // uint16_t wraparound, because OCR5A/OCR5B are 16-bit hardware compare
  // registers matched against the free-running 16-bit TCNT5 - that part was
  // never broken. Only the interval/period MEASUREMENT needed the 32-bit fix.
  //
  // But fracTicks (the edge-to-spark delay, AFTER_EDGE_DEG/360 of a revolution)
  // must itself fit in 16 bits to be scheduled correctly - if it doesn't, the
  // (uint16_t) cast below would silently wrap to a much smaller value and the
  // coil would fire at a bogus, wrong crank angle instead of visibly failing.
  // With AFTER_EDGE_DEG=315 this caps real minimum speed at ~200rpm - a genuine
  // hardware ceiling of one 16-bit output-compare register, not a tunable
  // threshold. Confirmed by bench test: below ~200rpm this is what was causing
  // MAX_DWELL trips (wrong dwell/spark gap from the wrapped compare targets).
  uint32_t period    = interval;
  uint32_t fracTicks = period * AFTER_EDGE_DEG / 360UL;
  if (fracTicks > 0xFFFFUL){
    lastCaptureExt=capExt; lastPeriod=period; synced=true;
    lastEdgeMicros=micros();
    dbgEvent=DBG_UNSCHED; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
    return;
  }
  uint16_t sparkAt   = capRaw + (uint16_t)fracTicks;

  TIFR5 = _BV(OCF5A) | _BV(OCF5B);
  OCR5A = sparkAt;
  if (fracTicks > DWELL_TICKS){
    OCR5B = capRaw + (uint16_t)(fracTicks - DWELL_TICKS);
    TIMSK5 |= _BV(OCIE5A) | _BV(OCIE5B);
  } else {
    COIL_HIGH();
    coilCharging=true; coilHighMicros=micros();
    TIMSK5 |= _BV(OCIE5A);
  }

  lastCaptureExt=capExt; lastPeriod=period;
  synced=true; lastEdgeMicros=micros();
  dbgEvent=DBG_FIRED; dbgInterval=interval; dbgBlank=blank;
  dbgPeriod=period; dbgFracUs=fracTicks*US_PER_TICK; dbgSeq++;
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
  Serial.begin(115200);
  Serial.println(F("one_cyl_ignition DEBUG build"));

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

/* ---- debug telemetry printer (loop-side only, never blocks the ISR) ---- */
uint8_t lastPrintedSeq = 0;
void printDebug(){
  uint8_t  seq, ev; uint32_t interval, blank, period, fracUs;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    seq=dbgSeq; ev=dbgEvent; interval=dbgInterval; blank=dbgBlank;
    period=dbgPeriod; fracUs=dbgFracUs;
  }
  if (seq == lastPrintedSeq) return;
  uint8_t missed = seq - lastPrintedSeq - 1;
  lastPrintedSeq = seq;

  Serial.print(F("seq=")); Serial.print(seq);
  if (missed) { Serial.print(F(" missed=")); Serial.print(missed); }
  Serial.print(F(" ev="));
  switch (ev){
    case DBG_BLANKED:   Serial.print(F("BLANKED  ")); break;
    case DBG_REJECTED:  Serial.print(F("REJECTED ")); break;
    case DBG_SYNCFIRST: Serial.print(F("SYNCFIRST")); break;
    case DBG_FIRED:      Serial.print(F("FIRED    ")); break;
    case DBG_UNSCHED:    Serial.print(F("UNSCHED  ")); break;
    default:             Serial.print(F("NONE     ")); break;
  }
  Serial.print(F(" interval_us=")); Serial.print(interval * US_PER_TICK);
  Serial.print(F(" blank_us="));    Serial.print(blank * US_PER_TICK);
  if (ev == DBG_FIRED){
    Serial.print(F(" period_us=")); Serial.print(period * US_PER_TICK);
    Serial.print(F(" rpm="));       Serial.print(60000000UL / (period * US_PER_TICK));
    Serial.print(F(" edgeToSpark_us=")); Serial.print(fracUs);
  }
  Serial.println();
}

/* ---- safety watchdogs ---- */
void loop(){
  uint32_t now = micros();

  bool charging; uint32_t highAt;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ charging=coilCharging; highAt=coilHighMicros; }
  if (charging && (int32_t)(now - highAt) > (int32_t)MAX_DWELL_US){
    COIL_LOW();
    TIMSK5 &= ~_BV(OCIE5A);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; }
    Serial.println(F("WATCHDOG: MAX_DWELL exceeded, forced coil LOW"));
  }

  static bool stallLatched = false;
  uint32_t lastEdge;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ lastEdge=lastEdgeMicros; }
  if ((int32_t)(now - lastEdge) > (int32_t)STALL_US){
    COIL_LOW();
    TIMSK5 &= ~(_BV(OCIE5A) | _BV(OCIE5B));
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; synced=false; }
    if (!stallLatched){
      Serial.print(F("WATCHDOG: STALL gap_us=")); Serial.println((int32_t)(now - lastEdge));
      stallLatched = true;
    }
  } else {
    stallLatched = false;
  }

  printDebug();
}
