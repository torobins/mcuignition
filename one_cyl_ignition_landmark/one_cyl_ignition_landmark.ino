/* one_cyl_ignition_landmark.ino — EXPERIMENT branch (experiment/longest-pulse-landmark).
 *
 * Hypothesis under test: the VR conditioner's starter-cranking output is a
 * repeatable multi-edge BURST per revolution (~8 edges), which defeats the
 * production decoder's time-based blanking (blank = lastPeriod/2 becomes shorter
 * than the burst itself at starter speed, so a trailing burst edge leaks past
 * and mis-syncs — see README "Firmware sync test under starter"). But every rev
 * contains ONE uniquely-large rising-to-rising interval (~90us at ~370rpm: the
 * ~84ms long HIGH plus its trailing gap), ~2.5x larger than any other interval
 * in the rev. Offline analysis of the vr_logger captures showed that landmark
 * recurs at a stable ~370rpm (period CV ~8%, 94-95% of landmark-to-landmark
 * periods within the +/-25% sync-agree band).
 *
 * This build keys sync on that landmark instead of a time-blank: classify each
 * rising-to-rising interval as LANDMARK (uniquely large) or SKIP (burst), measure
 * the revolution period landmark-to-landmark, and run the SAME proven plausibility
 * / sync-agree / rate gates on those landmark periods. Everything below the ISR's
 * landmark classifier (extendCapture, the gates, scheduling, the safety watchdogs,
 * the non-blocking telemetry) is carried over unchanged from one_cyl_ignition_debug.
 *
 * DIAGNOSTIC ONLY: pin 5 drives an LED (no coil this run), so the "spark" schedule
 * is just a once-per-rev blink at AFTER_EDGE_DEG past the landmark edge. The point
 * is to see whether landmark sync HOLDS on the real starter signal (steady rpm,
 * SYNCFIRST then sustained FIRED) where the production decoder mis-fired at
 * 486-1735rpm. The landmark edge's true crank angle is NOT yet calibrated — do not
 * read edgeToSpark as correct timing, only as rev-to-rev CONSISTENCY. Same 15ms
 * WDT + MAX_DWELL safety envelope as production is kept regardless.
 *
 *   VR sensor -> conditioner -> ICP5 (pin 48, Timer5 capture, rising edges)
 * HARDWARE: 10k from pin 5 to GND at the pin (holds output OFF through reset).
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>

/* ---- watchdog: recovers a hung loop(); disabled in .init3 to avoid a boot loop
 * through the stock bootloader (same reasoning as one_cyl_ignition_debug). ---- */
void wdt_early_disable(void) __attribute__((naked, used, section(".init3")));
void wdt_early_disable(void){
  MCUSR = 0;
  wdt_disable();
}

/* ---- config ---- */
#define TRIGGER_ANGLE_BTDC   330      // provisional; landmark edge angle NOT yet
#define ADVANCE_BTDC         15       // calibrated for this build (see header).
#define AFTER_EDGE_DEG       (TRIGGER_ANGLE_BTDC - ADVANCE_BTDC)  // edge->spark = 315

#define DWELL_US             3000UL   // D514A ~3 ms
#define MAX_DWELL_US         5000UL   // watchdog: never charge longer than this
#define STALL_US             3000000UL // no edge this long -> force safe
#define CAPTURE_RISING       1        // match conditioner output polarity
#define RATEJUMP_ESCAPE_COUNT 3       // consecutive rate-gate rejections before rebuild

/* ---- LANDMARK classifier ----
 * refBig is a decaying peak-tracker of the raw rising-to-rising interval. An
 * interval is a LANDMARK if it exceeds LM_NUM/LM_DEN (0.6) of refBig. The long
 * interval (~90us) sits at the peak; the next-largest burst interval (~36us) is
 * ~0.4x it, so 0.6 cleanly separates them with margin on both sides. The long
 * interval always immediately precedes the largest burst interval in the rev, so
 * refBig is freshest (highest) exactly when that burst interval is tested. Decay
 * (>>REFBIG_DECAY_SHIFT per edge, ~1.5%) lets refBig track real accel/decel. */
#define LM_NUM               5        // landmark if interval*LM_NUM > refBig*LM_DEN
#define LM_DEN               3        //   i.e. interval > 0.6 * refBig
#define REFBIG_DECAY_SHIFT   6        // refBig -= refBig>>6 on a non-peak edge

/* Two consecutive LANDMARK periods must agree within +/-25% before sync. */
#define SYNC_AGREE_NUM       4
#define SYNC_AGREE_DEN       3

/* Rate-of-change bounds on landmark periods (asymmetric, same rationale as prod). */
#define RATE_MAX_NUM         3        // reject if period > lastPeriod * 3/2
#define RATE_MAX_DEN         2
#define RATE_MIN_NUM         1        // reject if period < lastPeriod * 1/2
#define RATE_MIN_DEN         2

/* ---- timebase (/256 @ 16 MHz -> 16 us/tick) ---- */
#define US_PER_TICK          16UL
#define US_TO_TICKS(us)      ((uint32_t)(us)/US_PER_TICK)
#define DWELL_TICKS          US_TO_TICKS(DWELL_US)
#define PERIOD_MIN_TICKS     US_TO_TICKS(6000UL)      // ~10000 rpm ceiling
#define PERIOD_MAX_TICKS     US_TO_TICKS(1100000UL)   // ~54.5rpm floor (uint32_t compare)

/* ---- LED/coil pin (D5 = PE3) ---- */
#define COIL_DDR   DDRE
#define COIL_PORT  PORTE
#define COIL_BIT   PE3
#define COIL_HIGH() (COIL_PORT |=  _BV(COIL_BIT))
#define COIL_LOW()  (COIL_PORT &= ~_BV(COIL_BIT))

/* ---- shared state ---- */
volatile uint32_t timerHigh       = 0;
volatile uint32_t lastCaptureExt  = 0;   // 32-bit extended tick of the previous edge (any)
volatile uint32_t lastLandmarkExt = 0;   // 32-bit extended tick of the previous LANDMARK
volatile uint32_t refBig          = 0;   // decaying peak of raw interval, for landmark test
volatile uint32_t lastPeriod      = 0;   // last accepted landmark-to-landmark period
volatile uint32_t prevPeriod      = 0;   // previous landmark period (sync-agree reference)
volatile bool     synced          = false;
volatile bool     coilCharging    = false;
volatile uint32_t coilHighMicros  = 0;
volatile uint32_t lastEdgeMicros  = 0;
volatile uint8_t  rateJumpStreak  = 0;

/* ---- debug telemetry ---- */
#define DBG_NONE      0
#define DBG_SKIP      1   // burst edge, not the landmark -> ignored
#define DBG_REJECTED  2   // landmark period outside absolute plausibility
#define DBG_SYNCFIRST 3   // two landmark periods agreed -> synced, no fire yet
#define DBG_FIRED     4   // scheduled once-per-rev LED pulse
#define DBG_UNSCHED   5   // landmark period too slow to schedule
#define DBG_RATEJUMP  6   // landmark period outside 0.5x-1.5x of lastPeriod
#define DBG_ESCAPE    7   // rate-gate escape hatch fired
#define DBG_SYNCCAND  8   // landmark recorded, not yet agreeing
volatile uint8_t  dbgSeq      = 0;
volatile uint8_t  dbgEvent    = DBG_NONE;
volatile uint32_t dbgInterval = 0;   // raw interval (SKIP) or landmark period (others)
volatile uint32_t dbgRefBig   = 0;
volatile uint32_t dbgPeriod   = 0;
volatile uint32_t dbgFracUs   = 0;

ISR(TIMER5_OVF_vect){ timerHigh++; }

static inline uint32_t extendCapture(uint16_t icr){
  uint32_t high = timerHigh;
  if ((TIFR5 & _BV(TOV5)) && icr < 0x8000U) high++;
  return (high << 16) | icr;
}

/* ---- capture: classify landmark vs burst, sync on the landmark ---- */
ISR(TIMER5_CAPT_vect){
  uint16_t capRaw   = ICR5;
  uint32_t capExt   = extendCapture(capRaw);
  uint32_t interval = capExt - lastCaptureExt;   // raw edge-to-edge
  lastCaptureExt    = capExt;
  lastEdgeMicros    = micros();

  // Classify against the current peak BEFORE folding this interval into it.
  bool isLandmark = (refBig > 0) && (interval * LM_NUM > refBig * LM_DEN);
  // Only PLAUSIBLE intervals may raise the peak. An idle gap before the first
  // crank edge, or a capture-extension glitch, is many seconds long and would
  // otherwise latch refBig astronomically high (the slow decay can't recover
  // within a cranking burst) so that no real ~90ms landmark ever clears the
  // 0.6*refBig threshold — observed as an all-SKIP run. A real landmark
  // interval, even at the ~55rpm floor, stays under PERIOD_MAX_TICKS.
  if (interval > refBig){
    if (interval < PERIOD_MAX_TICKS) refBig = interval;
  } else {
    refBig -= (refBig >> REFBIG_DECAY_SHIFT);
  }

  if (!isLandmark){
    dbgEvent=DBG_SKIP; dbgInterval=interval; dbgRefBig=refBig; dbgSeq++;
    return;                        // burst edge -> discard for sync/timing
  }

  // ---- landmark: this rev's once-per-rev reference edge ----
  uint32_t period = capExt - lastLandmarkExt;
  lastLandmarkExt = capExt;        // advance the anchor even if we reject below

  // Absolute plausibility.
  if (period < PERIOD_MIN_TICKS || period > PERIOD_MAX_TICKS){
    synced=false; prevPeriod=0;
    dbgEvent=DBG_REJECTED; dbgInterval=period; dbgRefBig=refBig; dbgSeq++;
    return;
  }

  // Rate-of-change gate, with the same escape hatch as production.
  if (synced && ((period * RATE_MAX_DEN > lastPeriod * RATE_MAX_NUM) ||
                 (period * RATE_MIN_DEN < lastPeriod * RATE_MIN_NUM))){
    if (++rateJumpStreak < RATEJUMP_ESCAPE_COUNT){
      synced=false; prevPeriod=period;
      dbgEvent=DBG_RATEJUMP; dbgInterval=period; dbgRefBig=refBig; dbgSeq++;
      return;
    }
    rateJumpStreak=0; lastPeriod=period; prevPeriod=period; synced=true;
    dbgEvent=DBG_ESCAPE; dbgInterval=period; dbgRefBig=refBig; dbgSeq++;
    return;
  }

  // Acquisition: two consecutive landmark periods must agree.
  if (!synced){
    bool agrees = prevPeriod &&
                  (period * SYNC_AGREE_NUM > prevPeriod * SYNC_AGREE_DEN) &&
                  (period * SYNC_AGREE_DEN < prevPeriod * SYNC_AGREE_NUM);
    prevPeriod = period;
    if (agrees){
      lastPeriod=period; synced=true;
      dbgEvent=DBG_SYNCFIRST; dbgInterval=period; dbgRefBig=refBig; dbgSeq++;
      return;
    }
    dbgEvent=DBG_SYNCCAND; dbgInterval=period; dbgRefBig=refBig; dbgSeq++;
    return;
  }

  // ---- schedule this rev's once-per-rev LED pulse off the landmark edge ----
  uint32_t fracTicks = period * AFTER_EDGE_DEG / 360UL;
  if (fracTicks > 0xFFFFUL){
    lastPeriod=period; prevPeriod=period; synced=true;
    dbgEvent=DBG_UNSCHED; dbgInterval=period; dbgRefBig=refBig; dbgSeq++;
    return;
  }
  uint16_t sparkAt = capRaw + (uint16_t)fracTicks;

  OCR5A = sparkAt;
  if (fracTicks > DWELL_TICKS){
    OCR5B = capRaw + (uint16_t)(fracTicks - DWELL_TICKS);
    TIFR5 = _BV(OCF5A) | _BV(OCF5B);
    TIMSK5 |= _BV(OCIE5A) | _BV(OCIE5B);
  } else {
    COIL_HIGH();
    coilCharging=true; coilHighMicros=micros();
    TIFR5 = _BV(OCF5A) | _BV(OCF5B);
    TIMSK5 |= _BV(OCIE5A);
  }

  rateJumpStreak=0; lastPeriod=period; prevPeriod=period; synced=true;
  dbgEvent=DBG_FIRED; dbgInterval=period; dbgRefBig=refBig;
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
  Serial.println(F("one_cyl_ignition LANDMARK experiment build"));

  COIL_DDR |= _BV(COIL_BIT);
  COIL_LOW();
  DDRL &= ~_BV(PL1);                 // ICP5 (pin 48) input

  cli();
  TCCR5A=0; TCCR5B=0;
#if CAPTURE_RISING
  TCCR5B |= _BV(ICES5);
#endif
  TCCR5B |= _BV(ICNC5);              // noise canceller
  TCCR5B |= _BV(CS52);               // /256
  TCNT5=0;
  TIMSK5 = _BV(ICIE5) | _BV(TOIE5);
  sei();

  lastEdgeMicros = micros();
  wdt_enable(WDTO_15MS);
}

/* ---- non-blocking telemetry (same discipline as one_cyl_ignition_debug) ---- */
uint8_t lastPrintedSeq = 0;
void printDebug(){
  uint8_t  seq, ev; uint32_t interval, rb, period, fracUs;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    seq=dbgSeq; ev=dbgEvent; interval=dbgInterval; rb=dbgRefBig;
    period=dbgPeriod; fracUs=dbgFracUs;
  }
  if (seq == lastPrintedSeq) return;
  if (Serial.availableForWrite() < (SERIAL_TX_BUFFER_SIZE - 1)) return;
  uint8_t missed = seq - lastPrintedSeq - 1;
  lastPrintedSeq = seq;

  Serial.print(F("seq=")); Serial.print(seq);
  if (missed) { Serial.print(F(" missed=")); Serial.print(missed); }
  Serial.print(F(" ev="));
  switch (ev){
    case DBG_SKIP:      Serial.print(F("SKIP     ")); break;
    case DBG_REJECTED:  Serial.print(F("REJECTED ")); break;
    case DBG_SYNCFIRST: Serial.print(F("SYNCFIRST")); break;
    case DBG_FIRED:     Serial.print(F("FIRED    ")); break;
    case DBG_UNSCHED:   Serial.print(F("UNSCHED  ")); break;
    case DBG_RATEJUMP:  Serial.print(F("RATEJUMP ")); break;
    case DBG_ESCAPE:    Serial.print(F("ESCAPE   ")); break;
    case DBG_SYNCCAND:  Serial.print(F("SYNCCAND ")); break;
    default:            Serial.print(F("NONE     ")); break;
  }
  // interval_us is the raw edge interval for SKIP, else the landmark period.
  Serial.print(F(" val_us="));   Serial.print(interval * US_PER_TICK);
  Serial.print(F(" refBig_us=")); Serial.print(rb * US_PER_TICK);
  if (ev == DBG_FIRED){
    Serial.print(F(" period_us=")); Serial.print(period * US_PER_TICK);
    Serial.print(F(" rpm="));       Serial.print(60000000UL / (period * US_PER_TICK));
    Serial.print(F(" edgeToSpark_us=")); Serial.print(fracUs);
  }
  Serial.println();
}

/* ---- safety watchdogs (unchanged from production/debug) ---- */
void loop(){
  wdt_reset();
  uint32_t now = micros();

  bool charging; uint32_t highAt;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ charging=coilCharging; highAt=coilHighMicros; }
  if (charging && (int32_t)(now - highAt) > (int32_t)MAX_DWELL_US){
    COIL_LOW();
    TIMSK5 &= ~_BV(OCIE5A);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; }
    if (Serial.availableForWrite() >= (SERIAL_TX_BUFFER_SIZE - 1)){
      Serial.println(F("WATCHDOG: MAX_DWELL exceeded, forced coil LOW"));
    }
  }

  static bool stallLatched = false;
  uint32_t lastEdge;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ lastEdge=lastEdgeMicros; }
  if ((int32_t)(now - lastEdge) > (int32_t)STALL_US){
    COIL_LOW();
    TIMSK5 &= ~(_BV(OCIE5A) | _BV(OCIE5B));
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; synced=false; prevPeriod=0; refBig=0; }
    if (!stallLatched && Serial.availableForWrite() >= (SERIAL_TX_BUFFER_SIZE - 1)){
      Serial.print(F("WATCHDOG: STALL gap_us=")); Serial.println((int32_t)(now - lastEdge));
      stallLatched = true;
    }
  } else {
    stallLatched = false;
  }

  printDebug();
}
