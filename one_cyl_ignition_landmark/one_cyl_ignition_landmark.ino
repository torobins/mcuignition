/* one_cyl_ignition_landmark.ino — EXPERIMENT branch (experiment/longest-pulse-landmark).
 *
 * Dead-reckoning PLL landmark decoder (v5). Background: the VR conditioner's
 * starter-cranking output is a repeatable ~8-edge BURST per revolution that
 * defeats the production decoder's time-based blanking (see README "Firmware sync
 * test under starter"). Every rev contains ONE uniquely-large rising-to-rising
 * interval (~90ms at ~370rpm: the long-HIGH span), ~2.5x larger than any other,
 * which a decaying peak-tracker (refBig) picks out as a once-per-rev LANDMARK.
 *
 * Earlier landmark builds (v1-v4, see git history) synced on that landmark but
 * anchored the spark to the RAW landmark edge. At rough starter cranking the edge
 * jitters +/-one burst-position (~30ms ~= 68deg), so ~40% of revs fired off a
 * mis-placed anchor — a wrong-angle spark on a real coil (confirmed by an
 * anti-correlation test on the v4 log: long revs systematically followed by short
 * ones summing back to ~2x the period = the landmark hopping, not real crank speed).
 *
 * This build runs a phase-locked loop instead: it keeps a MODEL of the crank phase
 * (phaseExt) and period (phasePeriod) and fires every rev off the MODEL, not the
 * raw edge. Each accepted landmark only nudges the model — phase by residual>>KP
 * (1/8) and frequency by residual>>KF (1/32) — so a single hopped edge moves the
 * spark by only ~1/8 of the hop while a sustained real speed change still tracks in
 * a few revs. n-rounding absorbs a genuinely missed landmark (fire on rev N+1 with
 * n=2 instead of mis-locking); a 0.6*P refractory guard ignores an extra/early edge
 * so we never double-fire; >3*P since a model landmark means we're lost -> re-acquire.
 * Leaning on the model is also the safe choice at cranking: worst case is a few
 * degrees of timing scatter, never a wild wrong-angle spark.
 *
 * DIAGNOSTIC ONLY: pin 5 drives an LED (no coil this run). The landmark edge's true
 * crank angle is NOT yet calibrated — read edgeToSpark as consistency, not correct
 * BTDC. Same 15ms WDT + MAX_DWELL safety envelope as production is kept regardless.
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

/* ---- LANDMARK classifier (unchanged from v4) ----
 * refBig is a decaying peak-tracker of the raw rising-to-rising interval; a landmark
 * is any interval > LM_NUM/LM_DEN (0.6) of refBig. Rise is capped to +25%/edge so a
 * single irregular rev can't spike the threshold, and only intervals < PERIOD_MAX
 * may raise it (an idle gap or capture glitch must not poison it). */
#define LM_NUM               5        // landmark if interval*LM_NUM > refBig*LM_DEN
#define LM_DEN               3        //   i.e. interval > 0.6 * refBig
#define REFBIG_DECAY_SHIFT   6        // refBig -= refBig>>6 on a non-peak edge

/* Acquisition: two consecutive landmark periods must agree within +/-25% to lock. */
#define SYNC_AGREE_NUM       4
#define SYNC_AGREE_DEN       3

/* ---- PLL loop filter ----
 * Correction applied to the model per accepted landmark: phase by residual>>KP,
 * frequency by residual>>KF. Gentle gains (1/8, 1/32) heavily attenuate per-rev
 * edge jitter/hops in the OUTPUT while still tracking a real speed change over a
 * handful of revs. */
#define PLL_KP_SHIFT         3        // phase correction  = residual >> 3  (1/8)
#define PLL_KF_SHIFT         5        // freq  correction  = residual >> 5  (1/32)
#define REFRACTORY_NUM       3        // ignore an edge < 0.6*P since last fire (extra/early)
#define REFRACTORY_DEN       5
#define LOST_REVS            3        // > this many revs since a model landmark -> re-acquire

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

/* ---- timing-light STROBE pins (D6/D7/D8 = PH3/PH4/PH5) ----
 * A short bright flash at the exact spark instant, for reading the flywheel timing
 * marks like a timing light without depending on the inductive pickup triggering on
 * the smart coil's HT pulse. Drives THREE pins together, one LED (+ its own series R,
 * ~100R) per pin to GND: sharing one pin across 3 LEDs oversourced it (~54mA) and
 * sagged them dim, so each LED gets its own pin's ~18mA at full brightness instead.
 * (Still not enough in daylight? Drive the LEDs from +12V through an NPN transistor.) */
#define STROBE_DDR   DDRH
#define STROBE_PORT  PORTH
#define STROBE_MASK  (_BV(PH3) | _BV(PH4) | _BV(PH5))   // D6, D7, D8 -- one LED each
#define STROBE_HIGH() (STROBE_PORT |=  STROBE_MASK)
#define STROBE_LOW()  (STROBE_PORT &= ~STROBE_MASK)
#define STROBE_US     1000UL         // flash width (~2.8 deg at 460rpm; wider = easier to see)

/* ---- shared state ---- */
volatile uint32_t timerHigh      = 0;
volatile uint32_t lastCaptureExt = 0;   // 32-bit extended tick of the previous edge (any)
volatile uint32_t refBig         = 0;   // decaying peak of raw interval, for landmark test
volatile uint32_t phaseExt       = 0;   // MODEL: extended tick of the last model landmark
volatile uint32_t phasePeriod    = 0;   // MODEL: rev period estimate (ticks)
volatile uint32_t prevPeriod     = 0;   // acquisition-only: previous landmark period
volatile bool     synced         = false;
volatile bool     coilCharging   = false;
volatile uint32_t coilHighMicros = 0;
volatile uint32_t lastEdgeMicros = 0;
volatile bool     strobeActive   = false;   // strobe LED currently lit
volatile uint32_t strobeHighMicros = 0;      // when it was lit, to time its width in loop()

/* ---- debug telemetry ---- */
#define DBG_NONE      0
#define DBG_SKIP      1   // burst edge, not the landmark
#define DBG_REJECTED  2   // acquisition period outside absolute plausibility
#define DBG_SYNCFIRST 3   // two landmark periods agreed -> model locked
#define DBG_FIRED     4   // scheduled once-per-rev LED pulse off the model
#define DBG_UNSCHED   5   // too slow to schedule in 16 bits
#define DBG_RELOCK    6   // >LOST_REVS since a model landmark -> re-acquire
#define DBG_EARLY     7   // refractory: extra/early edge ignored
#define DBG_SYNCCAND  8   // acquisition: landmark recorded, not yet agreeing
volatile uint8_t  dbgSeq      = 0;
volatile uint8_t  dbgEvent    = DBG_NONE;
volatile uint32_t dbgVal      = 0;   // raw interval (SKIP) or delta-since-model (locked)
volatile uint32_t dbgRefBig   = 0;
volatile uint32_t dbgPeriod   = 0;   // model period on FIRED
volatile int32_t  dbgResid    = 0;   // phase error corrected on FIRED
volatile uint32_t dbgFracUs   = 0;

ISR(TIMER5_OVF_vect){ timerHigh++; }

static inline uint32_t extendCapture(uint16_t icr){
  uint32_t high = timerHigh;
  if ((TIFR5 & _BV(TOV5)) && icr < 0x8000U) high++;
  return (high << 16) | icr;
}

/* ---- capture: classify landmark, then PLL-track the crank phase ---- */
ISR(TIMER5_CAPT_vect){
  uint16_t capRaw   = ICR5;
  uint32_t capExt   = extendCapture(capRaw);
  uint32_t interval = capExt - lastCaptureExt;   // raw edge-to-edge
  lastCaptureExt    = capExt;
  lastEdgeMicros    = micros();

  // --- landmark classifier (decaying, capped-rise peak-tracker) ---
  bool isLandmark = (refBig > 0) && (interval * LM_NUM > refBig * LM_DEN);
  if (interval > refBig){
    if (interval < PERIOD_MAX_TICKS){
      uint32_t capped = refBig + (refBig >> 2);
      refBig = (refBig == 0 || interval < capped) ? interval : capped;
    }
  } else {
    refBig -= (refBig >> REFBIG_DECAY_SHIFT);
  }
  if (!isLandmark){
    dbgEvent=DBG_SKIP; dbgVal=interval; dbgRefBig=refBig; dbgSeq++;
    return;                        // burst edge -> discard
  }

  // --- ACQUISITION: lock the model with two agreeing landmark periods ---
  if (!synced){
    uint32_t period = capExt - phaseExt;     // phaseExt holds the last landmark tick here
    phaseExt = capExt;
    if (period < PERIOD_MIN_TICKS || period > PERIOD_MAX_TICKS){
      prevPeriod = 0;
      dbgEvent=DBG_REJECTED; dbgVal=period; dbgRefBig=refBig; dbgSeq++;
      return;
    }
    bool agrees = prevPeriod &&
                  (period * SYNC_AGREE_NUM > prevPeriod * SYNC_AGREE_DEN) &&
                  (period * SYNC_AGREE_DEN < prevPeriod * SYNC_AGREE_NUM);
    prevPeriod = period;
    if (agrees){
      phasePeriod = period; synced = true;
      dbgEvent=DBG_SYNCFIRST; dbgVal=period; dbgPeriod=phasePeriod; dbgRefBig=refBig; dbgSeq++;
    } else {
      dbgEvent=DBG_SYNCCAND; dbgVal=period; dbgRefBig=refBig; dbgSeq++;
    }
    return;
  }

  // --- LOCKED: dead-reckoning PLL ---
  uint32_t delta = capExt - phaseExt;          // since the last MODEL landmark

  if (delta > phasePeriod * LOST_REVS){        // way overdue -> model is lost, re-acquire
    synced=false; prevPeriod=0; phaseExt=capExt;
    dbgEvent=DBG_RELOCK; dbgVal=delta; dbgRefBig=refBig; dbgSeq++;
    return;
  }
  if (delta * REFRACTORY_DEN < phasePeriod * REFRACTORY_NUM){   // < 0.6*P: extra/early edge
    dbgEvent=DBG_EARLY; dbgVal=delta; dbgRefBig=refBig; dbgSeq++;
    return;                                    // ignore; already fired this rev
  }

  // How many whole revs elapsed since the model landmark (>=1; handles a missed one).
  uint32_t n = (delta + (phasePeriod >> 1)) / phasePeriod;
  uint32_t predicted = phaseExt + n * phasePeriod;
  int32_t  residual  = (int32_t)(capExt - predicted);   // signed phase error, |.|<0.5*P

  // PI update of the model (gentle: heavily attenuates hops, tracks real speed slowly).
  int32_t np = (int32_t)phasePeriod + (residual >> PLL_KF_SHIFT);
  if (np < (int32_t)PERIOD_MIN_TICKS) np = PERIOD_MIN_TICKS;
  if (np > (int32_t)PERIOD_MAX_TICKS) np = PERIOD_MAX_TICKS;
  phasePeriod = (uint32_t)np;
  phaseExt    = predicted + (residual >> PLL_KP_SHIFT);

  // Schedule this rev's LED pulse off the MODEL anchor, not the raw (jittery) edge.
  uint32_t fracTicks = phasePeriod * AFTER_EDGE_DEG / 360UL;
  uint32_t sparkExt  = phaseExt + fracTicks;
  uint32_t delayExt  = sparkExt - capExt;      // ticks from now to spark
  if (delayExt == 0 || delayExt > 0xFFFFUL){
    dbgEvent=DBG_UNSCHED; dbgVal=delta; dbgRefBig=refBig; dbgSeq++;
    return;
  }
  uint16_t sparkAt = (uint16_t)sparkExt;

  OCR5A = sparkAt;
  if (delayExt > DWELL_TICKS){
    OCR5B = (uint16_t)(sparkExt - DWELL_TICKS);
    TIFR5 = _BV(OCF5A) | _BV(OCF5B);
    TIMSK5 |= _BV(OCIE5A) | _BV(OCIE5B);
  } else {
    COIL_HIGH();
    coilCharging=true; coilHighMicros=micros();
    TIFR5 = _BV(OCF5A) | _BV(OCF5B);
    TIMSK5 |= _BV(OCIE5A);
  }

  dbgEvent=DBG_FIRED; dbgVal=delta; dbgRefBig=refBig;
  dbgPeriod=phasePeriod; dbgResid=residual; dbgFracUs=fracTicks*US_PER_TICK; dbgSeq++;
}

/* ---- dwell start ---- */
ISR(TIMER5_COMPB_vect){
  TIMSK5 &= ~_BV(OCIE5B);
  COIL_HIGH();
  coilCharging=true; coilHighMicros=micros();
}

/* ---- spark (coil fires here; also kick the timing-light strobe) ---- */
ISR(TIMER5_COMPA_vect){
  TIMSK5 &= ~_BV(OCIE5A);
  COIL_LOW();
  coilCharging=false;
  STROBE_HIGH();
  strobeActive=true; strobeHighMicros=micros();
}

void setup(){
  Serial.begin(115200);
  Serial.println(F("one_cyl_ignition LANDMARK-PLL experiment build"));
  Serial.print(F("timing: AFTER_EDGE_DEG=")); Serial.print(AFTER_EDGE_DEG);
  Serial.print(F(" TRIGGER_ANGLE_BTDC=")); Serial.print(TRIGGER_ANGLE_BTDC);
  Serial.print(F(" ADVANCE_BTDC=")); Serial.println(ADVANCE_BTDC);

  COIL_DDR |= _BV(COIL_BIT);
  COIL_LOW();
  STROBE_DDR |= STROBE_MASK;
  STROBE_LOW();
  // Strobe/wiring self-test: 6 clearly-visible blinks at boot, so pin 6 + the LED
  // wiring can be verified WITHOUT cranking. The real strobe is only 300us and is
  // nearly invisible to the eye -- during cranking, watch the marks it lights, not
  // the LED itself.
  for (uint8_t i=0; i<6; i++){ STROBE_HIGH(); delay(90); STROBE_LOW(); delay(90); }

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
  uint8_t  seq, ev; uint32_t val, rb, period, fracUs; int32_t resid;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    seq=dbgSeq; ev=dbgEvent; val=dbgVal; rb=dbgRefBig;
    period=dbgPeriod; resid=dbgResid; fracUs=dbgFracUs;
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
    case DBG_RELOCK:    Serial.print(F("RELOCK   ")); break;
    case DBG_EARLY:     Serial.print(F("EARLY    ")); break;
    case DBG_SYNCCAND:  Serial.print(F("SYNCCAND ")); break;
    default:            Serial.print(F("NONE     ")); break;
  }
  Serial.print(F(" val_us="));    Serial.print(val * US_PER_TICK);
  Serial.print(F(" refBig_us=")); Serial.print(rb * US_PER_TICK);
  if (ev == DBG_FIRED){
    Serial.print(F(" period_us=")); Serial.print(period * US_PER_TICK);
    Serial.print(F(" rpm="));       Serial.print(60000000UL / (period * US_PER_TICK));
    Serial.print(F(" resid_us="));  Serial.print(resid * (int32_t)US_PER_TICK);
    Serial.print(F(" edgeToSpark_us=")); Serial.print(fracUs);
  }
  Serial.println();
}

/* ---- safety watchdogs (unchanged from production/debug) ---- */
void loop(){
  wdt_reset();
  uint32_t now = micros();

  // End the strobe flash a fixed width after the spark instant (non-blocking).
  if (strobeActive && (int32_t)(now - strobeHighMicros) > (int32_t)STROBE_US){
    STROBE_LOW();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ strobeActive=false; }
  }

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
    // keep refBig warm across the stall; just drop the lock and acquisition state
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; synced=false; prevPeriod=0; }
    if (!stallLatched && Serial.availableForWrite() >= (SERIAL_TX_BUFFER_SIZE - 1)){
      Serial.print(F("WATCHDOG: STALL gap_us=")); Serial.println((int32_t)(now - lastEdge));
      stallLatched = true;
    }
  } else {
    stallLatched = false;
  }

  printDebug();
}
