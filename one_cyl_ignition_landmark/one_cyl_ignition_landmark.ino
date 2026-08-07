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
 * STATUS (2026-08-01): bench-validated on the real starter with the real Dyna D514A
 * coil — confirmed spark every rev. Timing CALIBRATED: with a charged battery
 * (steady ~436rpm crank) the strobe put the fixed-15 build at ~15deg BTDC, so
 * TRIGGER_ANGLE_BTDC=330 is correct (calibration is speed-independent geometry). The
 * cranking retard below is verified working — spark sits in a ~0-15deg BTDC window
 * near TDC at cranking (kickback-safe). NOT yet started on fuel (carbs off); the
 * running advance (ADVANCE_BTDC) still gets its final dial-in at idle with a timing
 * light. Same 15ms WDT + MAX_DWELL safety envelope as production.
 *
 * This is still the experiment/debug build: it also drives a calibration STROBE on
 * D6/D7/D8 (flash at the spark instant, for reading flywheel marks) and prints
 * telemetry. Once proven on fuel, port the decoder into production one_cyl_ignition.ino.
 *
 *   VR sensor -> conditioner -> ICP5 (pin 48, Timer5 capture, rising edges)
 *   spark: Dyna D514A smart coil on pin 5 (HIGH=charge, LOW=fire)
 * HARDWARE: 10k from pin 5 to GND at the coil end (holds the coil trigger OFF through
 * reset/brownout even if the signal wire is loose; an LED is NOT a substitute -- it's
 * a diode, open below its forward voltage, so it can't hold the line low).
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
/* CALIBRATION KNOB: TRIGGER_ANGLE_BTDC is the ASSUMED crank angle (BTDC) of the
 * landmark edge. It is NOT yet calibrated. A jittery low-battery strobe read on
 * 2026-07-31 showed ~30 deg actual advance while intending 15 -> true landmark is
 * ~345 BTDC, so this likely wants to become ~345 once a clean (charged-battery)
 * strobe reading is taken. Because BOTH advances below derive from it, fixing this
 * one number corrects the cranking AND running spark angle together. */
#define TRIGGER_ANGLE_BTDC   330      // assumed landmark-edge angle BTDC (uncalibrated)
#define ADVANCE_BTDC         15       // running advance (fixed for now)

/* Cranking retard: below CRANK_RPM, fire at a near-TDC advance instead of the
 * running curve. Starting doesn't need precise timing, and the once-per-rev sync
 * is jittery at cranking (long prediction horizon + real torque-pulse speed
 * variation); firing near TDC keeps every jittery cranking spark in a start-
 * friendly, kickback-safe zone. Above CRANK_RPM it switches to ADVANCE_BTDC. */
#define CRANK_ADVANCE_BTDC   5        // near-TDC advance used below CRANK_RPM
#define CRANK_RPM            500      // crank/run switchover speed

/* SAFETY: hard ceiling on spark advance. Normal operation (5/15 deg) never reaches
 * it; it only bites when a wrong/lagging model would schedule a far-advanced spark
 * (e.g. a hard deceleration the ÷32 frequency loop hasn't tracked yet, which would
 * otherwise fire well BTDC into a kickback). The clamp is computed from the ACTUAL
 * measured rev time, not the model, so it holds even when the model is wrong. */
#define MAX_ADVANCE_BTDC     25       // never fire more advanced than this (deg BTDC)

#define AFTER_EDGE_RUN       (TRIGGER_ANGLE_BTDC - ADVANCE_BTDC)        // edge->spark, running
#define AFTER_EDGE_CRANK     (TRIGGER_ANGLE_BTDC - CRANK_ADVANCE_BTDC)  // edge->spark, cranking

#define DWELL_US             3000UL   // D514A ~3 ms
#define MAX_DWELL_US         5000UL   // watchdog: never charge longer than this
#define STALL_US             3000000UL // no edge this long -> force safe
#define CAPTURE_RISING       1        // match conditioner output polarity

/* ---- LANDMARK classifier (unchanged from v4) ----
 * refBig is a decaying peak-tracker of the raw rising-to-rising interval; a landmark
 * is any interval > LM_NUM/LM_DEN (0.6) of refBig. Rise is capped to +25%/edge so a
 * single irregular rev can't spike the threshold, and only intervals < PERIOD_MAX
 * may raise it (an idle gap or capture glitch must not poison it). */
#define LM_NUM              10        // landmark if interval*LM_NUM > refBig*LM_DEN
#define LM_DEN               7        //   i.e. interval > 0.7 * refBig
/* 2026-08-03: raised 0.6 -> 0.8 -> 0.9. At 0.6, refBig tracking the true ~90ms
 * landmark admitted anything over ~54ms — and on some channels the REMAINDER of the
 * revolution survives as one gap that long, giving TWO landmarks per rev. Acquisition
 * then locks at HALF the true period (measured: 81ms and 85ms against a true 162ms),
 * and n-rounding sustains the false lock indefinitely — a real landmark 161ms later is
 * counted as n=2, confirming the wrong period instead of correcting it. The 0.6*P
 * refractory cannot help because the false lock happens during ACQUISITION, before any
 * model exists. Channel-dependent (burst structure differs), which is why wiring
 * changes — twisting, separating, swapping conditioners — never touched it.
 * 0.8 fixed cyl 2 outright (730 rpm -> 373, 0/57) and moved cyl 3 from always-wrong
 * to a 50/50 coin flip per acquisition. 0.9 fixed cyl 3 but began REJECTING genuine
 * landmarks on cyls 1 and 2 (FIRED count 57 -> 24 and 57 -> 40, with a doubled
 * period appearing on cyl 1). No single threshold suits all three channels, because
 * the margin between "reject the false landmark" and "keep the true one" depends on
 * each channel's burst structure. Settled at 0.8 — where cyls 1 and 2 are spotless —
 * and the residual half-lock is caught by HALFLOCK_STREAK below instead.
 *
 * 2026-08-03 (later): RELAXED 0.8 -> 0.7 once the cold-acquisition guard existed. The
 * two threshold errors are NOT symmetric:
 *   too LOW  -> extra landmarks -> false lock at half  -> caught by the guard AND by
 *               HALFLOCK_STREAK. Two layers of protection.
 *   too HIGH -> genuine landmarks rejected -> missed sparks -> caught by NOTHING.
 * So bias LOW deliberately: it moves toward the protected failure mode and away from
 * the unprotected one. 0.8 sat closer to the rejection cliff than necessary (0.9 was
 * already dropping FIRED counts 57->24), and a rewire that shifts burst structure could
 * push it over with no mechanism to notice. */

/* ---- half-lock detector ----
 * If acquisition locks on HALF the true period, every real landmark then arrives at
 * n=2 and the n-rounding CONFIRMS the wrong period instead of correcting it — a
 * stable false lock (measured: 81ms and 85ms models against a true 162ms). A genuine
 * missed landmark also gives n=2, but only occasionally; a half-lock gives an
 * unbroken run. So: after this many CONSECUTIVE n==2 landmarks, double the model
 * period and re-lock. Detection + correction only — nothing in the firing path
 * changes, and if it never trips the behaviour is identical to plain 0.8. */
#define HALFLOCK_STREAK      4        // consecutive n==2 landmarks before doubling
#define REFBIG_DECAY_SHIFT   6        // refBig -= refBig>>6 on a non-peak edge

/* ---- cold-acquisition plausibility guard ----
 * After a genuine STALL (STALL_US with no edges, so the engine really stopped), the
 * next lock can only be at CRANKING speed — a starter physically cannot spin this
 * engine at 600rpm (fastest ever recorded here: 437 on a fresh battery, so ~37%
 * margin). So reject an implausibly short acquisition period outright and
 * keep looking. This is a PHYSICAL constraint, not a tuned threshold, so it adds no
 * fragility: it would have caught every false half-lock seen on 2026-08-03 (81/85ms
 * models = 700-740rpm) at the moment of acquisition, instead of four revolutions
 * later via HALFLOCK.
 * Deliberately NOT applied to a RELOCK, which can legitimately happen at speed.
 * Tightened 800 -> 600 when the landmark threshold was relaxed to 0.7: with two
 * landmarks per rev the candidate periods are ~90ms and ~72ms (summing to the 162ms
 * revolution). At 800rpm the floor is 75ms, which rejects the 72 but ACCEPTS the 90.
 * At 600rpm the floor is 100ms and both are rejected — so the guard actually catches
 * what the relaxed threshold lets through. */
#define COLD_ACQ_MAX_RPM     600
#define COLD_ACQ_MIN_TICKS   US_TO_TICKS(60000000UL / COLD_ACQ_MAX_RPM)

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
#define CRANK_PERIOD_TICKS   US_TO_TICKS(60000000UL / CRANK_RPM)   // period at CRANK_RPM; slower
                                                                    // (larger period) -> cranking retard

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
#define STROBE_BOOT_TEST 0           // 1 = ~1s of blinks at boot to verify LED wiring without
                                     // cranking. KEEP 0 for any run near fuel: the blocking
                                     // delay is ~1s of dead ignition on every reset/brownout,
                                     // long enough to stall a running engine.

/* ---- EFI trigger output (D9 = PH6) ----
 * A once-per-rev-derived pulse train for Speeduino (fuel-only, see README "EFI").
 * Speeduino's Basic Distributor decoder sets triggerActualTeeth = nCylinders, so on
 * this 3-cylinder engine it expects THREE evenly-spaced pulses per crank revolution
 * (it models a distributor with one lobe per cylinder). Rather than wiring all three
 * ignition boards' outputs together through a diode-OR, ONE board synthesises all 3
 * pulses by subdividing its own PLL-tracked rev period -- Speeduino only counts
 * pulses, so it cannot tell the difference, and the sub-pulses come from a single
 * already-smoothed model rather than three independently-tracking boards.
 *
 * Emitted from loop(), NOT a timer compare: Timer5's compare units are reserved for
 * dwell/spark, and loop-polling jitter (a few us) is negligible against a rev period
 * of tens of ms. This output is fully decoupled from the coil path -- it never
 * touches COIL_*, OCR5A/B or the ignition watchdogs.
 *
 * FREE-RUNNING off the model (v2). The first version re-anchored the train to the raw
 * landmark edge every rev, which leaked raw-edge jitter straight back into the output --
 * the very thing the v5 PLL exists to reject. Bench log 2026-08-02_19.51.08 showed the
 * signature clearly: 16 of 18 long intervals immediately cancelled by a short one
 * (~+/-6ms, ~13 crank deg of displacement), plus a dropped pulse whose recovery gap
 * landed 3.5% off Speeduino's Medium trigger-filter reject threshold (50% of the
 * previous gap). So the train now only takes its PERIOD from the model and free-runs;
 * it is never re-anchored to an edge. Absolute phase is then free to drift slowly,
 * which is fine -- Basic Distributor only counts pulses, it does not use them for
 * ignition timing. Free-running also rides through a missed landmark instead of
 * leaving a gap.
 *
 * Wiring: D9 -> diode anode, cathode -> Speeduino trigger input (D19/CAS), with a
 * 10k pulldown on the Speeduino side. Speeduino trigger edge = RISING. */
#define EFI_DDR    DDRH
#define EFI_PORT   PORTH
#define EFI_BIT    PH6
#define EFI_HIGH() (EFI_PORT |=  _BV(EFI_BIT))
#define EFI_LOW()  (EFI_PORT &= ~_BV(EFI_BIT))
#define EFI_PULSES_PER_REV  3        // = nCylinders, what Basic Distributor expects
#define EFI_PULSE_US        500UL    // rising edge is what's timed; width just needs to
                                     // be comfortably detectable. 500us stays <25% duty
                                     // even at the PERIOD_MIN_TICKS rpm ceiling.

/* ---- cylinder ID jumpers (D10 = PB4, D11 = PB5) ----
 * Identifies which cylinder this board serves, WITHOUT breaking the byte-identical
 * firmware property: all three boards run the same image and read their identity from
 * the harness, which is exactly where the 120 deg cylinder phasing already lives. A
 * compile-time #define would give three different binaries and lose the "pre-flashed
 * spare drops into any position" guarantee.
 *
 * Jumper the pin to GND to assert it; internal pull-ups mean open = not asserted.
 *   cyl 1 : both open
 *   cyl 2 : D10 -> GND
 *   cyl 3 : D11 -> GND
 *   both grounded = 0 (unlabelled) -- reported as CYL=? rather than guessing.
 *
 * Read ONCE at boot into cylId; nothing in the ignition path consults it. It exists so
 * telemetry logs are self-identifying, since COM port numbers renumber between sessions
 * and are useless for identity when logging several boards at once. */
#define CYLID_A_PIN   10      // PB4
#define CYLID_B_PIN   11      // PB5
uint8_t cylId = 0;            // 1/2/3, or 0 if unlabelled

static uint8_t readCylId(void){
  pinMode(CYLID_A_PIN, INPUT_PULLUP);
  pinMode(CYLID_B_PIN, INPUT_PULLUP);
  delayMicroseconds(50);                       // let the pull-ups settle
  bool a = (digitalRead(CYLID_A_PIN) == LOW);  // asserted = jumpered to GND
  bool b = (digitalRead(CYLID_B_PIN) == LOW);
  if (!a && !b) return 1;
  if ( a && !b) return 2;
  if (!a &&  b) return 3;
  return 0;                                    // both grounded: not a valid label
}

/* ---- shared state ---- */
volatile uint32_t timerHigh      = 0;
volatile uint32_t lastCaptureExt = 0;   // 32-bit extended tick of the previous edge (any)
volatile uint32_t refBig         = 0;   // decaying peak of raw interval, for landmark test
volatile uint32_t phaseExt       = 0;   // MODEL: extended tick of the last model landmark
volatile uint32_t phasePeriod    = 0;   // MODEL: rev period estimate (ticks)
volatile uint32_t prevPeriod     = 0;   // acquisition-only: previous landmark period
volatile uint8_t  nTwoStreak     = 0;   // consecutive n==2 landmarks (half-lock detector)
volatile bool     coldAcquire    = true;  // next lock follows a real stop -> cranking speed only
volatile bool     synced         = false;
volatile bool     coilCharging   = false;
volatile uint32_t coilHighMicros = 0;
volatile uint32_t lastEdgeMicros = 0;
volatile bool     strobeActive   = false;   // strobe LED currently lit
volatile uint32_t strobeHighMicros = 0;      // when it was lit, to time its width in loop()

/* EFI pulse train. The schedule (below) is written by the capture ISR and consumed by
 * loop(), so it needs atomic read-modify-write there. */
volatile bool     efiTrainRunning    = false; // free-running once the model has locked
volatile uint32_t efiNextPulseMicros = 0;   // when the next one is due
volatile uint32_t efiPulseSpacingUs  = 0;   // model rev period / EFI_PULSES_PER_REV
/* These two are touched ONLY by loop(), so they need no volatile/atomic guard. */
bool     efiPulseActive     = false;        // EFI pin currently HIGH
uint32_t efiPulseHighMicros = 0;            // when it went HIGH, to time its width

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
#define DBG_HALFLOCK  9   // half-lock detected -> model period doubled
#define DBG_IMPLAUS  10   // cold acquisition rejected: period implies > COLD_ACQ_MAX_RPM
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
    // Cold-acquisition guard: coming out of a real stop, only cranking speed is possible.
    if (coldAcquire && period < COLD_ACQ_MIN_TICKS){
      prevPeriod = 0;
      dbgEvent=DBG_IMPLAUS; dbgVal=period; dbgRefBig=refBig; dbgSeq++;
      return;
    }
    bool agrees = prevPeriod &&
                  (period * SYNC_AGREE_NUM > prevPeriod * SYNC_AGREE_DEN) &&
                  (period * SYNC_AGREE_DEN < prevPeriod * SYNC_AGREE_NUM);
    prevPeriod = period;
    if (agrees){
      phasePeriod = period; synced = true; coldAcquire = false;
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

  // Half-lock detector: a sustained run of n==2 means the model period is half the
  // real one. Double it and re-lock rather than letting n-rounding keep confirming
  // the error. An isolated n==2 (a genuinely missed landmark) resets the streak.
  if (n == 2) {
    if (++nTwoStreak >= HALFLOCK_STREAK) {
      uint32_t dbl = phasePeriod << 1;
      if (dbl <= PERIOD_MAX_TICKS) {
        phasePeriod = dbl;
        phaseExt    = capExt;          // re-anchor here; next rev re-establishes phase
        nTwoStreak  = 0;
        dbgEvent=DBG_HALFLOCK; dbgVal=delta; dbgPeriod=phasePeriod; dbgRefBig=refBig; dbgSeq++;
        return;                        // skip this rev's spark; model just moved
      }
      nTwoStreak = 0;
    }
  } else {
    nTwoStreak = 0;
  }

  uint32_t predicted = phaseExt + n * phasePeriod;
  int32_t  residual  = (int32_t)(capExt - predicted);   // signed phase error, |.|<0.5*P

  // PI update of the model (gentle: heavily attenuates hops, tracks real speed slowly).
  int32_t np = (int32_t)phasePeriod + (residual >> PLL_KF_SHIFT);
  if (np < (int32_t)PERIOD_MIN_TICKS) np = PERIOD_MIN_TICKS;
  if (np > (int32_t)PERIOD_MAX_TICKS) np = PERIOD_MAX_TICKS;
  phasePeriod = (uint32_t)np;
  phaseExt    = predicted + (residual >> PLL_KP_SHIFT);

  // EFI train: take only the PERIOD from the model. Deliberately does NOT re-anchor
  // efiNextPulseMicros to this edge -- that is what injected raw-edge jitter in v1.
  // The train is started once, on the first locked landmark, and free-runs thereafter.
  efiPulseSpacingUs = (phasePeriod * US_PER_TICK) / EFI_PULSES_PER_REV;
  if (!efiTrainRunning){
    efiNextPulseMicros = micros();
    efiTrainRunning    = true;
  }

  // Schedule this rev's spark off the MODEL anchor, not the raw (jittery) edge.
  // Cranking retard: below CRANK_RPM (period longer than CRANK_PERIOD_TICKS) fire at
  // the near-TDC crank advance; above it, the running advance.
  uint16_t afterEdge = (phasePeriod > CRANK_PERIOD_TICKS) ? AFTER_EDGE_CRANK : AFTER_EDGE_RUN;
  uint32_t fracTicks = phasePeriod * afterEdge / 360UL;

  // SAFETY max-advance clamp: floor fracTicks so the spark can't fire more advanced
  // than MAX_ADVANCE_BTDC. Reference is the ACTUAL measured rev time (delta/n, n>=1),
  // NOT the model, so a wrong/lagging model can't defeat it. Larger fracTicks = later
  // = more retarded = the safe direction; we only ever push retard here, never advance.
  uint32_t minFrac = (delta / n) * (uint32_t)(TRIGGER_ANGLE_BTDC - MAX_ADVANCE_BTDC) / 360UL;
  if (fracTicks < minFrac) fracTicks = minFrac;

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
  cylId = readCylId();
  Serial.print(F("one_cyl_ignition LANDMARK-PLL experiment build  CYL="));
  if (cylId) Serial.println(cylId); else Serial.println('?');
  Serial.print(F("timing: TRIGGER_ANGLE_BTDC=")); Serial.print(TRIGGER_ANGLE_BTDC);
  Serial.print(F(" run_adv=")); Serial.print(ADVANCE_BTDC);
  Serial.print(F(" crank_adv=")); Serial.print(CRANK_ADVANCE_BTDC);
  Serial.print(F(" below_rpm=")); Serial.println(CRANK_RPM);

  COIL_DDR |= _BV(COIL_BIT);
  COIL_LOW();
  STROBE_DDR |= STROBE_MASK;
  STROBE_LOW();
  EFI_DDR |= _BV(EFI_BIT);
  EFI_LOW();
#if STROBE_BOOT_TEST
  // Strobe/wiring self-test: 6 clearly-visible blinks at boot, so the LED wiring can be
  // verified WITHOUT cranking. DISABLED by default -- this blocks ~1s, which is ~1s of
  // dead ignition on every reset (would stall a running engine). Enable only for bench
  // wiring bring-up (STROBE_BOOT_TEST 1). The real strobe is only 1ms and near-invisible
  // to the eye anyway -- during cranking, watch the marks it lights, not the LED.
  for (uint8_t i=0; i<6; i++){ STROBE_HIGH(); delay(90); STROBE_LOW(); delay(90); }
#endif

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

  Serial.print(F("cyl="));
  if (cylId) Serial.print(cylId); else Serial.print('?');
  Serial.print(F(" seq=")); Serial.print(seq);
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
    case DBG_HALFLOCK:  Serial.print(F("HALFLOCK ")); break;
    case DBG_IMPLAUS:   Serial.print(F("IMPLAUS  ")); break;
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

  // EFI trigger train: emit a pulse when one is due, then end it a fixed width later.
  // The due-check and the schedule advance are one atomic RMW so a landmark ISR landing
  // mid-check can't have its spacing update clobbered by a stale write-back.
  bool efiDue = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    if (efiTrainRunning && (int32_t)(now - efiNextPulseMicros) >= 0){
      efiDue = true;
      // Catch-up guard: if we're more than a full spacing late (a long ISR burst, or the
      // model period just shrank sharply), re-base instead of rapid-firing the backlog --
      // a burst of closely-spaced pulses is exactly what Speeduino's trigger filter
      // rejects, and it would read as a false rpm spike.
      if ((int32_t)(now - efiNextPulseMicros) > (int32_t)efiPulseSpacingUs){
        efiNextPulseMicros = now + efiPulseSpacingUs;
      } else {
        efiNextPulseMicros += efiPulseSpacingUs;
      }
    }
  }
  if (efiDue){
    EFI_HIGH();
    efiPulseActive = true; efiPulseHighMicros = now;
  }
  if (efiPulseActive && (int32_t)(now - efiPulseHighMicros) > (int32_t)EFI_PULSE_US){
    EFI_LOW();
    efiPulseActive = false;
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
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; synced=false; prevPeriod=0;
                                       efiTrainRunning=false; coldAcquire=true; }
    EFI_LOW();                       // don't strand the EFI pin HIGH mid-pulse
    efiPulseActive = false;
    if (!stallLatched && Serial.availableForWrite() >= (SERIAL_TX_BUFFER_SIZE - 1)){
      Serial.print(F("WATCHDOG: STALL gap_us=")); Serial.println((int32_t)(now - lastEdge));
      stallLatched = true;
    }
  } else {
    stallLatched = false;
  }

  printDebug();
}
