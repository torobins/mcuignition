/* efi_trigger -- EFI trigger generator for Speeduino. FUEL ONLY. Drives NO coil.
 * =============================================================================
 *
 * Architecture this belongs to (2026-08-21): the STOCK Yamaha CDI does ignition. This
 * board exists solely to give Speeduino a crank trigger, taken from ONE pulser coil via
 * a VR conditioner into D48, and emitted on D9.
 *
 * WHY THIS IS A SEPARATE, MUCH SIMPLER SKETCH
 *
 * one_cyl_ignition_landmark.ino is careful because it fires a SPARK: a wrong crank angle
 * means kickback and broken starters, so it carries a max-advance clamp, a cold-acquisition
 * plausibility guard, a half-lock detector, dwell watchdogs and a timing strobe. None of
 * that applies here, because this board drives no coil.
 *
 * Speeduino's Basic Distributor decoder only COUNTS pulses. Absolute phase is irrelevant
 * to it. So the requirement collapses to three things:
 *
 *   1. EFI_PULSES_PER_REV pulses per crank revolution  (= nCylinders = 3)
 *   2. roughly the right RATE -- a few percent of rpm error is invisible in the fuel calc
 *   3. spaced evenly enough that Speeduino's trigger filter does not reject them
 *
 * Rate accuracy still matters, because injection FREQUENCY scales with rpm: a model locked
 * on half the true period means double the fuel. But that is now a "runs badly" failure,
 * not a "breaks parts" one -- which is exactly what lets this sketch drop the guards that
 * kept locking the ignition build out.
 *
 * WHAT WENT WRONG WITH THE IGNITION BUILD HERE, AND WHY THIS FIXES IT
 *
 * The ignition build refuses to acquire above COLD_ACQ_MAX_RPM (600) after a stop, because
 * a starter physically cannot spin the engine faster than that -- a sound guard for spark.
 * But on the stock CDI the engine lights off on residual fuel almost immediately, so it
 * passed 600rpm before acquisition finished. Every landmark thereafter looked implausible:
 * 1057 perfectly good landmarks at 500-3000rpm discarded across a 20-second run, Speeduino
 * never saw a trigger, and it injected nothing the whole time.
 * (bench_logs/speedup_COM13_2026-08-21_09.45.06.txt)
 *
 * This sketch locks at whatever speed it finds, because it has no reason not to.
 *
 * ONE MECHANISM INSTEAD OF THREE
 *
 * The ignition build grew three separate correctors -- acquisition agreement, HALFLOCK
 * (model too short) and SPEEDUP (model too long). Here a single rule covers all of it:
 *
 *     a landmark period is ACCEPTED if it agrees with the current model;
 *     otherwise it becomes a CANDIDATE, and ADOPT_COUNT consecutive agreeing candidates
 *     replace the model outright.
 *
 * That one rule handles cold acquisition, a sub-multiple lock, a hard acceleration, and a
 * one-off glitch, without any of them needing to be named or detected separately.
 */

#include <avr/wdt.h>
#include <util/atomic.h>

/* ---- pins ----
 *   D48 = ICP5 (PL1)  input   pulser, via VR conditioner
 *   D9  = PH6         output  EFI trigger to Speeduino pin 19 (Crank/VR1), RISING edge,
 *                             with a 10k pulldown on the Speeduino side. The diode from
 *                             the original 3-board diode-OR plan is NOT needed: one board,
 *                             one source. */
#define EFI_DDR    DDRH
#define EFI_PORT   PORTH
#define EFI_BIT    PH6
#define EFI_HIGH() (EFI_PORT |=  _BV(EFI_BIT))
#define EFI_LOW()  (EFI_PORT &= ~_BV(EFI_BIT))

#define CAPTURE_RISING      1
#define EFI_PULSES_PER_REV  3        // = nCylinders, what Basic Distributor expects
#define EFI_PULSE_US        500UL    // rising edge is what Speeduino times; width only
                                     // needs to be comfortably detectable

/* ---- timebase: Timer5 /256 @ 16MHz -> 16us/tick ---- */
#define US_PER_TICK         16UL
#define US_TO_TICKS(us)     ((uint32_t)(us) / US_PER_TICK)

/* Plausible crank revolution period. The upper bound is deliberately tighter than the
 * ignition build's 1.1s: long coast-down revolutions were dragging refBig up to 336ms,
 * which then made every running-speed interval fail the landmark test. */
#define PERIOD_MIN_TICKS    US_TO_TICKS(6000UL)     // 10000 rpm ceiling
#define PERIOD_MAX_TICKS    US_TO_TICKS(500000UL)   // 120 rpm floor

/* ---- landmark classifier ----
 * The pulser produces several edges per revolution; the once-per-rev LANDMARK is the
 * longest gap. refBig is a decaying peak-tracker of the raw edge-to-edge interval, and a
 * landmark is any interval above LM_NUM/LM_DEN of it.
 *
 * The floor is not optional. In the ignition build a long noise burst decayed refBig below
 * 4 ticks, at which point BOTH shifts truncated to zero -- it could neither grow nor shrink
 * and froze at 32us permanently, making every edge look like a landmark. That bricked the
 * channel until reset. Flooring it also stops noise dragging the threshold down into the
 * range where noise itself qualifies. */
#define LM_NUM              10       // landmark if interval*LM_NUM > refBig*LM_DEN
#define LM_DEN               7       //   i.e. interval > 0.7 * refBig
#define REFBIG_DECAY_SHIFT   6       // refBig -= refBig>>6 on a non-peak edge
#define REFBIG_MIN_TICKS    (PERIOD_MIN_TICKS >> 1)

/* ---- the single accept/adopt rule ----
 * AGREE is +/-25%: p agrees with q if p > 0.75*q and p < 1.333*q. */
#define AGREE_NUM            4
#define AGREE_DEN            3
#define ADOPT_COUNT          2       // consecutive agreeing candidates that replace the model
#define TRACK_SHIFT          2       // accepted period: model += (p - model) >> 2
#define MAX_MISSED_REVS      4       // n-rounding: accept a landmark up to 4 model revs late

/* ---- liveness ----
 * The pulse train free-runs off the model, so without this it keeps feeding Speeduino a
 * perfectly steady train after the engine has stopped -- a phantom rpm, and fuel injected
 * into a dead engine.
 *
 * 6, not 3. Measured on a running engine at ~1800rpm, about a THIRD of landmarks are
 * missed, so 3 revolutions of silence is a routine occurrence rather than evidence the
 * engine stopped -- the train was being stopped and restarted constantly, and Speeduino
 * kept losing the trigger. Free-running exists precisely to ride through missed landmarks;
 * this must not defeat it. 6 revs is still only ~200ms at 1800rpm. */
#define LIVENESS_REVS        6UL
#define LIVENESS_MAX_US      500000UL

#define STATUS_INTERVAL_MS   250

/* ---- ISR state ---- */
volatile uint16_t timerHigh        = 0;
volatile uint32_t lastCaptureExt   = 0;   // previous edge (any)
volatile uint32_t lastLandmarkExt  = 0;   // previous landmark
volatile uint32_t refBig           = 0;
volatile uint32_t revPeriod        = 0;   // model crank period, ticks. 0 = unlocked
volatile uint32_t candPeriod       = 0;
volatile uint8_t  candCount        = 0;

volatile uint32_t efiPulseSpacingUs   = 0;
volatile uint32_t efiNextPulseMicros  = 0;
volatile uint32_t efiLastLandmarkMicros = 0;
volatile bool     efiTrainRunning     = false;

/* counters, for the status line */
volatile uint16_t cntEdges = 0, cntLandmarks = 0, cntAccept = 0, cntAdopt = 0, cntReject = 0;
volatile uint16_t cntMissed = 0;          // revolutions covered by n-rounding (n-1 each)

/* loop()-only */
bool     efiPulseActive    = false;
uint32_t efiPulseHighMicros = 0;

ISR(TIMER5_OVF_vect){ timerHigh++; }

static inline uint32_t extendCapture(uint16_t icr){
  uint16_t hi = timerHigh;
  // A capture taken just before an overflow that has already been serviced reads as a
  // large ICR against an incremented high word; step the high word back for that case.
  if ((TIFR5 & _BV(TOV5)) && (icr < 0x8000U)) hi++;
  return ((uint32_t)hi << 16) | icr;
}

static inline bool agrees(uint32_t p, uint32_t q){
  return q && (p * AGREE_NUM > q * AGREE_DEN) && (p * AGREE_DEN < q * AGREE_NUM);
}

ISR(TIMER5_CAPT_vect){
  uint32_t capExt   = extendCapture(ICR5);
  uint32_t interval = capExt - lastCaptureExt;
  lastCaptureExt    = capExt;
  cntEdges++;

  /* --- classify: is this the once-per-rev landmark? --- */
  bool isLandmark = (refBig > 0) && (interval * LM_NUM > refBig * LM_DEN);
  if (interval > refBig){
    if (interval < PERIOD_MAX_TICKS){
      uint32_t rise = refBig >> 2;
      if (rise == 0) rise = 1;              // >>2 truncates to 0 below 4 ticks
      uint32_t capped = refBig + rise;      // cap the rise so one odd gap can't spike it
      refBig = (refBig == 0 || interval < capped) ? interval : capped;
    }
  } else if (refBig > REFBIG_MIN_TICKS){
    uint32_t dec = refBig >> REFBIG_DECAY_SHIFT;
    if (dec == 0) dec = 1;                  // >>6 truncates to 0 below 64 ticks
    refBig = (refBig - dec > REFBIG_MIN_TICKS) ? (refBig - dec) : REFBIG_MIN_TICKS;
  }
  if (!isLandmark) return;                  // burst edge -> discard

  cntLandmarks++;
  uint32_t p = capExt - lastLandmarkExt;
  lastLandmarkExt = capExt;

  if (p < PERIOD_MIN_TICKS || p > PERIOD_MAX_TICKS){
    cntReject++;
    candCount = 0;
    return;
  }

  /* --- the single rule, with n-rounding ---
   * A MISSED landmark puts the next one at 2x the model period (3x for two misses). At
   * ~1800rpm about a third of landmarks are missed, so this is the common case, not an
   * edge case. Without n-rounding those arrivals disagree with the model, become
   * candidates, and two in a row would ADOPT double the period -- halving reported rpm and
   * therefore halving the fuel. So accept p if it is close to an integer number of model
   * revolutions, and divide it back down. */
  uint8_t n = 0;
  if (revPeriod){
    for (uint8_t k = 1; k <= MAX_MISSED_REVS; k++){
      if (agrees(p, revPeriod * k)){ n = k; break; }
    }
  }
  if (n){
    // tracks a real speed change smoothly, ignores per-rev jitter
    uint32_t effective = p / n;
    revPeriod += ((int32_t)effective - (int32_t)revPeriod) >> TRACK_SHIFT;
    candCount  = 0;
    cntAccept++;
    if (n > 1) cntMissed += (n - 1);
  } else {
    // Disagrees with the model -- or there is no model yet. Only a REPEATED disagreement
    // is real: that covers cold acquisition, a sub-multiple lock and a hard acceleration
    // alike, while a one-off glitch is simply dropped.
    if (agrees(p, candPeriod)) candCount++;
    else                     { candCount = 1; }
    candPeriod = p;
    if (candCount >= ADOPT_COUNT){
      revPeriod = p;
      candCount = 0;
      cntAdopt++;
    } else {
      return;                               // nothing to drive the train with yet
    }
  }

  /* --- feed the EFI train: PERIOD only, never re-anchored to this edge ---
   * Re-anchoring every revolution leaks raw-edge jitter straight into Speeduino, which is
   * what the original version did wrong. Absolute phase is free to drift; Basic
   * Distributor does not care. Free-running also rides through a missed landmark instead
   * of leaving a gap. */
  efiPulseSpacingUs     = (revPeriod * US_PER_TICK) / EFI_PULSES_PER_REV;
  efiLastLandmarkMicros = micros();
  if (!efiTrainRunning){
    efiNextPulseMicros = micros();
    efiTrainRunning    = true;
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println(F("efi_trigger -- FUEL ONLY, drives no coil"));
  Serial.print(F("pulses/rev=")); Serial.print(EFI_PULSES_PER_REV);
  Serial.print(F("  D9 -> Speeduino pin 19, RISING, 10k pulldown"));
  Serial.println();

  EFI_DDR |= _BV(EFI_BIT);
  EFI_LOW();

  DDRL &= ~_BV(PL1);                 // ICP5 (pin 48) input

  cli();
  TCCR5A = 0; TCCR5B = 0;
#if CAPTURE_RISING
  TCCR5B |= _BV(ICES5);
#endif
  TCCR5B |= _BV(ICNC5);              // noise canceller
  TCCR5B |= _BV(CS52);               // /256
  TCNT5   = 0;
  TIMSK5  = _BV(ICIE5) | _BV(TOIE5);
  sei();

  efiLastLandmarkMicros = micros();
  wdt_enable(WDTO_15MS);
}

void loop(){
  wdt_reset();
  uint32_t now = micros();

  /* --- emit the pulse train ---
   * The due-check and the schedule advance are one atomic RMW, so a landmark ISR landing
   * mid-check cannot have its spacing update clobbered by a stale write-back. */
  bool efiDue = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    if (efiTrainRunning && (int32_t)(now - efiNextPulseMicros) >= 0){
      efiDue = true;
      // Catch-up guard: more than a full spacing late (a long ISR burst, or the model
      // period just shrank sharply) -> re-base rather than rapid-firing the backlog. A
      // burst of closely-spaced pulses is exactly what Speeduino's trigger filter rejects,
      // and it would read as a false rpm spike.
      if ((int32_t)(now - efiNextPulseMicros) > (int32_t)efiPulseSpacingUs)
        efiNextPulseMicros = now + efiPulseSpacingUs;
      else
        efiNextPulseMicros += efiPulseSpacingUs;
    }
  }
  if (efiDue){ EFI_HIGH(); efiPulseActive = true; efiPulseHighMicros = now; }
  if (efiPulseActive && (int32_t)(now - efiPulseHighMicros) > (int32_t)EFI_PULSE_US){
    EFI_LOW();
    efiPulseActive = false;
  }

  /* --- liveness: stop the train when the engine stops --- */
  {
    bool     running; uint32_t lastLm, periodTicks;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      running     = efiTrainRunning;
      lastLm      = efiLastLandmarkMicros;
      periodTicks = revPeriod;
    }
    if (running){
      uint32_t limitUs = periodTicks * US_PER_TICK * LIVENESS_REVS;
      if (limitUs > LIVENESS_MAX_US) limitUs = LIVENESS_MAX_US;
      if ((int32_t)(now - lastLm) > (int32_t)limitUs){
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ efiTrainRunning = false; revPeriod = 0;
                                           candCount = 0; }
        EFI_LOW();                   // never strand the pin HIGH mid-pulse
        efiPulseActive = false;
        Serial.print(F("STOP  no landmark for us=")); Serial.println((int32_t)(now - lastLm));
      }
    }
  }

  /* --- status line --- */
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus >= STATUS_INTERVAL_MS){
    lastStatus = millis();
    uint32_t per, rb; uint16_t e, l, a, ad, rj, ms; bool run;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      per = revPeriod; rb = refBig; run = efiTrainRunning;
      e = cntEdges; l = cntLandmarks; a = cntAccept; ad = cntAdopt; rj = cntReject;
      ms = cntMissed;
      cntEdges = cntLandmarks = cntAccept = cntAdopt = cntReject = cntMissed = 0;
    }
    if (e || run){
      uint32_t perUs = per * US_PER_TICK;
      Serial.print(F("t=")); Serial.print(millis());
      Serial.print(F(" rpm="));
      if (perUs) Serial.print(60000000UL / perUs); else Serial.print(0);
      Serial.print(F(" rev_us=")); Serial.print(perUs);
      Serial.print(F(" refBig_us=")); Serial.print(rb * US_PER_TICK);
      Serial.print(F(" run=")); Serial.print(run ? 1 : 0);
      Serial.print(F(" edges=")); Serial.print(e);
      Serial.print(F(" lm=")); Serial.print(l);
      Serial.print(F(" acc=")); Serial.print(a);
      Serial.print(F(" adopt=")); Serial.print(ad);
      Serial.print(F(" missed=")); Serial.print(ms);
      Serial.print(F(" rej=")); Serial.println(rj);
    }
  }
}
