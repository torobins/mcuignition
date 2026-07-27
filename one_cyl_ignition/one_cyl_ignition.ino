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
 *
 * For live bench diagnostics (serial telemetry of every capture-ISR decision),
 * flash one_cyl_ignition_debug/one_cyl_ignition_debug.ino instead - same logic,
 * plus non-blocking Serial output. This file has no Serial overhead.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>

/* ---- watchdog: recovers a hung loop() (e.g. coil stuck charging forever) ----
 * MUST disable the WDT this early (.init3, before any C runtime init/bss zeroing)
 * or a WDT-triggered reset can trap the board in a boot loop: the stock Mega
 * bootloader doesn't clear MCUSR/disable the WDT on entry, so if the WDT is
 * still counting down from before reset, it can fire again mid-bootloader,
 * before this sketch ever gets a chance to run and take control of it. */
void wdt_early_disable(void) __attribute__((naked, used, section(".init3")));
void wdt_early_disable(void){
  MCUSR = 0;
  wdt_disable();
}

/* ---- config ---- */
#define TRIGGER_ANGLE_BTDC   330      // leading edge 30 ATDC = 330 BTDC of next TDC
#define ADVANCE_BTDC         15       // fixed advance (stock idle, all cyls). Try 10 for first crank.
#define AFTER_EDGE_DEG       (TRIGGER_ANGLE_BTDC - ADVANCE_BTDC)  // edge->spark = 315

#define DWELL_US             3000UL   // D514A ~3 ms
#define MAX_DWELL_US         5000UL   // watchdog: never charge longer than this
#define STALL_US             3000000UL // no edge this long -> force safe (must exceed the
                                        // period of the slowest speed we want to run at:
                                        // PERIOD_MAX_TICKS below implies an ~54.5rpm floor,
                                        // whose period is 1,100,000us, so this needs real
                                        // margin above that or the watchdog force-drops
                                        // sync between every single legitimate revolution.
                                        // Not currently load-bearing either way - REJECTED
                                        // already refreshes lastEdgeMicros on every capture,
                                        // even a too-slow one, so STALL only ever trips on
                                        // genuine prolonged silence - but keeping real margin
                                        // here costs nothing and avoids relying on that.)
#define CAPTURE_RISING       1        // match conditioner output polarity
#define RATEJUMP_ESCAPE_COUNT 3       // consecutive rate-gate rejections before giving up on
                                       // lastPeriod and rebuilding sync from scratch - see the
                                       // escape hatch comment in the rate-of-change gate below

/* Two consecutive intervals must agree within +/-25% before sync is declared.
 * An L->T->L alternation can never satisfy this (the twin gap and the real
 * period differ by ~12x), so the twin can no longer establish lastPeriod. */
#define SYNC_AGREE_NUM       4        // agreement band: interval*4 > ref*3  AND
#define SYNC_AGREE_DEN       3        //                 interval*3 < ref*4

/* Rate-of-change bounds, asymmetric on purpose. A dropped edge always makes
 * the interval LONGER (~2x), never shorter, so the upper bound is tightened
 * to 1.5x to put clear daylight between the accept band and that signature -
 * real deceleration won't reach 1.5x in a single revolution. The lower bound
 * stays loose at 0.5x because a starter spinning up genuinely can halve its
 * period rev-to-rev during the first few revolutions, and rejecting that
 * would fight cranking. 0.5x also keeps the lower bound aligned with the
 * dynamic blanking window (lastPeriod/2) so the same edge can't pass one
 * and fail the other. */
#define RATE_MAX_NUM         3        // reject if interval > lastPeriod * 3/2
#define RATE_MAX_DEN         2
#define RATE_MIN_NUM         1        // reject if interval < lastPeriod * 1/2
#define RATE_MIN_DEN         2

/* ---- timebase (/256 @ 16 MHz -> 16 us/tick) ----
 * Was /64 (4us/tick); moved to /256 because the edge-to-spark delay (fracTicks)
 * has to fit in the 16-bit OCR5A/OCR5B compare registers to be scheduled
 * correctly (see the fracTicks > 0xFFFF check below). At 4us/tick that capped
 * real minimum speed at ~200rpm; at 16us/tick the same 16-bit register covers
 * 4x the time range, dropping the floor to ~50rpm. Cost is coarser angular
 * resolution - 16us/tick is 0.67 degrees at 7000rpm and 0.08 degrees at idle,
 * both fine for this application. The input capture noise canceller (ICNC5)
 * is unaffected: it samples at the system clock rate, not the prescaled timer
 * clock, so its ~250ns response time doesn't change with this setting. */
#define US_PER_TICK          16UL
#define US_TO_TICKS(us)      ((uint32_t)(us)/US_PER_TICK)
#define DWELL_TICKS          US_TO_TICKS(DWELL_US)
#define PERIOD_MIN_TICKS     US_TO_TICKS(6000UL)      // ~10000 rpm ceiling
#define PERIOD_MAX_TICKS     US_TO_TICKS(1100000UL)   // ~54.5rpm floor, chosen with margin
                                                       // below the true fracTicks-fits-in-16-bits
                                                       // ceiling (~50rpm) so this measurement gate
                                                       // and the scheduling gate line up - previously
                                                       // (at /64) this floor was 80rpm while the
                                                       // scheduling floor was ~200rpm, so slow
                                                       // cranking between them measured fine but
                                                       // could never fire (sat in UNSCHED forever).
                                                       // Note: 68750 ticks exceeds a uint16_t on
                                                       // purpose - this is only ever compared as a
                                                       // uint32_t (`interval` is uint32_t), so don't
                                                       // "fix" this to fit 16 bits later.
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
volatile uint8_t  rateJumpStreak = 0;   // consecutive rate-gate rejections; see the
                                        // escape hatch in the rate-of-change gate below
volatile uint32_t prevInterval   = 0;   // last accepted raw interval. Doubles as the
                                        // blanking reference during acquisition (when
                                        // lastPeriod isn't trustworthy yet) and as the
                                        // agreement reference for declaring sync.
volatile bool     twinSeenThisRev = true;  // see the missing-twin gate below

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
  // 1/2 of the reference period) to the previous kept edge. Anchor stays on
  // the kept edge.
  //
  // The reference is lastPeriod once synced, but prevInterval (the last raw
  // accepted interval) during acquisition. Without that second case the
  // window collapses to the FIXED_BLANK_TICKS floor while unsynced, and no
  // fixed floor can work for both ends of the range - a 300rpm twin gap
  // (~16.7ms) is longer than a 6000rpm real period (10ms). Using the running
  // interval instead means one L->T->L pass is enough to widen the window
  // past the twin, so the twin never gets to establish a period.
  uint32_t blank = FIXED_BLANK_TICKS;
  uint32_t ref   = synced ? lastPeriod : prevInterval;
  if (ref){
    uint32_t dyn = ref / 2UL;
    if (dyn > blank) blank = dyn;
  }
  if (interval < blank){
    twinSeenThisRev = true;   // the expected trailing twin showed up on time
    return;   // trailing twin -> discard
  }

  // Plausibility gate: reject noise / impossible speeds (absolute bounds).
  if (interval < PERIOD_MIN_TICKS || interval > PERIOD_MAX_TICKS){
    lastCaptureExt=capExt; synced=false; prevInterval=0; twinSeenThisRev=true;
    lastEdgeMicros=micros();
    return;
  }

  // Missing-twin gate: if we're synced and expected the trailing twin before
  // this candidate leading edge but never saw it blanked, something is wrong
  // even though the interval itself may look numerically plausible. This is
  // the confirmed signature of a single dropped leading edge whose twin
  // still arrives: the twin lands at (period + twinGap) from the true
  // previous edge - only ~5-10% off a normal period at typical twin angles,
  // easily inside the rate gate's 1.5x bound - and the following real edge
  // then measures (period - twinGap) from THAT false anchor, also inside
  // bounds. Both sail through unchallenged and fire at a wrong angle unless
  // caught here. Reproduced on the bench with the simulator's dropped-edge
  // injection: two consecutive wrong-angle sparks and a MAX_DWELL trip,
  // with the rate gate never triggering at all. Shares the same streak/
  // escape mechanics as the rate gate below, since both represent "this
  // reference can no longer be trusted."
  if (synced && !twinSeenThisRev){
    if (++rateJumpStreak < RATEJUMP_ESCAPE_COUNT){
      lastCaptureExt=capExt; synced=false; prevInterval=interval; twinSeenThisRev=true;
      lastEdgeMicros=micros();
      return;
    }
    rateJumpStreak = 0;
    lastCaptureExt=capExt; lastPeriod=interval; prevInterval=interval;
    synced=true; twinSeenThisRev=false; lastEdgeMicros=micros();
    return;
  }

  // Rate-of-change gate: once synced, real rev-to-rev variation won't exceed
  // roughly 1.5x on the high side (see RATE_MAX_NUM/DEN above) or 0.5x on the
  // low side, so a dropped edge (~2x lastPeriod) or a spurious extra edge
  // (~0.5x or less) gets caught immediately instead of being accepted as a
  // legitimate speed and used to schedule a wrong-angle spark.
  //
  // Escape hatch (RATEJUMP_ESCAPE_COUNT consecutive rejections): without this,
  // a bad lastPeriod can lock out spark PERMANENTLY, not just cost a couple of
  // revolutions - confirmed by bench test, not theoretical. The acquisition
  // agreement check above (see the "!synced" block below) closes the specific
  // path that caused this (the twin mis-syncing as a period), but the escape
  // hatch stays as a backstop against a poisoned reference arriving by some
  // other path. After a few consecutive rejections, stop trusting lastPeriod
  // and rebuild sync from this edge instead, exactly like a fresh start.
  if (synced && ((interval * RATE_MAX_DEN > lastPeriod * RATE_MAX_NUM) ||
                 (interval * RATE_MIN_DEN < lastPeriod * RATE_MIN_NUM))){
    if (++rateJumpStreak < RATEJUMP_ESCAPE_COUNT){
      lastCaptureExt=capExt; synced=false; prevInterval=interval; twinSeenThisRev=true;
      lastEdgeMicros=micros();
      return;
    }
    rateJumpStreak = 0;
    lastCaptureExt=capExt; lastPeriod=interval; prevInterval=interval;
    synced=true; twinSeenThisRev=false; lastEdgeMicros=micros();
    return;
  }

  // Sync acquisition: require two consecutive intervals that agree within the
  // SYNC_AGREE band before declaring sync. A single interval is not enough -
  // it could be a leading-to-twin or twin-to-leading gap just as easily as a
  // real revolution, and accepting one of those poisons lastPeriod (bug #8).
  // Only the true L->L period repeats consistently, so agreement identifies it.
  //
  // IMPORTANT: do NOT reset rateJumpStreak in this branch. The escape hatch
  // above only ever accumulates because acquisition states sit between
  // consecutive rate-gate rejections. Clearing the streak here looks like
  // harmless hygiene and would silently cap it at 1, restoring the permanent
  // spark lockout that the escape hatch exists to break.
  if (!synced){
    bool agrees = prevInterval &&
                  (interval * SYNC_AGREE_NUM > prevInterval * SYNC_AGREE_DEN) &&
                  (interval * SYNC_AGREE_DEN < prevInterval * SYNC_AGREE_NUM);
    lastCaptureExt=capExt; prevInterval=interval;
    lastEdgeMicros=micros();
    if (agrees){
      lastPeriod=interval; synced=true; twinSeenThisRev=false;
      return;                    // period established, fire from next edge
    }
    return;                      // candidate only, not yet trusted
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
  // With AFTER_EDGE_DEG=315 this caps real minimum speed at ~50rpm at /256 -
  // a genuine hardware ceiling of one 16-bit output-compare register, not a
  // tunable threshold. Below that, safely decline to fire rather than mis-schedule.
  uint32_t period    = interval;
  uint32_t fracTicks = period * AFTER_EDGE_DEG / 360UL;
  if (fracTicks > 0xFFFFUL){
    lastCaptureExt=capExt; lastPeriod=period; prevInterval=period; synced=true; twinSeenThisRev=false;
    lastEdgeMicros=micros();
    return;
  }
  uint16_t sparkAt   = capRaw + (uint16_t)fracTicks;

  // Write the new compare value(s) BEFORE clearing the flags/enabling the
  // interrupt, not after. If the flags were cleared first while OCR5A still
  // held the STALE value from last revolution, and TCNT5 happened to pass
  // through that stale value in the gap before the new value lands, OCF5A
  // would set again - then enabling OCIE5A right after fires COMPA
  // immediately on the stale match, dropping the coil low and eating this
  // revolution's real spark. Safe to reorder this way because fracTicks is
  // always at least DWELL_TICKS/etc out, so the new match can't have already
  // passed by the time we finish writing it.
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

  rateJumpStreak = 0;
  lastCaptureExt=capExt; lastPeriod=period; prevInterval=period;
  synced=true; twinSeenThisRev=false; lastEdgeMicros=micros();
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
  TCCR5B |= _BV(CS52);               // /256
  TCNT5=0;
  TIMSK5 = _BV(ICIE5) | _BV(TOIE5);
  sei();

  lastEdgeMicros = micros();

  // loop() normally completes in low microseconds (just a couple of atomic
  // reads/comparisons, no blocking calls) - 15ms is enormous margin over that
  // while still forcing recovery quickly if it ever wedges, instead of leaving
  // the coil in whatever state it was in (e.g. stuck charging) indefinitely.
  wdt_enable(WDTO_15MS);
}

/* ---- safety watchdogs ---- */
void loop(){
  wdt_reset();
  uint32_t now = micros();

  bool charging; uint32_t highAt;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ charging=coilCharging; highAt=coilHighMicros; }
  if (charging && (int32_t)(now - highAt) > (int32_t)MAX_DWELL_US){
    COIL_LOW();
    TIMSK5 &= ~_BV(OCIE5A);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; }
  }

  uint32_t lastEdge;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ lastEdge=lastEdgeMicros; }
  if ((int32_t)(now - lastEdge) > (int32_t)STALL_US){
    COIL_LOW();
    TIMSK5 &= ~(_BV(OCIE5A) | _BV(OCIE5B));
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; synced=false; prevInterval=0; twinSeenThisRev=true; }
  }
}
