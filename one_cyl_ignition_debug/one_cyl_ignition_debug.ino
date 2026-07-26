/* one_cyl_ignition_debug.ino — bench-diagnostic build of one_cyl_ignition.ino.
 * Identical ignition logic and identical safety envelope (same WDT timeout) as
 * the production sketch, plus serial telemetry (115200 baud) so the capture
 * ISR's internal decisions can be observed live: every
 * BLANKED/REJECTED/RATEJUMP/SYNCFIRST/FIRED/UNSCHED event, plus watchdog
 * trips, are logged from loop() without ever blocking the ISR path.
 *
 * The telemetry print itself is genuinely non-blocking, not just "usually
 * fast": it checks Serial.availableForWrite() and skips an update rather than
 * spinning if the TX ring can't take a full line without the UART having to
 * catch up mid-write (the existing missed= counter makes a skipped update
 * visible rather than silent). This is what lets this build safely run the
 * same tight WDT timeout as production instead of needing a looser one to
 * tolerate blocking.
 *
 * Use this build when bringing up a new board, characterizing a real sensor's
 * twin-pulse gap, or diagnosing sync/timing issues on the bench (e.g. with
 * pulse_simulator). Flash the plain one_cyl_ignition.ino for actual engine use
 * - it has no Serial overhead, though this build's safety behavior is the same.
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
#include <avr/wdt.h>
#include <util/atomic.h>

/* ---- watchdog: recovers a hung loop() (e.g. coil stuck charging forever) ----
 * MUST disable the WDT this early (.init3, before any C runtime init/bss zeroing)
 * or a WDT-triggered reset can trap the board in a boot loop: the stock Mega
 * bootloader doesn't clear MCUSR/disable the WDT on entry, so if the WDT is
 * still counting down from before reset, it can fire again mid-bootloader,
 * before this sketch ever gets a chance to run and take control of it.
 *
 * Same WDTO_15MS as production - safe here too because printDebug() below is
 * genuinely non-blocking (see file header), not because a hang is somehow
 * less dangerous on this build. It isn't: this build gets used with a real
 * coil connected on the bench, and the WDT timeout is the effective worst-
 * case charge duration if loop() ever wedges while COIL_HIGH() is asserted
 * (MAX_DWELL_US's own software check also depends on loop() running, so it
 * doesn't help during a genuine hang). An earlier version of this file used
 * WDTO_250MS to tolerate blocking Serial calls - that was a real mistake,
 * not a reasonable tradeoff: ~83x the D514A's intended dwell is long enough
 * to risk damaging the coil or its driver, so the fix had to be making the
 * telemetry non-blocking, not loosening the watchdog to accommodate it. */
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

/* ---- debug telemetry (loop() prints, ISR only does cheap int copies) ---- */
#define DBG_NONE      0
#define DBG_BLANKED   1   // trailing twin discarded
#define DBG_REJECTED  2   // implausible interval
#define DBG_SYNCFIRST 3   // first good edge, period established, no fire yet
#define DBG_FIRED     4   // normal scheduled spark
#define DBG_UNSCHED   5   // period measured fine, but too slow to SCHEDULE (see below)
#define DBG_RATEJUMP  6   // rejected: interval outside ~0.5x-2x of lastPeriod (dropped/extra edge)
#define DBG_ESCAPE    7   // rate-gate escape hatch fired: lastPeriod was stuck wrong, rebuilt sync
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
  // 1/2 of last period) to the previous kept edge. Anchor stays on the kept edge.
  // (Was 3/8: raised to exactly match the rate-of-change gate's lower bound
  // below, so an edge in [3/8,1/2)x lastPeriod doesn't fall through blanking
  // only to get caught - and drop sync - by the rate gate instead. A silent
  // blank is the gentler outcome for the same signal. Margin against wrongly
  // blanking a genuine speed increase is unchanged in practice: that still
  // needs >2x acceleration in one revolution, the same assumption the rate
  // gate's upper bound already makes elsewhere.)
  uint32_t blank = FIXED_BLANK_TICKS;
  if (synced){
    uint32_t dyn = lastPeriod / 2UL;
    if (dyn > blank) blank = dyn;
  }
  if (interval < blank){
    dbgEvent=DBG_BLANKED; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
    return;   // trailing twin -> discard
  }

  // Plausibility gate: reject noise / impossible speeds (absolute bounds).
  if (interval < PERIOD_MIN_TICKS || interval > PERIOD_MAX_TICKS){
    lastCaptureExt=capExt; synced=false;
    lastEdgeMicros=micros();
    dbgEvent=DBG_REJECTED; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
    return;
  }

  // Rate-of-change gate: once synced, real rev-to-rev variation won't exceed
  // roughly 2x, so a dropped edge (~2x lastPeriod) or a spurious extra edge
  // (~0.5x or less) gets caught immediately instead of being accepted as a
  // legitimate speed and used to schedule a wrong-angle spark. (It would
  // eventually self-correct on the next capture anyway, since that recomputes
  // fresh from this rejection's own timestamp, but this avoids the one/two
  // wrong-angle sparks in between.)
  //
  // Escape hatch (RATEJUMP_ESCAPE_COUNT consecutive rejections): without this,
  // a bad lastPeriod can lock out spark PERMANENTLY, not just cost a couple of
  // revolutions - confirmed by bench test, not theoretical. If the twin ever
  // gets mistaken for a real edge right after a resync (FIXED_BLANK_TICKS is
  // a fixed absolute floor, but the twin's absolute gap grows at lower rpm -
  // reproduced live at 800rpm idle: 30+ seconds of zero FIRED events, stuck
  // alternating SYNCFIRST-on-the-twin then RATEJUMP-on-the-real-edge forever),
  // lastPeriod gets poisoned to the tiny twin-gap value. Every subsequent real
  // edge then looks like a >2x jump from that wrong reference and gets
  // rejected, which drops sync, and the very next capture (still the twin)
  // gets mis-synced the same way again - a stable trap with no way out, since
  // nothing ever gets a chance to relearn the true period. After a few
  // consecutive rejections, stop trusting lastPeriod and rebuild sync from
  // this edge instead, exactly like a fresh "!synced" start.
  if (synced && (interval > lastPeriod * 2UL || interval * 2UL < lastPeriod)){
    if (++rateJumpStreak < RATEJUMP_ESCAPE_COUNT){
      lastCaptureExt=capExt; synced=false;
      lastEdgeMicros=micros();
      dbgEvent=DBG_RATEJUMP; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
      return;
    }
    rateJumpStreak = 0;
    lastCaptureExt=capExt; lastPeriod=interval;
    synced=true; lastEdgeMicros=micros();
    dbgEvent=DBG_ESCAPE; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
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
  // With AFTER_EDGE_DEG=315 this caps real minimum speed at ~50rpm at /256 (was
  // ~200rpm at /64) - a genuine hardware ceiling of one 16-bit output-compare
  // register, not a tunable threshold. Confirmed by bench test: below the
  // ceiling this is what was causing MAX_DWELL trips (wrong dwell/spark gap
  // from the wrapped compare targets).
  uint32_t period    = interval;
  uint32_t fracTicks = period * AFTER_EDGE_DEG / 360UL;
  if (fracTicks > 0xFFFFUL){
    lastCaptureExt=capExt; lastPeriod=period; synced=true;
    lastEdgeMicros=micros();
    dbgEvent=DBG_UNSCHED; dbgInterval=interval; dbgBlank=blank; dbgSeq++;
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
  TCCR5B |= _BV(CS52);               // /256
  TCNT5=0;
  TIMSK5 = _BV(ICIE5) | _BV(TOIE5);
  sei();

  lastEdgeMicros = micros();

  wdt_enable(WDTO_15MS);
}

/* ---- debug telemetry printer (loop-side only, never blocks the ISR) ----
 * Genuinely non-blocking: the 64-byte hardware TX ring can't hold a full
 * line at once regardless (our longest line, a FIRED event, is ~130+ bytes),
 * so the only way to guarantee this call can't stall waiting on the UART is
 * to require the ring be completely empty before starting. That bounds
 * worst-case blocking to roughly (lineLen-64 bytes)/11520 bytes-per-sec =~6ms
 * even in the single worst case, instead of letting blocking compound
 * indefinitely under sustained high event rates. If there's no room, this
 * skips the update entirely (not just fields of it) - the existing missed=
 * counter, driven off the dbgSeq gap, makes a skip visible next time a line
 * does print rather than silently dropping it. */
uint8_t lastPrintedSeq = 0;
void printDebug(){
  uint8_t  seq, ev; uint32_t interval, blank, period, fracUs;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    seq=dbgSeq; ev=dbgEvent; interval=dbgInterval; blank=dbgBlank;
    period=dbgPeriod; fracUs=dbgFracUs;
  }
  if (seq == lastPrintedSeq) return;
  // SERIAL_TX_BUFFER_SIZE - 1, not SERIAL_TX_BUFFER_SIZE: this core's ring
  // buffer reserves one slot to distinguish empty from full, so a fully
  // empty buffer reports SIZE-1, never SIZE - checking for the wrong one
  // means this can never be true and nothing would ever print.
  if (Serial.availableForWrite() < (SERIAL_TX_BUFFER_SIZE - 1)) return;
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
    case DBG_RATEJUMP:   Serial.print(F("RATEJUMP ")); break;
    case DBG_ESCAPE:     Serial.print(F("ESCAPE   ")); break;
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
  wdt_reset();
  uint32_t now = micros();

  // The coil-forcing/state-clearing actions below are always unconditional -
  // only the diagnostic Serial.print calls are gated on buffer space, same
  // reasoning as printDebug() above. Never skip the safety action itself.
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
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ coilCharging=false; synced=false; }
    if (!stallLatched && Serial.availableForWrite() >= (SERIAL_TX_BUFFER_SIZE - 1)){
      Serial.print(F("WATCHDOG: STALL gap_us=")); Serial.println((int32_t)(now - lastEdge));
      stallLatched = true;
    }
  } else {
    stallLatched = false;
  }

  printDebug();
}
