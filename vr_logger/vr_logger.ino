/* vr_logger.ino — VR conditioner output logger for bench characterization.
 *
 * Flash to any spare Mega2560. Wire the real VR conditioner's output to
 * ICP5 (pin 48) — the exact same pin, timer, and noise-canceler config
 * one_cyl_ignition.ino uses — so what gets logged here is exactly what the
 * ignition firmware would see, not an idealized version of it. Turn the
 * engine over with a drill and watch the serial log to characterize the
 * real twin-pulse gap, pulse width, jitter, and any noise/bounce, before
 * tuning FIXED_BLANK_TICKS / TWIN_GAP_DEG to match (see README Roadmap:
 * "Build/verify the VR conditioner circuit against a real sensor").
 *
 * This sketch performs NO ignition logic whatsoever — pure capture and
 * log. Nothing here drives a coil pin. Safe to run with a real conditioner
 * connected and nothing else.
 *
 * Captures BOTH edges (toggles ICES5 after every capture), so each logged
 * line is either:
 *   EDGE=RISE  gap_us=...    <- low-time since the previous (falling) edge
 *   EDGE=FALL  pulse_us=...  <- high-time since the previous (rising) edge
 * A clean VR-conditioner revolution should look like:
 *   RISE(lead) FALL(lead, short pulse_us) RISE(twin, short gap_us)
 *   FALL(twin, short pulse_us) RISE(next lead, long gap_us) ...
 * i.e. two short pulses per revolution separated by a short gap (the twin),
 * then one long gap back to the next leading edge. That long/short gap
 * split is what TWIN_GAP_DEG and FIXED_BLANK_TICKS need to be tuned against.
 *
 * Timer5 setup (prescale, noise canceler) intentionally mirrors
 * one_cyl_ignition.ino's production config exactly — see that file for the
 * reasoning. 'n' below lets you toggle the noise canceler live for
 * comparison, but it starts ON (matching production) by default.
 *
 * SERIAL (115200):
 *   p = print min/max/avg summary stats since last reset
 *   r = reset summary stats (does not clear the live per-edge log)
 *   n = toggle input-capture noise canceler (ICNC5) on/off
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/* ---- timebase: identical to one_cyl_ignition.ino (/256 @ 16MHz -> 16us/tick) ---- */
#define US_PER_TICK 16UL

/* ---- ring buffer: single producer (ISR) / single consumer (loop), no locks needed
 * beyond making sure a slot is fully written before publishing it via head. ---- */
#define BUF_SIZE 64   // must be power of two
struct Event { uint32_t intervalTicks; uint8_t dir; };  // dir: 1=RISE, 0=FALL
volatile Event   buf[BUF_SIZE];
volatile uint8_t head = 0, tail = 0;
volatile uint16_t dropped = 0;

volatile uint32_t timerHigh      = 0;
volatile uint32_t lastCaptureExt = 0;
volatile bool     haveLast       = false;

ISR(TIMER5_OVF_vect){ timerHigh++; }

// Same capture-vs-overflow race handling as one_cyl_ignition.ino.
static inline uint32_t extendCapture(uint16_t icr){
  uint32_t high = timerHigh;
  if ((TIFR5 & _BV(TOV5)) && icr < 0x8000U) high++;
  return (high << 16) | icr;
}

ISR(TIMER5_CAPT_vect){
  uint16_t capRaw = ICR5;
  uint32_t capExt  = extendCapture(capRaw);
  uint8_t  dir     = (TCCR5B & _BV(ICES5)) ? 1 : 0;  // edge we just captured
  TCCR5B ^= _BV(ICES5);                              // capture the opposite edge next

  if (haveLast){
    uint32_t interval = capExt - lastCaptureExt;
    uint8_t nextHead = (head + 1) & (BUF_SIZE - 1);
    if (nextHead != tail){
      buf[head].intervalTicks = interval;
      buf[head].dir = dir;
      head = nextHead;
    } else {
      dropped++;   // buffer full (loop() couldn't drain fast enough) - drop, keep going
    }
  }
  lastCaptureExt = capExt;
  haveLast = true;
}

/* ---- summary stats (updated in loop(), from drained events) ---- */
uint32_t pulseMinUs = 0xFFFFFFFFUL, pulseMaxUs = 0;
uint64_t pulseSumUs = 0; uint32_t pulseCount = 0;
uint32_t gapShortMinUs = 0xFFFFFFFFUL, gapShortMaxUs = 0;   // gaps < gapSplitUs
uint32_t gapLongMinUs  = 0xFFFFFFFFUL, gapLongMaxUs  = 0;   // gaps >= gapSplitUs
uint32_t gapSplitUs = 5000;   // rough short/long gap divider; adjust after first look at the log
uint32_t edgeCount = 0;

void resetStats(){
  pulseMinUs = 0xFFFFFFFFUL; pulseMaxUs = 0; pulseSumUs = 0; pulseCount = 0;
  gapShortMinUs = 0xFFFFFFFFUL; gapShortMaxUs = 0;
  gapLongMinUs  = 0xFFFFFFFFUL; gapLongMaxUs  = 0;
  edgeCount = 0; dropped = 0;
  Serial.println(F("stats reset"));
}

void printStats(){
  Serial.println(F("---- summary ----"));
  Serial.print(F("edges=")); Serial.print(edgeCount);
  Serial.print(F(" dropped=")); Serial.println(dropped);
  if (pulseCount){
    Serial.print(F("pulse_us  min=")); Serial.print(pulseMinUs);
    Serial.print(F(" max=")); Serial.print(pulseMaxUs);
    Serial.print(F(" avg=")); Serial.println((uint32_t)(pulseSumUs / pulseCount));
  } else {
    Serial.println(F("pulse_us  (no data yet)"));
  }
  if (gapShortMaxUs){
    Serial.print(F("gap_short_us (twin gap, <")); Serial.print(gapSplitUs);
    Serial.print(F(") min=")); Serial.print(gapShortMinUs);
    Serial.print(F(" max=")); Serial.println(gapShortMaxUs);
  }
  if (gapLongMaxUs){
    Serial.print(F("gap_long_us  (rev gap, >=")); Serial.print(gapSplitUs);
    Serial.print(F(") min=")); Serial.print(gapLongMinUs);
    Serial.print(F(" max=")); Serial.println(gapLongMaxUs);
  }
  Serial.println(F("-----------------"));
}

void handleSerial(){
  while (Serial.available()){
    switch (Serial.read()){
      case 'p': printStats(); break;
      case 'r': resetStats(); break;
      case 'n': {
        bool on = TCCR5B & _BV(ICNC5);
        if (on) TCCR5B &= ~_BV(ICNC5); else TCCR5B |= _BV(ICNC5);
        Serial.print(F("noise canceler ")); Serial.println(on ? F("OFF") : F("ON"));
        break;
      }
    }
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println(F("vr_logger — VR conditioner output capture, no ignition logic"));
  Serial.println(F("wire conditioner output -> pin 48 (ICP5), common ground required"));
  Serial.println(F("p=stats r=reset n=toggle noise canceler"));

  DDRL &= ~_BV(PL1);   // ICP5 (pin 48) input

  cli();
  TCCR5A = 0; TCCR5B = 0;
  TCCR5B |= _BV(ICES5);   // start by capturing rising, matches production polarity
  TCCR5B |= _BV(ICNC5);   // noise canceler ON by default, matches production
  TCCR5B |= _BV(CS52);    // /256 prescale, matches production
  TCNT5 = 0;
  TIMSK5 = _BV(ICIE5) | _BV(TOIE5);
  sei();
}

void loop(){
  handleSerial();

  while (tail != head){
    uint32_t us  = buf[tail].intervalTicks * US_PER_TICK;
    uint8_t  dir = buf[tail].dir;

    if (Serial.availableForWrite() < (SERIAL_TX_BUFFER_SIZE - 1)) break;  // don't block, retry next loop()
    tail = (tail + 1) & (BUF_SIZE - 1);

    edgeCount++;
    if (dir){
      Serial.print(F("EDGE=RISE gap_us="));   Serial.println(us);
      if (us < gapSplitUs){
        if (us < gapShortMinUs) gapShortMinUs = us;
        if (us > gapShortMaxUs) gapShortMaxUs = us;
      } else {
        if (us < gapLongMinUs) gapLongMinUs = us;
        if (us > gapLongMaxUs) gapLongMaxUs = us;
      }
    } else {
      Serial.print(F("EDGE=FALL pulse_us=")); Serial.println(us);
      if (us < pulseMinUs) pulseMinUs = us;
      if (us > pulseMaxUs) pulseMaxUs = us;
      pulseSumUs += us; pulseCount++;
    }
  }
}
