/* opto_probe -- characterise the optocoupler output from an ignition coil tap.
 * =============================================================================
 *
 * PURPOSE
 *
 * Before trusting a coil-primary tap as Speeduino's crank trigger, we need to know
 * three things that only measurement can answer:
 *
 *   1. PULSES PER REVOLUTION -- tap one coil and you get 1/rev; tap all three OR'd
 *      and you get 3/rev. Speeduino's Basic Distributor expects 3/rev (= nCylinders).
 *      This sketch prints implied rpm for 1, 3 and 6 pulses/rev so you can compare
 *      against a known rpm and see which assumption matches.
 *
 *   2. PULSE WIDTH -- a CDI discharge is only tens of microseconds. Speeduino's crank
 *      input has its own conditioning and may filter a very short pulse out entirely.
 *      If the active width here is only a few us, the pulse needs stretching.
 *
 *   3. CONSISTENCY -- min/max interval vs the mean shows jitter, dropped pulses
 *      (interval ~2x) and spurious extra edges (interval very short). That is the
 *      whole reason for preferring this over the VR pulser, so it is worth proving.
 *
 * It also prints the IDLE LEVEL, which tells you the board's output polarity: most
 * PC817 modules pull OUT low when the input is energised, so idle is HIGH and the
 * spark event is a FALLING edge. Speeduino is currently set to TrigEdge = RISING, so
 * that would need changing.
 *
 * WIRING  (ESP32)
 *
 *   opto board VCC    -> ESP 3V3      <-- NOT 5V. ESP32 GPIOs are not 5V tolerant.
 *   opto board GND    -> ESP GND
 *   opto board OUT    -> PULSE_PIN
 *   opto board INPUT+ -> coil primary through 2 x 15k in series
 *   opto board INPUT- -> engine ground
 *                        1N4148 across INPUT+/INPUT-, band toward INPUT+
 *
 * The input side is galvanically isolated from the ESP, which is the point: the only
 * thing between 300V and this board is light.
 *
 * NOTE ON FILTERING: this sketch deliberately does NOT debounce or filter. We want to
 * see the signal as it really is, including any glitches. Cleaning it up is a decision
 * for later, once we know what is actually there.
 */

#define PULSE_PIN     4          // any free GPIO; avoid strapping pins (0, 2, 12, 15)
#define REPORT_MS     500
#define USE_PULLUP    1          // 1 = enable internal pull-up. Harmless if the board
                                 // already has one; essential if its OUT is bare
                                 // open-collector, which would otherwise float.

/* ---- ISR-shared state ----
 * Accumulated in the ISR rather than buffered, so there is no chance of overrunning a
 * ring buffer during a noise burst -- which is exactly when we most want valid numbers. */
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t lastEdgeUs   = 0;
volatile uint32_t lastRiseUs   = 0;
volatile bool     haveRise     = false;

volatile uint32_t risePeriodSum = 0;   // between consecutive RISING edges
volatile uint32_t risePeriodMin = 0xFFFFFFFF;
volatile uint32_t risePeriodMax = 0;
volatile uint32_t riseCount     = 0;

volatile uint32_t highSum = 0, highMin = 0xFFFFFFFF, highMax = 0, highCount = 0;
volatile uint32_t lowSum  = 0, lowMin  = 0xFFFFFFFF, lowMax  = 0, lowCount  = 0;

volatile uint32_t edgeTotal = 0;

void IRAM_ATTR onEdge(){
  uint32_t now = micros();
  int lvl = digitalRead(PULSE_PIN);

  portENTER_CRITICAL_ISR(&mux);
  edgeTotal++;

  if (lastEdgeUs){
    uint32_t w = now - lastEdgeUs;
    // The level we just read is the NEW level, so the span that just ended was the
    // opposite one.
    if (lvl){                       // now high -> the span that ended was LOW
      lowSum += w; lowCount++;
      if (w < lowMin) lowMin = w;
      if (w > lowMax) lowMax = w;
    } else {                        // now low -> the span that ended was HIGH
      highSum += w; highCount++;
      if (w < highMin) highMin = w;
      if (w > highMax) highMax = w;
    }
  }
  lastEdgeUs = now;

  if (lvl){                         // RISING edge: measure period between them
    if (haveRise){
      uint32_t p = now - lastRiseUs;
      risePeriodSum += p; riseCount++;
      if (p < risePeriodMin) risePeriodMin = p;
      if (p > risePeriodMax) risePeriodMax = p;
    }
    lastRiseUs = now;
    haveRise   = true;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void setup(){
  Serial.begin(115200);
  delay(200);
  pinMode(PULSE_PIN, USE_PULLUP ? INPUT_PULLUP : INPUT);
  Serial.println();
  Serial.println(F("opto_probe -- coil-tap signal characterisation"));
  Serial.print  (F("pin=")); Serial.print(PULSE_PIN);
  Serial.print  (F("  pullup=")); Serial.print(USE_PULLUP ? "on" : "off");
  Serial.print  (F("  idle level at boot=")); Serial.println(digitalRead(PULSE_PIN));
  Serial.println(F("(idle HIGH => active-low board => spark is a FALLING edge)"));
  Serial.println();
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), onEdge, CHANGE);
}

void loop(){
  static uint32_t lastReport = 0;
  if (millis() - lastReport < REPORT_MS) return;
  lastReport = millis();

  uint32_t rs, rmin, rmax, rc, hs, hmin, hmax, hc, ls, lmin, lmax, lc, et;
  portENTER_CRITICAL(&mux);
  rs = risePeriodSum; rmin = risePeriodMin; rmax = risePeriodMax; rc = riseCount;
  hs = highSum; hmin = highMin; hmax = highMax; hc = highCount;
  ls = lowSum;  lmin = lowMin;  lmax = lowMax;  lc = lowCount;
  et = edgeTotal;
  risePeriodSum = 0; risePeriodMin = 0xFFFFFFFF; risePeriodMax = 0; riseCount = 0;
  highSum = 0; highMin = 0xFFFFFFFF; highMax = 0; highCount = 0;
  lowSum  = 0; lowMin  = 0xFFFFFFFF; lowMax  = 0; lowCount  = 0;
  edgeTotal = 0;
  portEXIT_CRITICAL(&mux);

  int lvl = digitalRead(PULSE_PIN);

  if (et == 0){
    Serial.print(F("t=")); Serial.print(millis());
    Serial.print(F("  idle, level=")); Serial.println(lvl);
    return;
  }

  Serial.print(F("t=")); Serial.print(millis());
  Serial.print(F(" edges=")); Serial.print(et);
  Serial.print(F(" lvl=")); Serial.print(lvl);

  if (rc){
    uint32_t mean = rs / rc;
    Serial.print(F("  period_us mean=")); Serial.print(mean);
    Serial.print(F(" min=")); Serial.print(rmin);
    Serial.print(F(" max=")); Serial.print(rmax);
    // Spread relative to the mean is the honest jitter number: a dropped pulse shows
    // as max ~= 2x mean, a spurious edge as min ~= small.
    Serial.print(F(" spread=")); Serial.print((int32_t)(rmax - rmin));

    if (mean){
      uint32_t f60 = 60000000UL / mean;       // rpm if 1 pulse per rev
      Serial.print(F("  rpm@1/rev=")); Serial.print(f60);
      Serial.print(F(" @3/rev=")); Serial.print(f60 / 3);
      Serial.print(F(" @6/rev=")); Serial.print(f60 / 6);
    }
  }

  // Which of these is the ACTIVE span depends on polarity; print both and let the
  // idle level above decide. The short one is the pulse.
  if (hc){
    Serial.print(F("  high_us mean=")); Serial.print(hs / hc);
    Serial.print(F("/min=")); Serial.print(hmin);
    Serial.print(F("/max=")); Serial.print(hmax);
  }
  if (lc){
    Serial.print(F("  low_us mean=")); Serial.print(ls / lc);
    Serial.print(F("/min=")); Serial.print(lmin);
    Serial.print(F("/max=")); Serial.print(lmax);
  }
  Serial.println();
}
