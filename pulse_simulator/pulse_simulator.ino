/* pulse_simulator.ino — 3-channel crank/pulse-coil simulator for bench testing
 * one_cyl_ignition boards without running the engine.
 *
 * Emulates the CONDITIONER OUTPUT (clean digital edges), not the raw analog
 * VR waveform. Feeds directly into each ignition board's ICP5 (pin 48).
 *
 * Per revolution, each of the 3 channels produces two rising-edge pulses:
 *   leading edge (the real trigger)  ->  trailing edge (the twin to blank)
 * spaced TWIN_GAP_DEG apart, exactly like a real VR sensor's double
 * zero-crossing per magnet pass. Channels are offset 120 deg apart to model
 * one crank with three sensor positions.
 *
 * WIRING: sim pin -> ignition board pin 48 (ICP5), one wire per board.
 *         Common ground between simulator and every ignition board is required.
 *
 * SERIAL (115200): c=crank rpm, i=idle rpm, s=stop, j=toggle jitter,
 *                   +/- = nudge target rpm 50, p = print status
 */

#define NUM_CYL 3
const uint8_t pulsePin[NUM_CYL]     = {2, 3, 4};
const float   phaseOffsetDeg[NUM_CYL] = {0.0, 120.0, 240.0};

// Heartbeat LED: toggles once per revolution, independent of the pulse
// outputs above, so it's actually visible (unlike the 200us edge pulses).
// Blink rate = rpm/120 Hz. Driven on both the Mega's built-in LED (pin 13,
// no wiring needed) and an external pin for a bigger/brighter bench LED.
const uint8_t statusLedPinBuiltin  = 13;
const uint8_t statusLedPinExternal = 6;
bool          statusLedState       = false;

/* ---- tunables ---- */
float         twinGapDeg    = 30.0;   // trailing twin offset from leading edge
unsigned long pulseWidthUs  = 200;    // width of each simulated edge pulse

unsigned int  crankRpm      = 250;    // typical starter cranking speed
unsigned int  idleRpm       = 800;
float         rpmRampPerSec = 200.0;  // rpm/sec when ramping toward target

bool          jitterEnabled = true;
float         jitterPercent = 8.0;    // +/- % random speed variation per rev

/* ---- runtime state ---- */
float         targetRpm  = crankRpm;
float         currentRpm = crankRpm;
bool          running    = true;

unsigned long currentPeriodUs;
unsigned long revStartUs;
unsigned long lastRampUs;
unsigned long pulseEndTime[NUM_CYL] = {0, 0, 0};

struct Edge { unsigned long timeUs; uint8_t cyl; };
Edge   events[NUM_CYL * 2];
uint8_t nextEventIdx = 0;

void computePeriod(){
  float dt = (micros() - lastRampUs) / 1000000.0;
  lastRampUs = micros();
  if (currentRpm < targetRpm) currentRpm = min((float)targetRpm, currentRpm + rpmRampPerSec * dt);
  else if (currentRpm > targetRpm) currentRpm = max((float)targetRpm, currentRpm - rpmRampPerSec * dt);

  float effectiveRpm = currentRpm;
  if (jitterEnabled && effectiveRpm > 1.0){
    float jitter = 1.0 + (random(-1000, 1001) / 1000.0) * (jitterPercent / 100.0);
    effectiveRpm *= jitter;
  }
  if (effectiveRpm < 1.0) effectiveRpm = 1.0;
  currentPeriodUs = (unsigned long)(60000000.0 / effectiveRpm);
}

void scheduleRevolution(){
  computePeriod();
  for (uint8_t i = 0; i < NUM_CYL; i++){
    unsigned long lead    = revStartUs + (unsigned long)(phaseOffsetDeg[i] / 360.0 * currentPeriodUs);
    unsigned long trail   = revStartUs + (unsigned long)((phaseOffsetDeg[i] + twinGapDeg) / 360.0 * currentPeriodUs);
    events[i * 2]         = {lead, i};
    events[i * 2 + 1]     = {trail, i};
  }

  statusLedState = !statusLedState;
  digitalWrite(statusLedPinBuiltin, statusLedState);
  digitalWrite(statusLedPinExternal, statusLedState);
  // simple insertion sort, 6 elements
  for (uint8_t i = 1; i < NUM_CYL * 2; i++){
    Edge key = events[i];
    int8_t j = i - 1;
    while (j >= 0 && events[j].timeUs > key.timeUs){ events[j+1] = events[j]; j--; }
    events[j+1] = key;
  }
  nextEventIdx = 0;
}

void setup(){
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_CYL; i++){
    pinMode(pulsePin[i], OUTPUT);
    digitalWrite(pulsePin[i], LOW);
  }
  pinMode(statusLedPinBuiltin, OUTPUT);
  pinMode(statusLedPinExternal, OUTPUT);
  digitalWrite(statusLedPinBuiltin, LOW);
  digitalWrite(statusLedPinExternal, LOW);
  randomSeed(analogRead(A0));
  revStartUs = micros();
  lastRampUs = micros();
  scheduleRevolution();

  Serial.println(F("pulse_simulator ready"));
  Serial.println(F("c=crank i=idle s=stop j=jitter +/- nudge p=status"));
}

void handleSerial(){
  while (Serial.available()){
    switch (Serial.read()){
      case 'c': running = true;  targetRpm = crankRpm; break;
      case 'i': running = true;  targetRpm = idleRpm;  break;
      case 's': running = false; targetRpm = 0; currentRpm = 0;
                for (uint8_t i = 0; i < NUM_CYL; i++){ digitalWrite(pulsePin[i], LOW); pulseEndTime[i]=0; }
                statusLedState = false;
                digitalWrite(statusLedPinBuiltin, LOW);
                digitalWrite(statusLedPinExternal, LOW);
                break;
      case 'j': jitterEnabled = !jitterEnabled;
                Serial.print(F("jitter ")); Serial.println(jitterEnabled ? F("ON") : F("OFF"));
                break;
      case '+': targetRpm += 50; break;
      case '-': targetRpm = max(0.0f, targetRpm - 50); break;
      case 'p':
        Serial.print(F("target=")); Serial.print(targetRpm);
        Serial.print(F(" current=")); Serial.print(currentRpm);
        Serial.print(F(" periodUs=")); Serial.print(currentPeriodUs);
        Serial.print(F(" jitter=")); Serial.println(jitterEnabled ? F("ON") : F("OFF"));
        break;
    }
  }
}

void loop(){
  handleSerial();
  if (!running) return;

  unsigned long now = micros();

  for (uint8_t i = 0; i < NUM_CYL; i++){
    if (pulseEndTime[i] != 0 && (long)(now - pulseEndTime[i]) >= 0){
      digitalWrite(pulsePin[i], LOW);
      pulseEndTime[i] = 0;
    }
  }

  while (nextEventIdx < NUM_CYL * 2 && (long)(now - events[nextEventIdx].timeUs) >= 0){
    uint8_t cyl = events[nextEventIdx].cyl;
    digitalWrite(pulsePin[cyl], HIGH);
    pulseEndTime[cyl] = now + pulseWidthUs;
    nextEventIdx++;
  }

  if (nextEventIdx >= NUM_CYL * 2){
    revStartUs = events[0].timeUs;   // anchor to cylinder-1 leading edge, avoids drift
    scheduleRevolution();
  }
}
