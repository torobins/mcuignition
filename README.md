# MCU Ignition — 3-Cylinder Rotary Engine

Standalone electronic ignition for a 3-cylinder two-stroke engine, built from three identical Arduino Mega 2560 boards — one per cylinder. Each board reads its own VR (pulse coil) sensor, cleans up the trigger signal internally, and fires a smart ignition coil at a fixed advance angle. No shared "cleaner" board and no cross-wiring between cylinders — every board is a complete, self-sufficient ignition channel.

## Current status & next steps (2026-08-03)

**Where things stand — a clean pick-up point across sessions/machines.**

- **Ignition: two channels bench-proven and agreeing.** Landmark PLL decoder (branch
  `experiment/longest-pulse-landmark`, sketch `one_cyl_ignition_landmark/`). Calibrated
  (`TRIGGER_ANGLE_BTDC=330`), cranking retard verified, safety-reviewed. **Two boards on two
  channels now report the same engine speed (370 rpm) to within 1 rpm with zero impossible
  readings** — see "Simultaneous two-board capture". `master` still holds the older production
  sketch; the PLL needs porting there once proven on fuel.
- **Pulser mapping strobe-MEASURED (not inferred):** cyl 1 → **W/G** (60°), cyl 2 → **W/R** (180°),
  cyl 3 → **W/B** (300°). Each sensor sits 60° ahead of its own TDC, identically for all three —
  the byte-identical firmware property is now verified rather than assumed.
- **Root cause of a full afternoon of trigger scatter: an untwisted sensor lead.** Fixed by twisting
  all four pulser conductors. Not the grounding, not the air gap, not the damping resistor, not the
  max-advance clamp — all of which were suspected and eliminated. See "ROOT CAUSE".
- **EFI: fully validated and ready.** Trigger synthesised on ignition **D9** (3 pulses/rev), zero
  sync losses across every log for two days, injection commanded on all 3 channels, every analog
  input valid, six corrupt enrichment tables rebuilt, **Required Fuel corrected 10.8 → 9.0 ms** from
  real engine/injector specs and verified (PW **33.1 ms**, duty **21.0%**).
- **Engine: Yamaha 65U, 1176 cc.** Injectors Daytona 675, ~330 cc/min @ 3 bar, measured **10.5 Ω
  (High-Z, drives direct)**. **Compression 110–115 psi across all three** — modest but workable, and
  the ≤5 psi spread rules out a weak cylinder.
- **Carbs are OUT.** Throttle body mounts directly to the intake.

**Blocked on:** a **third VR conditioner channel** (in the mail — the existing board is 2-channel).

**Immediate next actions:**
1. **Do not attempt a start on two cylinders.** Speeduino injects on all three regardless of spark,
   so cylinder 3 would take full fuelling with no ignition — raw fuel into the crankcase and exhaust.
   Either wait for the third channel or disable injector 3 first.
2. **Twist channel 3's pulser leads before wiring it in.** Known failure mode on this engine now;
   thirty seconds to prevent, an afternoon to diagnose.
3. **Plumb the fuel system** — pump at/below tank outlet level (inline pumps push well, pull badly),
   filter, rail, regulator **3 bar** with its **vacuum port open to atmosphere** (no MAP compensation
   on Alpha-N), and a **return line**.
4. **Wire the fuel pump relay** off Speeduino **pin 45** for prime-and-cut-out behaviour.
5. **Spark test with plugs in and grounded**, under cranking compression — still not done, and a
   spark that jumps in open air can fail under cylinder pressure.
6. **Then attempt a start.** Flood clear armed at 75% TPS.

**Useful to acquire:** an **optical/laser tachometer**. Every rpm figure in this project is
decoder-derived; an independent reference that shares nothing with the ignition system would have
saved hours today.

**Expected first-start outcome:** fires, runs a few seconds, dies or runs rough — then two or three
rounds of VE trim. That is the normal path and a *good* result: it means spark, fuel and timing are
fundamentally right. Tune in the **VE table**, not Required Fuel, which is now a known-good physical
anchor.

**Key pointers:** ignition sketch `one_cyl_ignition_landmark/one_cyl_ignition_landmark.ino`; ignition
boards are CH340 clone Megas (COM9/COM10 on Windows, `/dev/ttyUSB*` on Linux); Speeduino is a genuine
Mega (COM4 / `/dev/ttyACM0`); flash cmd
`arduino-cli upload -p <port> --fqbn arduino:avr:mega:cpu=atmega2560 one_cyl_ignition_landmark`.
**Diagnostic of choice: the simultaneous two-board capture** (`tools/` pattern in bench_logs) — two
boards on the same crank share a true rpm, so a disagreement is proof rather than inference.

## Why the boards are identical

Each board only ever needs to know one thing: the angle from *its own* sensor edge to *its own* cylinder's TDC. Because the three pulse coils are mounted 120° apart on the crank, and the three TDCs are also 120° apart, the offset is the same for every cylinder — the 120° cancels out of the math. The firmware is therefore identical on all three boards; the only thing that makes "board 2" fire "cylinder 2" is that it's plugged into cylinder 2's sensor and cylinder 2's coil.

This means:
- All three boards run the exact same `.ino` — one build, flashed three times.
- A pre-flashed spare Mega can be swapped into any of the three positions.
- The 120° cylinder phasing lives entirely in the wiring harness, not in the code. **Do not cross the harness connectors between cylinders.**

### Pulser coil identification and positions — STROBE-MEASURED 2026-08-03

**This mapping was measured directly, not inferred.** Method: connect one pulser wire to a board,
crank, strobe the flywheel, and see which cylinder's TDC mark the spark lands on. Repeated for all
three wires. It exercises the whole chain (magnet, sensor, conditioner, firmware, timing) and
answers the only question that matters: *which cylinder does this wire time correctly for.*

| Cylinder | Pulser wire | Sensor position |
|---|---|---|
| 1 | **W/G** (white/green) | 60° |
| 2 | **W/R** (white/red) | 180° |
| 3 | **W/B** (white/black) | 300° |

**Wire each board to its cylinder's pulser AND that cylinder's coil.** Crossing them puts a cylinder
120° or 240° out.

**Reference frame:** degree wheel used as a fixed protractor, 0° = 12 o'clock, numbers increasing
clockwise, crank turning counter-clockwise, viewed from the flywheel side. Cylinder 1 is nearest
the flywheel. **TDC order is 1 → 2 → 3** (pencil-verified). The magnet's protractor reading
**increases** as the engine turns.

**The one relationship the design depends on:**

| Cyl | sensor | TDC | sensor − TDC |
|---|---|---|---|
| 1 | 60° | 0° | 60 |
| 2 | 180° | 120° | 60 |
| 3 | 300° | 240° | 60 |

Each cylinder's sensor sits **60° ahead of its own TDC**, identically for all three. That — sensors
120° apart, TDCs 120° apart, constant offset between them — is the entire basis of the
byte-identical firmware property, and it is now measured rather than assumed. All three cylinders
strobe correctly on the same build, which re-confirms `TRIGGER_ANGLE_BTDC = 330`.

> **Two traps that cost real time here, both worth avoiding on a re-check:**
> 1. **Wire naming.** The pulsers are **W/B, W/R, W/G**. The manual also carries a **B/W**
>    (black/white), but that is a CDI *output* to ignition coil 2 — a different wire. W/B vs B/W is
>    an easy transposition.
> 2. **Don't derive the mapping from magnet geometry.** An earlier attempt to predict it from
>    eyeballed magnet-edge positions produced a confident but wrong answer, because a sign error in
>    the rotation convention flips `sensor − TDC` into `sensor + TDC` and spreads the three offsets
>    120° apart. Strobe it instead; it is one crank per wire and admits no ambiguity.

## Hardware per channel

| Signal | Connection |
|---|---|
| VR pulse coil → conditioner output | ICP5 / pin 48 (Timer5 input capture) |
| Ignition coil (Dyna D514A smart coil) | Pin 5 / PE3 — HIGH = charging, LOW = fire |
| Pulldown | 10 kΩ, pin 5 to GND, at the pin (holds coil OFF through reset/brownout) |

### VR conditioner input network (recorded 2026-08-03)

Across the VR **+ and −** at the conditioner input, per channel:

| Component | Value | Purpose |
|---|---|---|
| Damping resistor | **470 Ω** | loads the coil to kill ringing / noise pickup |
| Filter cap | **10 nF** | shunts RF and high-frequency noise |

**What these do to the signal**, given the factory pulser specs (248-372 Ω coil resistance,
see `bench_logs/yamaha_stator_specs.md`):

- **The 470 Ω forms a divider against the coil's own ~310 Ω source impedance**, so only
  **~60%** of the open-circuit signal reaches the conditioner (56-65% across the coil
  tolerance range). At the manual's 3.2 V open-cranking figure that's roughly **1.9 V** at the
  input — and notably **heavier loading than the factory CDI**, which the manual shows leaving
  2.4 V at cranking.
- **The 10 nF against the ~187 Ω effective source** (coil ∥ damping) gives a corner around
  **85 kHz**. That's far above anything in the pulse waveform, so it filters RF without touching
  the edges. This part is doing its job cleanly.

**The trade-off lives entirely at cranking.** Pulser output swings ~9x across the rev range
(3.2 V open cranking → 21.1 V at 3500 rpm), so at 3500 rpm the 40% loss is irrelevant, but at
cranking it comes straight off an already-marginal 2-3 V. Since every trigger problem in this
project has appeared at cranking and none at speed, **the damping resistor is a prime suspect if a
channel proves marginal** — raising it (10 kΩ retains ~97%) would recover amplitude at the cost of
less damping. Do not change it without a before/after telemetry capture; the noise it suppresses is
what it was fitted for.

Each cylinder is a fully independent chain: **pulse coil → conditioner → Mega → smart coil**. The three chains share only power and ground. This gives fault isolation — one cylinder's sensor or board failing takes out only that cylinder, not the engine.

**Before deployment, set the BOD fuse to 4.3V via ISP** (see Roadmap) — stock Mega fuses ship with `BODLEVEL` at 2.7V, but an ATmega2560 at 16MHz is only in spec down to 4.5V, leaving a window where the MCU keeps executing instead of resetting cleanly. The pin-5 pulldown's entire safety argument assumes a supply sag produces a clean reset; at the stock fuse setting it doesn't.

### Optional bench LED indicator (no firmware change)

An LED + ~220-330 Ω series resistor tapped straight onto pin 5 (anode to pin 5, cathode to GND, in parallel with the existing 10 kΩ pulldown) blinks once per revolution whenever the board is actually issuing charge/fire commands — no code needed, since you're just observing the real coil-control signal. The 3 ms dwell window is long enough to read as a clear flash by eye (unlike the sub-millisecond pulse-coil edges on the simulator board), visible at both cranking and idle speed. Useful as a quick "is this board actually firing" check; disconnect before the real bench spark test so nothing extra is sharing the coil trigger line.

## Firmware

`one_cyl_ignition/one_cyl_ignition.ino` — flash this unmodified to all three boards.

`one_cyl_ignition_debug/one_cyl_ignition_debug.ino` — byte-for-byte the same ignition logic and the same safety envelope (same WDT timeout) as the production sketch, plus genuinely non-blocking serial telemetry (115200 baud, see bug #7 below) that logs every capture-ISR decision (`BLANKED`/`REJECTED`/`RATEJUMP`/`ESCAPE`/`SYNCFIRST`/`SYNCCAND`/`TWINMISS`/`FIRED`/`UNSCHED`) and watchdog trip. Use this for bench bring-up or diagnosing sync/timing issues; flash the plain (non-`_debug`) sketch for actual engine use, since it has zero Serial overhead.

Key parameters (top of the file):

| Parameter | Value | Meaning |
|---|---|---|
| `TRIGGER_ANGLE_BTDC` | 330° | Leading trigger edge, ~30° ATDC (provisional — verify with timing light) |
| `ADVANCE_BTDC` | 15° | Fixed spark advance for start/idle (all cylinders) |
| `AFTER_EDGE_DEG` | 315° | Derived: edge-to-spark angle |
| `DWELL_US` | 3000 µs | Coil dwell (D514A) |
| `MAX_DWELL_US` | 5000 µs | Dwell watchdog ceiling |
| `STALL_US` | 3,000,000 µs | No-edge timeout → force coil safe. Must stay comfortably above the period of the slowest speed the ignition is expected to run at (see `PERIOD_MAX_TICKS` below) or the watchdog force-drops sync between every legitimate slow revolution. Not currently load-bearing either way — `REJECTED` already refreshes the stall timestamp on every capture, even a too-slow one — but kept with real margin since it costs nothing. |
| `PERIOD_MAX_TICKS` | ~1,100,000 µs (~54.5 rpm) | Slowest period the capture ISR will accept as plausible. Chosen with margin below the true ~50rpm scheduling ceiling (see "Known hardware limitation" below) so the measurement gate and the scheduling gate line up. Safe at this speed only because capture timestamps are 32-bit extended (see below) — a bare 16-bit register can't represent periods this long. (This constant's raw tick value exceeds 16 bits on purpose — it's only ever compared as `uint32_t`.) |
| `FIXED_BLANK_TICKS` | ~3 ms floor | Startup blanking window before a period is learned; tune to measured magnet/twin-pulse width |
| `RATEJUMP_ESCAPE_COUNT` | 3 | Consecutive rate-gate/missing-twin rejections tolerated before giving up on `lastPeriod` and rebuilding sync from scratch. See bug #8 below — without this the rate gate can lock out spark permanently, not just cost a couple of revolutions. |
| `SYNC_AGREE_NUM`/`DEN` | 4/3 | Two consecutive raw intervals must agree within ±25% before sync is declared during acquisition. See bug #10 — a single interval isn't enough, since a leading-to-twin gap and a real revolution can both look "plausible" in isolation. |
| `RATE_MAX_NUM`/`DEN`, `RATE_MIN_NUM`/`DEN` | 3/2, 1/2 | Asymmetric rate-of-change bounds: reject above 1.5× `lastPeriod`, below 0.5×. See bug #11 for why the bounds aren't symmetric. |

Timebase is Timer5 at `/256` prescale (16 MHz → 16 µs/tick). The capture ISR keeps the leading edge of each pulse coil's signal and blanks the trailing-edge "twin" a VR sensor produces per revolution, using `max(fixed floor, 1/2 of the reference period)` — the reference is `lastPeriod` once synced, but the last raw accepted interval during acquisition (bug #10), since no single fixed floor can blank a twin correctly at both ends of the operating range. Sync is only declared once two consecutive raw intervals agree within `SYNC_AGREE_NUM`/`DEN` (bug #10) — a lone interval can't distinguish a real revolution from a leading-to-twin gap. Once synced, an asymmetric rate-of-change gate rejects anything outside `RATE_MIN`–`RATE_MAX` × the last measured period (bug #11), and a missing-twin gate separately rejects a candidate leading edge if the expected trailing twin was never seen blanked first (bug #12) — both share the same escape hatch (`RATEJUMP_ESCAPE_COUNT` consecutive rejections) so a bad reference period can't lock either gate shut forever. Spark and dwell are scheduled with Timer5 output compare (`OCR5A`/`OCR5B`), computed as an integer fraction of the last measured revolution period — no floating point in the ISR path.

Safety watchdogs in `loop()`:
- Forces the coil low if dwell exceeds `MAX_DWELL_US` (prevents coil damage from an unfired charge).
- Forces the coil low and drops sync if no trigger edge arrives within `STALL_US` (engine stopped/stalled).
- Hardware watchdog timer (AVR WDT): if `loop()` itself ever wedges (a firmware hang, not covered by the two software watchdogs above, which depend on `loop()` still running), the MCU force-resets and recovers rather than leaving the coil in whatever state it was in indefinitely. **Both** the production sketch and the debug build use the same `WDTO_15MS` — see bug #7 below for why the debug build originally used a longer, unsafe timeout and why that was wrong rather than a reasonable tradeoff. Both sketches disable the WDT immediately on boot via a `.init3` init function before it's deliberately re-enabled in `setup()` — this is required on this MCU/bootloader combination, since the stock Mega bootloader doesn't clear `MCUSR`/disable the WDT on entry, so without the early-disable a WDT-triggered reset can trap the board in a boot loop rather than actually recovering. Note this only covers a firmware hang — it has no effect on brownout/power-loss recovery time, which is dominated by the bootloader's own wait-for-upload delay on every reset (see Roadmap).

### Bugs found and fixed during bench bring-up (2026-07-26)

Bench testing with `pulse_simulator` surfaced two real correctness bugs, both confirmed on real hardware before and after the fix:

1. **16-bit capture aliasing at low rpm.** `ICR5` (Timer5's input-capture register) is only 16 bits, so a raw `cap - lastCapture` difference can only represent periods up to ~262 ms (~229 rpm) before silently wrapping to a wrong, too-small value. Below that speed the ignition board would either lock onto a stable but wildly wrong interpretation of the signal (observed firing at a computed "2400–4600 rpm" while the real speed was ~200 rpm) or get stuck in a permanent stall/resync loop producing zero sparks (observed at 100–150 rpm). Fixed by replacing the raw 16-bit capture with a proper 32-bit extended timestamp (`extendCapture()`): the 16-bit register plus a software-tracked overflow count, combined with the standard capture-vs-overflow race handling for this MCU's interrupt priority order. Interval measurement is now correct from ~10,000 rpm down to `PERIOD_MAX_TICKS`'s floor — verified by a scripted rpm ramp against `pulse_simulator`.
2. **Unsigned-subtraction race in the `loop()` watchdogs.** Both the stall and max-dwell checks computed `now - lastEdge` (or `now - highAt`) as unsigned `uint32_t` math. A capture interrupt landing in the few-microsecond gap between reading `now` and reading the shared timestamp could make the timestamp *newer* than `now`, underflowing the subtraction to ~4.29 billion and instantly (and falsely) tripping the watchdog — observed causing sync to drop on almost every single revolution. Fixed by casting both differences to `int32_t` before comparing, the standard idiom for handling both real 32-bit timer wraparound and small negative races safely.
3. **No hardware watchdog.** Both software watchdogs above depend on `loop()` still running, so a genuine firmware hang had no recovery path at all — the coil could stay in whatever state it was in (e.g. stuck charging) indefinitely. Fixed by adding the AVR hardware WDT (see the `loop()` watchdogs list above for the timeout/bootloader details) — verified with a 60s live bench run showing zero unexpected resets.
4. **Prescaler-limited scheduling floor.** `OCR5A`/`OCR5B` are 16-bit compare registers, so the edge-to-spark delay (`fracTicks`) has to fit in 16 bits to be scheduled correctly — at the original `/64` prescale (4µs/tick) that capped real minimum speed at ~200rpm; below that, the ISR safely declined to fire (`fracTicks > 0xFFFF`) rather than risk a wrong-angle spark, but also created an asymmetry where `PERIOD_MAX_TICKS`'s old ~80rpm floor let the ISR keep *measuring* correctly in the 80–200rpm range while never actually firing there. Moved the prescaler to `/256` (16µs/tick), which drops the true scheduling floor to ~50rpm — cost is coarser angular resolution (0.67° at 7000rpm, 0.08° at idle), negligible for this application. `PERIOD_MAX_TICKS` was raised to match (~54.5rpm) so the two gates line up again. Directly verified on hardware: temporarily set the simulator to a sustained 65rpm (previously impossible to fire at all) and confirmed clean, correct `FIRED` events every revolution.
5. **Rate-of-change plausibility gate added.** The existing absolute-bounds gate (`PERIOD_MIN/MAX_TICKS`) would accept a single dropped edge (~2× the true period) or a spurious extra edge (~0.5× or less) as a "legitimate" speed change, since 2× a normal period is often still inside the absolute bounds — this self-corrected on the next capture, but could schedule one or two wrong-angle sparks in the meantime. Added a check that also rejects anything outside roughly 0.5×–2× the last measured period once synced, catching this immediately.
6. **Compare-register write-ordering race.** The spark-scheduling code cleared `TIFR5`'s compare-match flags *before* writing the new `OCR5A`/`OCR5B` values. If `TCNT5` happened to pass through the stale leftover `OCR5A` value from the previous revolution in that narrow window, the flag would be set again, and the immediately-following `TIMSK5` enable would fire `COMPA` right away on the stale match — dropping the coil low and eating that revolution's real spark. Fixed by writing the compare registers first, then clearing flags, then enabling the interrupt (safe because `fracTicks` is always far enough out that the real match can't have already passed).
7. **Debug build's WDT timeout was a coil hazard, not a reasonable tradeoff.** The debug build originally used `WDTO_250MS` instead of production's `WDTO_15MS`, chosen to tolerate `Serial.print` calls blocking at high event rates. That reasoning missed what the WDT timeout actually bounds: if `loop()` ever hangs while `COIL_HIGH()` is asserted, the WDT timeout *is* the worst-case charge duration, since `MAX_DWELL_US`'s own software check also depends on `loop()` running. 250ms is ~83× the D514A's intended 3ms dwell — long enough to risk damaging the coil or its driver, on a build that does get used with a real coil on the bench. Fixed properly: made `printDebug()` and the watchdog-trip prints genuinely non-blocking by checking `Serial.availableForWrite()` and skipping the print (not the underlying safety action) when the TX ring isn't fully drained, using the existing `missed=` counter to keep skips visible. That bounds worst-case blocking to roughly `(lineLen-64)/11520 bytes/sec ≈ 6ms`, safely under 15ms, so both builds now run the same tight timeout. (Caught an off-by-one while implementing this: the AVR core's TX ring reserves one slot to distinguish empty from full, so `availableForWrite()` can only ever return `SERIAL_TX_BUFFER_SIZE - 1`, never `SERIAL_TX_BUFFER_SIZE` — checking against the wrong one meant the gate was permanently unsatisfiable and nothing printed at all. Verified working after the fix with a 30s live run at idle showing continuous correct telemetry.)
8. **Rate-of-change gate (bug #5) could lock out spark permanently, not just cost extra revolutions.** Reproduced live on the bench, not theoretical: at idle speed, if the trailing twin ever gets mistaken for a real edge right at resync (`FIXED_BLANK_TICKS` is a fixed absolute floor, but the twin's absolute gap grows at lower rpm), `lastPeriod` gets poisoned to the tiny twin-gap value. Every subsequent *real* edge then looks like a >2× jump from that wrong reference and gets rejected by the rate gate, which drops sync — and the next capture is still the twin, which mis-syncs the same way again. A stable trap with no way out: observed 30+ seconds of zero fired sparks at 800rpm, continuously alternating a mis-synced twin with a rejected real edge. Fixed with an escape hatch (`RATEJUMP_ESCAPE_COUNT`): after a few consecutive rate-gate rejections, stop trusting `lastPeriod` and rebuild sync from the current edge instead, exactly like a fresh start. Verified by deliberately reproducing the trap across several reboots and confirming the escape hatch recovers cleanly every time (visible in debug telemetry as one `ESCAPE` event ending a `RATEJUMP` streak, followed immediately by correct sustained `FIRED` operation).
9. **Dynamic blanking threshold didn't match the rate gate's lower bound.** Blanking was `3/8 × lastPeriod`, the rate gate's lower bound (bug #5) was `1/2 × lastPeriod` — an edge landing in `[3/8, 1/2)` used to pass blanking only to hit the rate gate and drop sync, where a silent blank would have been the gentler outcome for the same signal. Raised blanking to `1/2` to match; margin against wrongly blanking a genuine speed increase is unchanged in practice (still needs >2× acceleration in one revolution either way).
10. **Acquisition-phase blanking had no adaptive reference — the actual root cause of bug #8, not just its symptom.** The escape hatch in bug #8 recovers from a poisoned `lastPeriod`, but doesn't prevent one from happening: while unsynced, the dynamic blanking window collapses to the `FIXED_BLANK_TICKS` floor (3ms) because `lastPeriod` doesn't exist yet, and the twin becomes a viable (and wrong) sync candidate at any speed below ~833rpm for a 30° twin. No single fixed floor can fix this — a 300rpm twin gap (~16.7ms) is longer than a 6000rpm real period (10ms), so the bands genuinely overlap. Fixed by carrying the last accepted raw interval in `prevInterval`, using it as the blanking reference during acquisition (so one leading→twin→leading pass is enough to widen the window past the twin), and requiring two consecutive intervals to agree within `SYNC_AGREE_NUM`/`DEN` before declaring sync at all — a lone interval can't tell a real revolution apart from a leading-to-twin gap, but a leading→twin→leading alternation can never satisfy an agreement test (the two differ by ~12×). Verified on the bench: cold-start acquisition from a random boot-time phase now reliably completes within 2 revolutions across repeated reboots, with zero `RATEJUMP`/`ESCAPE` events, versus the old design's occasional full 30+ second lockout.
11. **Rate gate's upper bound sat exactly on the dropped-edge signature.** The gate was `interval > lastPeriod * 2` (strict) — a genuinely dropped edge produces an interval of *almost exactly* 2.0×, so whether it gets caught came down to jitter (2.02× caught, 1.98× passed and scheduled a spark off a doubled period). Tightening both bounds symmetrically was rejected: a starter spinning up genuinely can halve its period in one revolution during the first few revs, so a tight lower bound would reject real acceleration. A dropped edge always makes the interval *longer*, never shorter, so the fix is asymmetric — tightened the upper bound to 1.5× (`RATE_MAX_NUM`/`DEN`) to put clear daylight between the accept band and the dropped-edge signature, left the lower bound at 0.5× (`RATE_MIN_NUM`/`DEN`, unchanged, matching the blanking window so the same edge can't pass one gate and fail the other).
12. **A single dropped leading edge (with its twin still present) wasn't caught by any gate — found using the simulator's new fault-injection commands (see below), not anticipated in the original design.** Dropping only the leading edge doesn't produce the ~2× interval the rate gate expects, because the twin belongs to the same magnet-pass event and survives independently: the next pulse the board actually sees is that revolution's own twin, arriving at `period + twinGap` (only ~5–10% longer than normal at typical twin angles) — comfortably inside the 1.5× bound, so it fires a wrong-angle spark. The *following* revolution's real leading edge then arrives at `period − twinGap` relative to that false anchor, also inside bounds, firing a second wrong-angle spark, before a real twin is finally blanked correctly and the corrupted dwell scheduling trips `MAX_DWELL`. No interval-ratio threshold can separate this from ordinary rev-to-rev jitter, since both occupy overlapping ranges — a threshold-only fix cannot work here. Fixed by tracking whether the expected trailing twin was actually seen (`twinSeenThisRev`): after any accepted leading edge, the very next capture must be a blanked twin, or it's treated as untrustworthy (a new `TWINMISS` gate, sharing the same streak/escape mechanics as the rate gate). Verified on the bench with the simulator's dropped-edge injection (`d`): reduced the failure from two wrong-angle sparks plus a `MAX_DWELL` trip down to one wrong-angle spark (the twin itself, which can't be identified as illegitimate until the *following* capture) with zero dwell faults, and confirmed the repeated-drop case still reaches the shared escape hatch correctly (`TWINMISS`, `TWINMISS`, `ESCAPE`). The remaining single bad spark is a fundamental limit of a once-per-revolution trigger, not a bug: there's no way to know an edge is illegitimate before accepting it.

### Known hardware limitation: ~50 rpm minimum to fire

`OCR5A`/`OCR5B` are 16-bit compare registers, so the edge-to-spark delay itself (`fracTicks`, currently `AFTER_EDGE_DEG`/360 = 315/360 of a revolution) must fit in 16 bits to be scheduled correctly. Below **~50 rpm** (at the current `/256` prescale — this was ~200rpm before the fix above), that delay exceeds the register's range — the firmware explicitly detects this (`fracTicks > 0xFFFF`) and safely declines to fire that revolution rather than silently scheduling a wrong-angle spark. This is a property of using one 16-bit timer with a trigger angle this late in the revolution; the prescaler is the one lever that trades it against angular resolution, and it's now set about as far in that direction as makes sense for this application. **Before running on the real engine, confirm the actual starter cranking rpm is reliably above ~50 rpm** (should be comfortable margin for any real starter) — if somehow not, `ADVANCE_BTDC`/the trigger angle would need to move earlier in the revolution to shrink the edge-to-spark delay further.

## Building / flashing

**Arduino IDE:** Open the `one_cyl_ignition` folder (the `.ino` must sit inside a same-named folder — already set up that way here). Select **Tools → Board → Arduino Mega or Mega 2560**, pick the correct COM port, and upload. Repeat per board — with three identical Megas plugged in, double-check the port before each upload so you don't flash into the wrong slot.

**VS Code + PlatformIO** is also supported if preferred; not required.

## Bench spark test (no fuel)

Safe to crank with starter and check for spark with intake, carbs, and fuel supply disconnected/off, and exhaust off. No combustible mixture present = no risk of the engine firing.

Checklist:
1. **Ground every plug** to the block (via coil/threads) while cranking — an inductive coil firing into an open circuit (ungrounded plug) can punch through coil insulation or damage the smart coil's driver.
2. Plugs out first: confirm one clean spark per revolution, per cylinder — this validates the trailing-twin blanking is working. Two closely-spaced sparks means `FIXED_BLANK_TICKS` needs to be raised to cover the measured twin gap.
3. Plugs in (grounded, cylinder sealed): confirm spark stays strong under cranking compression.
4. Look for a fat blue-white spark on all three cylinders, consistently, across many revolutions.
5. Power the MCUs/conditioners from a stable supply separate from the starter/cranking battery — voltage sag during cranking can brown out the boards and produce erratic results that look like a firmware problem but aren't.

Timing light verification (confirming actual BTDC angle) comes later, once the engine can run — it's not meaningful during a no-combustion cranking test.

## Bench simulator (`pulse_simulator/pulse_simulator.ino`)

A separate MCU sketch that fakes cranking and the three pulse-coil signals, so the ignition boards can be exercised on the bench without turning the engine over.

It emulates the **conditioner's output** (clean digital edges), not the raw analog VR waveform — it feeds straight into each ignition board's `ICP5` (pin 48), bypassing the analog front end. That means it validates the ignition board's capture/blanking/timing logic in isolation; it does not test the analog conditioner circuit itself.

Per revolution, each of its 3 output channels produces the same "twin pulse" a real VR sensor gives: a leading edge (the real trigger) followed by a trailing echo edge `TWIN_GAP_DEG` later — exactly the pattern the ignition sketch's blanking logic has to reject. The three channels are staggered 120°/240° apart to model one crank with three sensor positions.

### Wiring

| From | To |
|---|---|
| Simulator pin 2 | Cylinder 1 ignition board, pin 48 (ICP5) |
| Simulator pin 3 | Cylinder 2 ignition board, pin 48 (ICP5) |
| Simulator pin 4 | Cylinder 3 ignition board, pin 48 (ICP5) |
| Simulator GND | Common ground with every ignition board (required) |

Any digital output pins work here — no ICP-specific requirement on the simulator side, since it only generates edges, it doesn't capture them. Shared power is optional; shared ground is not.

### Heartbeat LED

The 200 µs pulse-coil edges above are too brief/low-duty-cycle to see by eye. A separate heartbeat LED toggles once per revolution instead (independent of the pulse-coil output logic), giving a visible rpm-scaled blink: rpm/120 Hz, so ~2 Hz at cranking (250 rpm) and ~6.7 Hz at idle (800 rpm). It's driven on both pin 13 (the Mega's built-in LED, no wiring needed) and pin 6 (external — wire an LED + ~220-330 Ω resistor to pin 6 and GND for a bigger/brighter bench indicator).

### Usage

Flash `pulse_simulator.ino` to its own Mega (or any Arduino — no special peripherals used) and open a serial monitor at 115200 baud:

| Key | Action |
|---|---|
| `c` | drop target speed to cranking rpm (default 300 — raised from 250, which sat exactly on the ignition firmware's old plausibility ceiling with zero margin) |
| `i` | ramp target speed to idle rpm (default 800) |
| `s` | stop — simulates stall/kill switch, all outputs go low |
| `j` | toggle rev-to-rev speed jitter on/off |
| `+` / `-` | nudge target rpm by 50 |
| `p` | print current target/actual rpm, period, jitter state, drop/inject armed state |
| `d` | one-shot: drop cylinder 0's leading edge on the next revolution (its twin still fires) |
| `x` | one-shot: inject one spurious cylinder 0 edge at `injectAtDeg` (default 90°, i.e. 0.25× the period) on the next revolution |

Suggested test sequence: start with `j` off and `c`, confirm each ignition board sees exactly one spark event per revolution (validates blanking is rejecting the trailing twin), then turn `j` on to stress-test the plausibility gate under cranking-like speed noise, then `i` to confirm behavior carries cleanly through to idle speed.

`d` and `x` exercise the ignition firmware's fault gates directly instead of reasoning about them or waiting for one to show up incidentally (this is how bug #12 was found). Both are one-shot — they arm, fire on the next revolution, and disarm. On the debug build, expect `x` to show up as a silent `BLANKED` (0.25× lands below the blanking window, the cleanest possible outcome) and `d` to show up as one `FIRED` (the surviving twin, unavoidably accepted once) followed by a `TWINMISS` on the very next capture, then clean resync. Sending `d` three times in quick succession should reach the shared escape hatch (`TWINMISS`, `TWINMISS`, `ESCAPE`). Note `injectAtDeg` between roughly 0.5×–1.5× of the period would be accepted as a real edge by design — a once-per-revolution trigger has no way to distinguish that from a genuine one, and no firmware change can fix it.

`TWIN_GAP_DEG` (default 30°) in the simulator should be kept in sync with whatever real magnet width you eventually measure, and compared against `FIXED_BLANK_TICKS` in the ignition sketch — these two values are meant to track each other.

## Bench VR conditioner logger (`vr_logger/vr_logger.ino`)

A pure capture-and-log tool for characterizing the **real** VR conditioner circuit against a real sensor — the one thing `pulse_simulator` can't do, since it only fakes the conditioner's output rather than testing the actual analog front end. No ignition logic at all; it never drives a coil pin, so it's safe to run on a spare Mega with the real conditioner connected and nothing else.

Wire the conditioner's output to pin 48 (ICP5) — same pin, same Timer5 config (`/256` prescale, noise canceler on, same capture polarity) `one_cyl_ignition.ino` uses, so what's logged is exactly what the ignition firmware would actually see, not an idealized version of it. Turn the engine over with a drill (no fuel/plugs needed for this — it's purely characterizing the sensor/conditioner signal, not testing spark) and watch the serial log at 115200 baud.

It captures both edges (toggles `ICES5` after every interrupt) and logs each one:
- `EDGE=RISE gap_us=...` — the low time since the previous falling edge
- `EDGE=FALL pulse_us=...` — the high time since the previous rising edge

A clean revolution should show a short `pulse_us`, a short `gap_us` (the twin), another short `pulse_us`, then one long `gap_us` back to the next leading edge. That long/short gap split is exactly what `TWIN_GAP_DEG` (in `pulse_simulator`) and `FIXED_BLANK_TICKS` (in `one_cyl_ignition`) need to be tuned against — this tool is how you get real numbers to tune them with instead of guessing.

| Key | Action |
|---|---|
| `p` | print min/max/avg summary (pulse width, short-gap range, long-gap range) since last reset |
| `r` | reset summary stats (doesn't clear the live per-edge log) |
| `n` | toggle the input-capture noise canceler (`ICNC5`) live, to compare against production's default-on behavior — useful for spotting real bounce/noise that the canceler would otherwise hide |

`gapSplitUs` (top of the file, default 5000 µs) is just the short/long divider used to bucket gaps into the summary stats — it doesn't affect the raw per-edge log, only which bucket a `RISE` line's stats get folded into. Adjust it after a first look at the raw log if the real twin gap turns out to be much shorter or longer than 5 ms.

Once you have real `pulse_us`/`gap_us` numbers from this tool, feed them back into `one_cyl_ignition`'s `FIXED_BLANK_TICKS` and `pulse_simulator`'s `TWIN_GAP_DEG` (see Roadmap).

## Starter-cranking noise investigation (2026-07-29)

`vr_logger` gave a clean signal when the engine was spun over with a drill, but switched to real noise once the actual starter motor was used to crank it — the conditioner could no longer produce a clean digital signal for the ignition board to capture. A capacitive charge coil mounted in the same area was running open (unloaded) during this test, which is a known EMI radiator on its own and a potential confound worth re-testing with it loaded or disconnected.

**Root cause: a ground loop, not the sensor or the conditioner chip.** The VR pickup is wired correctly differential into the MAX9924/9926-family conditioner (coil lead → VR+, coil neg → VR-), so this isn't a simple single-ended-to-chassis mistake. The actual problem: the coil's neg lead exits through a hole in the flywheel casing and is bonded to that casing (→ chassis) right at the sensor. Meanwhile the conditioner/MCU's own ground was wired as a *separate* wire straight back to battery neg (the MCU was powered off a laptop battery, otherwise electrically isolated from the boat's electrical system). That created two disagreeing paths to the same nominal "ground":

- **Path A** — sensor neg lead → flywheel casing → chassis
- **Path B** — conditioner GND → MCU GND → direct wire → battery neg

These two paths only actually meet at the battery post, with current-carrying chassis in between. Confirmed on the scope: ground clip on fuse-box neg (true battery neg), probe on the pulse coil's neg lead, showed a significant voltage delta during cranking — the chassis IR drop from a few hundred amps of starter return current, injected as if it were real differential signal.

**Fix being tested (not yet verified on the bench):** remove the conditioner-GND-to-battery-neg wire (path B). Instead, run a short ground wire from the conditioner's GND directly to the chassis, landing as close as physically possible to the same bolt/node the coil's neg lead already bonds to. Why this works: the MAX differential VR input can only reject the chassis IR drop as *common mode* if the chip's own ground reference sits at the same potential as the coil's reference (the casing). Co-locating the conditioner ground with the coil-neg casing bond makes VR- and the chip's ground ride the same chassis potential together, so the cranking-induced offset becomes common-mode (rejected) instead of a differential error injected as signal.

Three conditions have to hold together for this fix, all now confirmed or planned:
- **The conditioner-GND-to-battery-neg wire (path B) is removed.** This is only possible *because* the MCU runs off an isolated laptop battery — it doesn't need boat-battery ground for power, so it's free to reference purely to the sensor's local point. **This isolation is load-bearing: if the MCUs are ever powered from the boat battery, this whole grounding scheme must be reconsidered.**
- **No starter-neg strap into the sensor's chassis region.** An earlier attempt strapped starter neg to a chassis bolt near the pulse lead — that was counterproductive, deliberately routing hundreds of amps of starter return current through the exact chassis region the sensor references. Removed. This is only safe to remove because the starter already has a proper dedicated return: a large battery-negative cable lands directly on a starter bolt, so bulk cranking current flows starter → bolt → cable → battery neg, kept out of the sensor's chassis region rather than forced through it.
- **Conditioner GND lands on a bolt away from the starter-neg bolt (off the high-current path) and closest to the pulse lead.** The chosen bolt already carries a small ground lead to another chassis bolt near the exhaust; that light-gauge jumper is high-impedance and won't carry meaningful cranking current, so it's harmless — but it confirms the bolt is bonded into the chassis network as intended.

Placement notes that matter: (1) co-location precision dominates — even a few inches of additional chassis between the new ground wire and the coil's actual casing bond reintroduces a smaller version of the same differential; (2) don't pick a bolt with the high-current starter termination sitting in the metal path *directly between* the coil-neg bond and the conditioner-GND bolt — if heavy current flows through the casting *between* the two reference points, the differential comes back. Both reference points should sit off to the same side of the starter-current path, not straddling it. Note the scope still showed a real battery-neg-to-casing delta *with* the big starter cable already in place — that's expected and fine: the cable keeps the *bulk* current out of a bad path, but the casing still carries some current and sits at an offset; referencing the conditioner to the casing (not battery neg) is exactly what turns that offset into common mode.

### Charge coil: high voltage on open leads — short to disable

Separate from the ground loop but found during the same investigation: the flywheel charge coil and lighting coil leads were left open (unterminated). Per the factory peak-voltage chart, the **charge coil output is ~90V peak even at cranking** (open-cranking 90.2V / connected-cranking 90.9V) rising to ~97–101V at 3500 rpm — a genuine shock hazard and a potential EMI source, not a trivial static charge (the lighting coil is much tamer: ~5.6V cranking, ~28V at 3500 rpm). Notably the factory procedure only tests the charge coil "open" at cranking speed, never at running speed — Yamaha's own method avoids leaving it open at speed.

The charge coil is **three wires = one center-tapped winding**, not two separate coils: L (blue) and Br (brown) are the two ends, B/R (black/red) is a tap partway along it (resistance L–B/R ≈ 656–984Ω, the higher-voltage segment; Br–B/R ≈ 172–258Ω, the lower). To safely disable it: **twist all three leads (L, B/R, Br) together at one splice and insulate that joint** — a plain dead short, no power resistor needed. Shorting the whole winding collapses its terminal voltage toward zero (energy goes into the winding's own I²R rather than building up as voltage) and damps its self-resonant ringing, which is the safest state and also removes whatever EMI the open coil was contributing. Shorting a permanent-magnet magneto coil is standard practice — current is self-limited by the coil's own impedance, and shunt regulators short stator windings as their normal regulation mode. (Honest caveat: a dead short can draw more *continuous* current than pulsed CDI service does, so the concern would be sustained heating at high rpm — a non-issue for bench cranking here; if ever run hard for extended periods, reconnect the coil's original rectifier load instead of a random resistor.) Shorting only *one* pair would leave the other winding segment open and still able to develop voltage — hence all three together. Whether this measurably helps the line noise is secondary to the ground fix (which alone explains the scoped delta), but it's worth doing regardless for safety.

A before-fix baseline capture is saved at `bench_logs/vr_log_before_groundfix_2026-07-28.txt` (noise canceler off, real starter cranking) — no clean once-per-rev twin-pulse rhythm anywhere; pulse/gap widths range from sub-64µs chatter to 27,000–130,000µs stretches. Capture a same-format log after the ground fix, at the same test point (starter cranking, not the clean drill baseline), to compare directly rather than judging "it looks cleaner" by eye.

### Post-fix results (2026-07-31)

After terminating the charge coil (all three leads twisted together and insulated) and moving the VR conditioner ground to a single point at the flywheel-casing bolt (removing the conditioner-GND-to-battery-neg wire), five `vr_logger` captures were taken. All are in `bench_logs/`:

| File | Drive | Canceler | rpm | Edges/rev | Notes |
|---|---|---|---|---|---|
| `vr_log_before_groundfix_2026-07-28.txt` | starter | off | ~378 | ~10, unreadable | baseline: sub-tick chatter, no stable rhythm |
| `vr_after_starter_off_2026-07-31.txt` | starter | off | ~367 | ~7.7 | post-fix, first capture |
| `vr_after_starter_off_b_2026-07-31.txt` | starter | off | ~367 | ~7.7 | post-fix, repeat |
| `vr_after_starter_on_2026-07-31.txt` | starter | **on** | ~374 | ~8.1 | canceler makes no difference |
| `vr_after_drill_on_2026-07-31.txt` | drill | on | ~163 | ~5.3 | drill A/B |

**What the ground fix fixed (confirmed):** the high-frequency noise floor is gone. Baseline captures bottomed out at 0µs edge widths (sub-timer-tick chatter — noise edges piling up inside one tick); every post-fix capture has a minimum edge width of ~176µs and **no zero-width edges at all**. The revolution rhythm went from wildly erratic (110k–301k µs period, no stable once-per-rev) to a rock-steady ~160–167k µs period. The ground-loop diagnosis (chassis IR drop from starter return current injected as false differential) is validated — that class of noise is eliminated.

**What it did NOT fix, and why that's OK:** at cranking speed the conditioner still emits a **multi-edge burst** (~5–8 edges) per magnet pass instead of one clean edge, with a consistent `long-HIGH → medium-HIGH → short → short` signature every rev. Three findings pin down the cause:

- **The noise canceler (`n` toggle) made no difference** (starter on vs off: same ~8 edges/rev, same signature). The ICP hardware canceler only rejects sub-microsecond glitches; these edges are 3–8 ms wide, so they're **genuine conditioner output transitions, not electrical glitches.**
- **The drill shows the same burst** (~5 edges/rev), so it is **inherent to the VR sensor/conditioner at low speed, not starter-specific electrical noise.** (An earlier impression that the drill gave a "clean" signal was relative to the noise-mush baseline, not literally one edge per rev.)
- **The "long-HIGH" is not a fixed conditioner timeout** — it scales with speed (84 ms at ~370 rpm starter, ~117 ms at ~163 rpm drill), i.e. it tracks the raw VR waveform of one slow magnet pass. A VR sensor's output amplitude is proportional to rpm, so at cranking speed the signal barely clears the conditioner's adaptive threshold and it arms/disarms several times across a single slow bipolar pass. This is the textbook VR-at-cranking limitation; grounding and noise-canceling cannot fix it because it is not noise.

**Crucially, this multi-edge burst is not disqualifying for the ignition firmware.** `vr_logger` logs *every* edge (that's its purpose); the ignition firmware deliberately discards most of them. It keeps one leading edge per rev and blanks every edge for `max(FIXED_BLANK_TICKS, lastPeriod/2)` after it, requires two consecutive intervals to agree within ±25% before declaring sync, and rate-gates intervals to 0.5×–1.5× the last period. The burst is exactly the "trailing twin + low-speed multi-trigger" mess those filters were built to reject. Consistent with this, **running the actual ignition firmware on the drill produced clean ignitions** — the firmware locks onto one stable reference edge and blanks the rest. The real value of the ground fix is that it gave the *starter* signal the stable, repeatable per-rev rhythm the blanking logic needs to lock onto (the noisy baseline had no such rhythm).

**The one test that still gates deployment:** run the ignition firmware — ideally the debug build for telemetry — under **real starter cranking**, post-fix, and confirm clean stable ignitions at correct timing, as the drill gave. Watch for the leading edge staying on the same physical edge each rev (timing jitter if it hops) and any `MAX_DWELL` trips or wrong-angle sparks. Steady starter cranking should be easier for the decoder than the erratic hand-drill. If that test *does* show instability, the 1-magnet/3-Hall swap below is the robust fix (Hall output is amplitude-independent — one clean edge per pass at any rpm, eliminating the low-speed marginality entirely); until then it stays an optional robustness upgrade, not a required change.

*(Data note: a few absurd values in the post-fix logs — e.g. `pulse_us=606331584` — are `vr_logger` capture-overflow glitches, single occurrences; ignore them. The tool reported `dropped=0` on all runs.)*

### Firmware sync test under starter (2026-07-31) — FAILED, root-caused

The debug ignition build (`one_cyl_ignition_debug`, driving an LED on pin 5, no coil) was flashed and cranked with the real starter, telemetry captured to `bench_logs/starter_debug_2026-07-31.txt`. **The firmware did not hold sync.** It re-acquired sync 6 times and fired only 8 sparks total, each run collapsing within 1–2 revs; 96 `SYNCCAND` events means the two-intervals-agree gate failed nearly every revolution. The 8 sparks fired at nonsense periods — reported rpm of 1735/895/486/871/891/381/589/1035 against a true cranking speed of ~375 rpm — i.e. the decoder locked onto **intra-burst edge spacings** (34–100 ms fractions of a rev) and fired as if spinning 2–5× faster. `edgeToSpark_us` swung 30k–137k µs, so on a real coil these sparks would land at random crank angles (including near TDC on compression — a kickback hazard). **Do not run this on a real coil or fuel under the starter until sync is fixed.**

**Root cause: the blank window is shorter than the conditioner's burst at starter speed.** The firmware blanks `lastPeriod/2` after the kept leading edge. The multi-edge burst's absolute duration barely shrinks with rpm (long-HIGH ~117 ms on the drill at 163 rpm, ~84 ms on the starter at 375 rpm — *not* proportional to speed), so it occupies a growing fraction of the revolution as rpm rises:

| | rev period | burst (long-HIGH) | blank = period/2 | result |
|---|---|---|---|---|
| Drill (163 rpm) | ~368 ms | ~117 ms (32%) | ~184 ms | burst fits inside blank → masked ✓ |
| Starter (375 rpm) | ~160 ms | ~84 ms (52%) | ~80 ms | burst exceeds blank → trailing edge leaks ✗ |

At starter speed a trailing burst edge escapes the half-period blank and is mistaken for the next rev's leading edge → false short period → mis-sync. Once a short false period is locked, the blank shrinks to half of *that* (~33 ms), which can't even cover the 84 ms long-HIGH — a self-reinforcing mis-sync it can't recover from. The drill "worked" (clean ignitions) only because it was slow enough that `period/2` swallowed the whole burst. **The earlier expectation that steady starter cranking would be easier than the erratic drill was exactly backwards** — faster rev = shorter blank vs a burst that grows as a fraction of the rev.

**Implication:** this can't be safely tuned away with a bigger blank (a blank long enough to cover the burst at cranking would blank past the real trigger at higher rpm, and the burst fraction only grows with rpm until the VR signal is strong enough to stop the conditioner multi-triggering — an unknown, un-guaranteed point above cranking). The fix must stop the burst at the source or change sensors:
- **Cheapest first shot:** reduce the VR air gap / fit a stronger trigger magnet so the conditioner emits one clean edge at cranking instead of a burst. If it does, the existing firmware works unchanged.
- **The 1-magnet/3-Hall swap is now warranted, not optional** — a Hall sensor gives one clean edge per rev with no burst and no blank-vs-burst race, structurally eliminating the failure the bench just demonstrated. (This overturns the "optional robustness upgrade" framing above, which was based on the drill result; the starter result is decisive.)

### Alternative sensor architectures considered

If the ground fix doesn't fully resolve the noise, two sensor-hardware alternatives came up:

- **Revive the old 12-1 magnet wheel + single Hall sensor**, originally built for a Speeduino attempt on this engine. That attempt stalled on Speeduino needing a separate cam sensor to disambiguate a 3-cylinder crank — a problem specific to Speeduino's 4-stroke-oriented decoder architecture, not to this engine (a two-stroke fires every revolution, so there's no 4-stroke half-speed cam ambiguity to resolve) or to this project's firmware (each board already knows its own cylinder from wiring, not from decoding cylinder identity out of a shared signal). Reviving it here wouldn't hit the same wall, but it's still the bigger change: one shared crank sensor instead of three independent per-cylinder ones (a Hall sensor failure now stalls all three cylinders, not one — a real regression from the "Why the boards are identical" fault-isolation argument above, unless the single Hall output is fanned out to all three boards, each computing its own TDC independently), plus new firmware for missing-tooth sync detection and per-board TDC-offset constants (the boards would no longer be byte-identical, same as the per-cylinder-advance-curve roadmap item below).
- **1 magnet + 3 Hall sensors** — closer to the stock Yamaha trigger config, and a much smaller change than the wheel: same one-trigger-per-cylinder-per-rev physical layout as the current VR coils, just a Hall sensor instead of a VR coil at each of the three existing sensor positions. No cam sync, no per-board TDC redesign, and it sidesteps the same ground-loop/analog-noise class of problem the wheel option does. It would also simplify the firmware: a Hall sensor sees the magnet pass as a single clean high period, not a leading-edge-plus-echo, so most of the "twin pulse" handling (bugs #5, #8, #9, #10, #11, #12 above) wouldn't apply to a from-scratch Hall version — the rate-of-change gate, 16-bit capture aliasing fix, watchdog races, and prescaler floor (bugs #1–#4, #6) would still be relevant since those are general timer/capture robustness, not VR-specific.

## Landmark decoder — recovering VR sync at cranking (experiment branch, 2026-07-31)

Lives in `one_cyl_ignition_landmark/` on branch `experiment/longest-pulse-landmark` (not merged; master still holds the production firmware). This is the firmware answer to the starter-cranking failure documented above — an alternative to the Hall swap.

**The idea.** The production decoder fails at starter speed because its blank window (`lastPeriod/2`) is shorter than the conditioner's multi-edge burst, so a trailing burst edge leaks past and mis-syncs. But every revolution contains **one uniquely-large rising-to-rising interval** (~90 ms at ~370 rpm — the long-HIGH span, ~2.5× any other interval in the rev). Keying sync on that *shape* landmark instead of a time-blank gives one un-spoofable reference per rev. No dual-edge capture is needed: the single rising-edge interval already spans the long HIGH. Offline analysis of the `vr_logger` captures showed that landmark recurs at a stable ~370 rpm (period CV ~8 %), so a decoder locked to it would sync cleanly where the production decoder can't.

**Tuning progression (all bench-cranked on the real starter, LED then coil):**

| Version | Change | Result |
|---|---|---|
| v1 | landmark sync via the uniquely-large interval | true rpm, but ~32 % single-rev "hops" (refBig spiking on irregular revs) |
| v2 | cap refBig rise ≤25 %/edge | firings tight (356 ± 5 rpm) but frequent unsync → misses revs |
| v3 | sticky sync + persist refBig across stall | fires more, but preserves a corrupted anchor → wrong-rpm sparks (SD 140) |
| v4 | predictive window (only a landmark near the predicted time may move the anchor) | 48 fired, 384 ± 11 rpm — clean *and* frequent, but still anchored to the **raw** edge |
| **v5** | **dead-reckoning PLL** | fires off a smoothed **model**, not the raw edge — bounded scatter, no wrong-angle sparks |

An anti-correlation test on the v4 log (long revs systematically followed by short ones summing to ~2× the period) proved the raw edge still jitters ±one burst-position (~24–30 ms ≈ ±68°) at rough cranking — about half real starter torque-pulse speed variation, half landmark hop, and the two can't be fully separated from the signal alone. So anchoring the spark to the raw edge (v1–v4) fires ~40 % of revs at a wrong angle — a real-coil hazard.

**v5 dead-reckoning PLL (current).** Keeps a crank phase/period **model** and fires *every rev off the model*, not the raw edge. Each accepted landmark only nudges the model — phase by `residual>>3` (⅛), frequency by `residual>>5` (1/32) — so a single hopped edge moves the spark by only ~⅛ of the hop while a sustained real speed change still tracks in a few revs. `n`-rounding absorbs a genuinely missed landmark (fire on rev N+1 with n=2 instead of mis-locking); a 0.6·P refractory ignores extra/early edges so it never double-fires; >3·P since a model landmark means it's lost → re-acquire. Leaning on the model is also the *safe* choice at cranking: worst case is a few degrees of scatter, never a wild wrong-angle spark.

Bench result (LED, real starter, `bench_logs/starter_landmark_pll_2026-07-31.txt`): model rpm smooth to ~2 rpm/rev steps, signed residual centered (+2 ms, 43 late / 43 early → no bias), fires every rev, one clean re-lock at a 1.24 s pause. Raw edges still jitter ~24 ms median (inherent VR-at-cranking burst ambiguity); the PLL attenuates that to ~±3 ms (±7° crank, ±16° worst) spark scatter.

**Key finding — it cleans up with rpm.** On a faster crank (~463 rpm, `bench_logs/timinglight_coil_2026-07-31.txt`) the residual dropped to **under 1 ms** (vs ~24 ms at ~350 rpm). This is the predicted VR behavior — output amplitude ∝ rpm — and strongly suggests the residual cranking jitter largely evaporates at running speed. It is the main reason this firmware path looks viable rather than a dead end.

### Timing-light / strobe calibration (in progress)

The landmark edge's **true crank angle is not yet calibrated** — `edgeToSpark` is consistent rev-to-rev but its BTDC value is unverified, so `AFTER_EDGE_DEG` (currently 315) still has to be dialed in with a timing light. Status as of 2026-07-31:

- **No-fuel coil test works.** With the coil on pin 5 (and the mandatory 10 kΩ pin-5→GND pulldown, placed at the coil end so a loose signal wire still holds the trigger low), cranking produces a strong white spark every rev. With no fuel there's no combustion, so a wrong-angle spark is harmless — this is the standard no-fuel bench test, safe to run while the angle is still unverified.
- **A standard inductive timing light would not trigger** on the D514A smart coil at cranking rate (powered, clamped correctly, arrow toward plug, still no flash) — the smart coil's HT pulse and the low, irregular ~7–8 sparks/sec are a poor match for the inductive pickup.
- **Built an MCU strobe instead** (pin 6, `STROBE_*`): a short flash at the exact spark instant (`COMPA`), for reading the flywheel marks like a timing light without depending on the inductive pickup. A boot self-test blinks pin 6 six times so the wiring can be verified without cranking. **Open issue:** 3 white LEDs driven directly off pin 6 through 100 Ω each draw ~54 mA total — over the pin's limit, so they sag and read too dim to use. **Next session:** drive them from the 12 V rail through an NPN transistor (pin 6 → 1 kΩ → base; LEDs from +12 V through ~200 Ω each → collector → emitter → GND), or use a single LED direct. Then read the fired advance, compute the correction, and set `AFTER_EDGE_DEG`.

**Update 2026-08-01 — calibration confirmed, cranking retard verified, ready to start.** With a charged cranking battery (steady ~436 rpm, vs the jittery low-battery read the night before), the strobe showed the fixed-15° build firing at ~15° BTDC — i.e. **`TRIGGER_ANGLE_BTDC=330` is correct**; the earlier "~30°" was low-battery noise. Flashing the cranking-retard build then retarded the cranking spark to a **0–15° BTDC window centered near TDC** (5° intended + cranking jitter) — kickback-safe and start-friendly, exactly as designed. The ignition side is now **calibrated and ready for a no-fuel→fuel start attempt**; next real step is blocked only on the fuel system (as of 2026-08-02 the carbs are gone entirely — throttle body direct to intake). After it starts and revs past 500 rpm it auto-advances to 15°, and the running advance gets dialed in at idle with a timing light (which should also trigger properly at running spark rates).

### Remaining gates before this replaces the production decoder

1. Get the strobe bright enough, read the fired advance, and **calibrate `AFTER_EDGE_DEG`** to the target BTDC.
2. **Characterize at higher rpm** (spin faster than cranking) — confirm the residual jitter largely vanishes as predicted.
3. **Port the landmark PLL into production `one_cyl_ignition.ino`** (it currently lives only in the debug-style experiment sketch, which drives an LED/strobe and prints telemetry).

### Safety review (2026-08-01)

A pass focused on engine-damage / kickback failure modes. The core protections are sound: stuck-coil overcharge is layered (`MAX_DWELL` 5 ms + WDT 15 ms + `STALL` 3 s all force the coil low), reset/brownout lands in a safe state (pin-5 pulldown + `COIL_LOW()` in setup + re-sync-before-fire), and the scheduling math is range-checked. Two fixes were made immediately; the rest are tracked here.

**Fixed:**
- **Max-advance safety clamp.** `fracTicks` is now floored so the spark can never fire more advanced than `MAX_ADVANCE_BTDC` (25°), and the floor is computed from the *actual measured* rev time (`delta/n`), not the model — so a wrong or lagging model (e.g. a hard deceleration the ÷32 frequency loop hasn't caught) can't schedule a far-BTDC kickback spark. Normal 5°/15° operation never reaches the clamp.
- **Boot delay gated off.** The ~1 s blocking strobe self-test (dead ignition on every reset → would stall a running engine) is now behind `STROBE_BOOT_TEST`, default `0`. Enable only for bench wiring bring-up.

**Still open (address before sustained running, not blocking a first start):**
- **Cranking-retard decision uses the model period, not actual speed** (`phasePeriod > CRANK_PERIOD_TICKS`). On a hard decel a lagging model can stay in "run" (15°) while actually slow. The max-advance clamp now bounds the worst case, but the retard test should ideally key off measured `delta`.
- **A `MAX_DWELL` trip forces `COIL_LOW` = an uncontrolled-angle spark** (~1/150 revs observed). Can't avoid the discharge on an inductive coil, so the goal is to eliminate the *trips* — likely root cause is the compare-just-passed race in the immediate-charge path (`OCR5A` written, then flags cleared, then `OCIE5A` enabled; if `TCNT5` passes in the gap the match is missed until a ~1 s wrap). Harden the write/enable ordering.
- **Slow accel tracking (÷32)** lags the catch → retarded/ATDC sparks during rev-up (errs safe, but sluggish). Consider a faster `KF` or an accel feedforward.
- **Acquisition could lock a wrong (intra-burst/half-rev) period.** Low probability, strobe-visible, retard-bounded — add an rpm-plausibility cross-check to harden.
- This whole telemetry/strobe **experiment build should be ported to production `one_cyl_ignition.ino`** before it's the thing running on fuel long-term.

## EFI (Speeduino, fuel-only) — decided 2026-08-01

Adding fuel injection via the fitted Daytona throttle bodies + injectors, to get the engine
started. **Decision: ignition stays entirely on the 3 MCUs; add Speeduino for FUEL ONLY**,
triggered by a clean once-per-rev signal *from our ignition output*. Full build/verification
plan captured separately; summary:

**Why fuel-only, not Speeduino-does-everything:**
- The earlier 3-cyl ignition trouble was almost certainly the messy VR-conditioner signal —
  the landmark decoder is what tames it, so keep the thing that solved it.
- Speeduino can't do per-cylinder ignition timing; the independent boards can (roadmap).
- Preserves the 3-board fault isolation instead of a single ECU controlling everything.
- The VR burst can only be cleaned by the landmark logic (our firmware) — a dumb hardware
  conditioner can't. So Speeduino's clean trigger *comes from our ignition's output*, not from
  re-conditioning the VR. (Moving ignition onto Speeduino fed one-pulse-per-rev would inherit
  the same once-per-rev precision limit *and* lose fault isolation — no gain.)

**Board:** Speeduino **v0.4.4d** (on hand, Arduino-Mega-based like our ignition boards). Covers
a lot out of the box: **onboard MAP** (MPX4250 — no MAP to source), **4 injector channels with
onboard drivers** (use 3, batch), 4 ignition channels left unwired, and CLT/IAT/TPS inputs.

**Architecture:**
- **Trigger:** a once-per-rev pulse from our ignition (tap the **pin-5 coil-trigger** edge, zero
  firmware; the ~5–15° advance movement is negligible for fuel) into the 0.4.4's **Hall/logic**
  trigger input — **not** through the board's onboard VR conditioner (MAX9926); our signal is
  already clean, so set the trigger to Hall in TunerStudio and wire to the logic input. Fits
  Speeduino's **Basic Distributor** mode (N clean pulses/rev, no cam) directly.
- **Grounding — direct-first, verify, opto as fallback:** a direct wire needs a shared ground,
  which risks Speeduino's injector/pump switching current coupling back into the VR reference.
  So: run Speeduino's high-current grounds (injectors, fuel pump) **straight to battery negative**
  and share only a light signal ground MCU↔Speeduino; then **watch the landmark telemetry with
  Speeduino powered and injecting** — if the VR signal stays clean, done. Only if noise reappears,
  drop in an optocoupler (a single 6N137/PC817 + resistors, not a board) to break the ground tie.
- **Injectors:** injectors only on the bodies → the 0.4.4's onboard drivers, **batch** (all fire
  together once/rev — plenty to start a two-stroke). Onboard drivers suit **high-Z** injectors;
  confirm impedance (low-Z → ballast resistors / peak-and-hold).
- **Oiling:** premix **marine TCW-3** in the tank, injected with the fuel — lubricates the
  crankcase exactly as the carbs did (carbs removed entirely 2026-08-02; throttle body direct to intake). Testing-grade (2-stroke oil can varnish injectors / isn't
  ideal for an EFI pump long-term); revisit for a permanent install.
- **Control:** fixed cranking pulsewidth + prime + after-start enrichment, trimmed live in
  TunerStudio. No VE table needed to catch.

**Have:** HP pump/regulator/rail/filter/plumbing, throttle bodies + injectors, **Speeduino v0.4.4d**
(with onboard MAP). **Need:** confirm **injector impedance** (high-Z vs low-Z), a **CLT temp
sensor** for cranking enrichment (IAT optional), and an **optocoupler on the shelf** as the
grounding fallback (only fitted if the telemetry shows VR noise).

**De-risking order:** wire and prove the *trigger* first (crank with no fuel/coil, confirm
TunerStudio reads steady rpm) before touching fuel — that isolates the riskiest integration
point. Optional later: a dedicated fixed-angle once-per-rev pulse output in the ignition firmware
(toggle a spare pin at the landmark; put it in all boards to stay byte-identical, wire one to
Speeduino) for a reference that doesn't move with spark advance.

A single shared magnet/wheel feeding 3 Hall sensors (see "Alternative sensor architectures") would
also feed Speeduino's Basic Distributor mode directly and could retire the landmark decoder — but
that's a bigger sensor-hardware rebuild, not the get-it-started path.

## EFI trigger output — built and bench-proven 2026-08-02

**The plan above said to tap pin 5 from all three boards into a diode-OR. That's not what was
built.** Speeduino's Basic Distributor decoder sets `triggerActualTeeth = nCylinders`, so it
expects **3 evenly-spaced pulses per crank revolution** — one per cylinder, as a real distributor
would produce. Feeding it a single board's once-per-rev pulse made it read ~1/3 of true speed
(the ~122-150 rpm seen against a true ~370).

Rather than wire all three boards together, **one board synthesises all 3 pulses** by subdividing
its own PLL-tracked rev period on **D9 (PH6)**. Speeduino only counts pulses, so it can't tell the
difference — and this works with a single board on the bench, needs no diode-OR, and could stay
the permanent architecture (fewer parts, no cross-board wiring, and the sub-pulses come from one
already-smoothed model rather than three independently-tracking boards).

**Wiring:** `D9 -> 1N5817 anode, cathode -> Speeduino D19 (CAS/trigger input)`, with a 10k pulldown
from the shared node to ground. TunerStudio: Trigger Pattern **Basic Distributor**, Trigger edge
**RISING**, Trigger Filter **Medium**, Engine Stroke **Two-stroke**, 3 cylinders. Injector Layout
**Sequential** is correct here — Speeduino special-cases two-strokes (`init.cpp`: 0/120/240° over
`CRANK_ANGLE_MAX_INJ = 360`), and the cam requirement that applies to 4-stroke sequential doesn't,
because a two-stroke fires every rev so there's no 720° ambiguity to resolve. (Fallback if it ever
won't sync: switch Layout to **Paired**, same 0/120/240 angles.)

### Only ONE board feeds Speeduino — do NOT diode-OR all three

All three boards run the **same firmware** and all three generate the D9 train unconditionally;
**only one has its D9 wired** to Speeduino. The other two just run their own cylinder's ignition
with D9 going nowhere. Nothing designates "the EFI board" in firmware — wiring alone decides — so
the byte-identical invariant is preserved and a pre-flashed spare drops into any position,
including the trigger-source one. (A compile-time flag or jumper would have broken that; hence it
was written this way deliberately.)

> **GOTCHA — this is the opposite of the original pin-5 plan, and looks like a sensible redundancy
> upgrade until it isn't.** The *old* plan tapped **pin 5** from each board — **one** pulse/rev each
> — and diode-OR'd them into the 3 pulses/rev Basic Distributor wants. The *current* design has each
> board independently synthesising **all three** pulses/rev, free-running off its own PLL with **no
> phase relationship to the other boards**. OR three of those together and you get up to **9
> pulses/rev**, and Speeduino reads roughly **3x true rpm**. Wire exactly one.

**Fault asymmetry worth knowing:** the trigger-source board is a single point of failure for
*fuel* — if it dies, Speeduino loses its trigger and all three injectors stop, killing the engine
outright. A failure on either of the other two costs only that cylinder's spark. This doesn't
really regress anything (Speeduino was already central to fuel by choice), but it does mean the
ignition side's per-cylinder fault isolation **stops at the fuel boundary**.

**Two firmware iterations — the second matters:**
- **v1 re-anchored the train to the raw landmark edge each rev.** This leaked raw-edge jitter
  straight back in, which is precisely what the v5 PLL exists to reject. Signature in
  `bench_logs/efi_trigger_v1_anchored_2026-08-02.csv`: 16 of 18 long intervals immediately
  cancelled by a short one (~±6 ms, ~13 crank degrees of displacement), 9.27% stdev, one dropped
  pulse, and a recovery gap **3.5% off Speeduino's trigger-filter reject threshold** (Medium =
  50% of previous gap) — i.e. close to cascading into sync loss.
- **v2 takes only the PERIOD from the model and free-runs**, never re-anchoring to an edge, with a
  catch-up guard so a late `loop()` re-bases rather than rapid-firing a backlog (a pulse burst is
  exactly what the trigger filter rejects). Absolute phase drifts slowly, which is fine — Basic
  Distributor only counts pulses, it doesn't use them for ignition timing.

**Measured (v2, `bench_logs/efi_trigger_v2_freerun_2026-08-02.csv` and the 20:35/20:40 captures):**

| | v1 | v2 |
|---|---|---|
| interval stdev | 9.27% | **0.01%** (7.9 µs over 40 steady intervals) |
| worst filter ratio | 0.518 (near reject) | **0.862** |
| dropped pulses | 1 in 179 | **0** |
| Sync Loss # | — | **0** over a full crank |
| rpm | 370.2 | 368 (true ~370) |

**Gotcha when reading composite logs:** the composite buffer holds 127 entries and stops recording
until TunerStudio reads it out, so apparent 270-330 ms "dropouts" appear at buffer boundaries.
In the 20:35 capture there were exactly 3, against 497 logged edges = 3.9 buffer fills. They are
logging artifacts, not signal loss — confirmed by `Sync Loss # = 0` in the parallel datalog, and by
the fact that a real 331 ms gap would have exceeded `MAX_STALL_TIME` (~222 ms for this config) and
dropped sync.

## Sensor punch-list — mostly CLEARED 2026-08-02

Datalog `2026-08-02_20.40.21.msl` confirmed **injection is commanded on all 3 channels off our
trigger** (PW1/2/3 firing). But every analog input initially read invalid, pegging every
correction at 255% (byte-maximum saturation, not real enrichment demand) and clamping PW.
Three rounds of fixes, each verified by a fresh crank datalog:

| | orig (20:40) | +IAT/baro (21:18) | +inj V curve (21:25) |
|---|---|---|---|
| `Gammae` | 1427.6 | 258.0 | **101.0** |
| `Gbattery` | 255 | 255 | **100** |
| `Gair` | 255 | 100 | **100** |
| `Gbaro` | 255 | 100 | **100** |
| PW | 16.1 ms *(clamped)* | 52.4 ms | **22.3 ms** |
| Duty | 9.8% | 32.6% | **14.0%** |
| Battery V | 2.0 | 12.0 | **12.2** |
| Sync Loss # | 0 | 0 | **0** |

**What was fixed, and the gotchas:**
- **Battery voltage** read a flat 2.00 V. Fixed with **Tools → Calibrate Voltage Reading** ("Battery
  Voltage reading offset", `batVoltCorrect`, signed, ±2 V range, added directly to `battery10` in
  firmware). Note this is an *offset*, not a scale — verify it still tracks at cranking voltage
  (~12 V), not just at rest.
- **MAP and IAT are deliberately not used.** But *not fitting a sensor does not disable its
  correction* — `correctionIATDensity()` / `correctionBaro()` are plain table lookups with no
  enable flag, so they return whatever the table holds at the garbage input. Fixed by flattening
  **Settings → IAT Density** and **Settings → Barometric Correction** to 100% across the board.
  (Baro is derived from MAP at startup, so flattening its table also cuts the MAP dependency.)
- **Injector voltage correction curve was corrupt** — all 6 values at 255 *and all 6 voltage bins
  at 25.5 V*. Degenerate bins mean a 2D lookup can't interpolate, so it returned 255 regardless of
  input; that's why fixing the battery reading alone didn't move `Gbattery`. Compounding it, mode
  was **Whole PW**, so 255% multiplied the *entire* pulsewidth (52.4 / 2.55 ≈ 20.5 — accounts for
  the blowup almost exactly). Fixed to **Open Time only** with bins 6.0/9.2/12.4/15.6/18.8/22.0 and
  values 250/170/115/95/85/80 (≈110% at 13.2 V). Open Time only is both physically correct and a
  safer failure mode — a bad value scales ~1 ms of dead time, not the whole pulse.

**TPS and CLT — both resolved later the same session:**
- **TPS** wired and calibrated (Tools → Calibrate TPS), now reads **0% closed**, sweeps to 100%.
  Identify the three leads with a meter, not by colour: the two wires whose mutual resistance
  does **not** change as the throttle sweeps are the pot's end terminals (5V and ground); the
  third is the wiper/signal. At closed throttle, the end with *lower* resistance to the wiper is
  ground. Guessing risks putting 5V across near-zero ohms — a short across the 5V rail.
- **CLT**: no thermistor on hand, so a **fixed resistor stands in for the sensor** (signal pin to
  ground; the board supplies the pull-up). Calibrated via **Tools → Calibrate Temperature Sensors**
  with bias **2490Ω** and a standard NTC curve, giving a stable **82 °F**.

> **UNITS TRAP — cost real time.** TunerStudio's gauges and every enrichment table here are in
> **Fahrenheit**, even though the thermistor calibration dialog has its own separate C/F radio
> button. An early CLT reading of "88" was read as a wrong 88 °C when it was a perfectly correct
> 88 °F (= 31 °C, exactly what the divider maths predicts for 2.2kΩ against a 2490Ω bias). That
> sent us chasing a nonexistent bias-resistor fault. **Check units before diagnosing a temperature.**

> **The fixed-resistor CLT is a bench stand-in, not a calibration.** It reports a constant ~82 °F,
> so warmup enrichment will never taper as the engine actually warms. Fine for a start attempt;
> replace with a real thermistor and recalibrate before running properly.

## Enrichment tables — SIX found corrupt, all rebuilt 2026-08-02

Beyond the sensor inputs, most of the tune's enrichment tables had never been initialised. Two
distinct failure signatures, both worth recognising:

1. **Degenerate bins** — every X-axis bin set to the same value (all `419`, all `-40`, all `25.5`).
   A 2D lookup then cannot interpolate and returns the first value regardless of input. This is
   why fixing the battery *reading* didn't move `Gbattery`: the injector voltage curve's bins were
   all 25.5 V, so the lookup was degenerate no matter what the sensor said.
2. **Byte-max garbage values** — `255`, `419`, `417`, `127.5` appearing as data.

| Table | Was | Now |
|---|---|---|
| Injector voltage correction | all 255%, all bins 25.5 V, mode **Whole PW** | 250/170/115/95/85/80 over 6.0-22.0 V, mode **Open Time only** |
| WUE (warmup) | all bins 419, flat 100% (= WUE OFF) | 220% at -38 °F tapering to 100% at 180 °F+ |
| Cranking enrichment | flat 100% (no enrichment), top bin 417 | 240/180/140/100 over -38 to 190 °F |
| ASE (afterstart) | all bins -40, all values 0 | 50/40/25/12% and 20/15/10/5 s over -40 to 190 °F, 5 s taper |
| Priming pulsewidth | all bins -40, all 0 ms | 5.0/4.0/2.5/1.5 ms over -40 to 109 °F |
| Flood clear level | **127.5%** — unreachable, TPS caps at 100% | **75%** |

Notes:
- **Flood clear at 127.5% could never trigger**, leaving no way to clear a flooded engine short of
  pulling plugs — worth more than the prime pulse on a first start attempt.
- **ASE at 0 is the classic "fires, runs half a second, dies" cause.** Worth having set before a
  first attempt so a stall isn't misdiagnosed as bad base fuelling.
- **Battery correction mode matters:** on **Whole PW** a 255% value multiplied the *entire*
  pulsewidth (52.4 / 2.55 = 20.5, accounting for the blowup almost exactly). **Open Time only** is
  both physically correct (voltage changes how fast an injector opens, not its flow) and a safer
  failure mode — a bad value scales ~1 ms of dead time rather than the whole pulse.
- **Set the top bin high (~190 °F), not at the axis default.** A 2D lookup *clamps* past its last
  bin, so a cranking table ending at 110 °F would keep applying 120% enrichment to a hot restart.
- **The VE table is fine** — a uniform 80% with properly populated axes is a normal untuned
  starting point, not corruption. Note cranking (~375 rpm) sits below its lowest RPM bin (800), so
  VE tuning won't affect cranking; Speeduino's cranking path has its own enrichment.

**Final validation crank (`efi_all_sensors_valid_2026-08-02.msl`):** rpm 375, CLT 82 °F, TPS 0%,
battery 12.1 V, `Gwarm` 124%, `Gammae` **188%** (= 1.24 x 1.50, exactly the designed stack),
PW **39.3 ms** steady across 157 samples, duty **24.6%** (limit 85%), **zero sync losses**.

TunerStudio's PW gauge shows red above ~30 ms, but that is default gauge scaling for port
injection — **duty cycle is the real constraint.** Long pulses are expected here: small motorcycle
throttle-body injectors feeding a two-stroke that fires every revolution.

## Required Fuel — corrected and VERIFIED (10.8 → 9.0 ms)

**Hardware, finally pinned down:**
- **Engine: Yamaha 65U — 1176 cc**, 3-cyl 2-stroke, 84 mm bore, non-power-valve 1200, 135 hp.
- **Injectors: Triumph Daytona 675 (2009-2012) throttle bodies, ~330 cc/min @ 3 bar.** Community
  bench-test figure — Triumph publishes nothing, the injectors are Bosch units made for them.
  Flow scales with sqrt(pressure), so confirm the regulator setting before trusting it.

**The Required Fuel calculator inputs were garbage AND unused:**

| Field | Was | Now |
|---|---|---|
| Engine Displacement | **350** (looks like per-cylinder typed into a total field: 1176/3 = 392) | **1176** |
| Injector Flow | **30 cc/min** (not a real injector — likely 30 lb/hr typed with cc/min selected) | **330** |
| Cylinders / AFR | 3 / 13.0 | unchanged, both correct |

**The 10.8 ms in the Required Fuel box never came from those inputs.** Running the physics on
350/30 gives ~28.5 ms, nowhere near 10.8 — so 10.8 was a stale hand-entered value and the
calculator fields were never applied. (An earlier estimate here that fuelling was "3.3x too rich"
was wrong for exactly this reason: it assumed 10.8 derived from those inputs.)

**9.0 ms is physically correct**, verified independently:
`1176/3 = 392 cc/cyl x 1.184 g/L = 0.464 g air; /13.0 = 0.0357 g fuel;
330 cc/min = 5.5 cc/s x 0.745 g/cc = 4.10 g/s; 0.0357/4.10 = 8.7 ms` ✓

So the real change is **~17% leaner, not 3.3x**.

**Verified on the bench 2026-08-03** (`bench_logs/efi_reqfuel_9ms_2026-08-03.msl`):

| | reqFuel 10.8 | reqFuel 9.0 (burned) |
|---|---|---|
| PW | 39.3 ms | **33.1 ms** (33.03-33.23 over 326 samples) |
| Duty | 24.6% | **21.0%** |
| RPM | 375 | 380 |
| Sync Loss # | 0 | **0** |

Ratio 33.13/39.24 = **0.844** vs the expected 9.0/10.8 = 0.833. The small excess is injector dead
time — a constant added *after* the proportional term, so it doesn't scale. Backing it out gives a
fixed ~2.6 ms, the right order for open time plus overheads.

> **Gotcha: a value can read correctly in TunerStudio while the ECU still runs the old one.** The
> first re-crank showed PW unchanged at 39.2 ms with the field displaying 9.0 — because it hadn't
> been **burned**. Unburned values live in RAM and are lost on any reset, including a serial
> reconnect. **Burn, then power-cycle, then re-crank** if you want to be certain you're testing
> EEPROM. Flat-but-correct-looking behaviour after a settings change is the signature.

**Injector sizing check:** at 135 hp a two-stroke burns roughly 0.5-0.6 lb/hp/hr → ~230-275 cc/min
per cylinder at full power = **70-84% duty against the 85% limit**. Adequate, but no spare capacity
if more power is ever chased. Not a concern for starting.

**Tuning discipline from here:** Required Fuel now genuinely describes the engine and injectors, so
treat it as a fixed, known-good anchor and **do all fuel tuning in the VE table instead.** If the
first start is rich, pull VE down in the low-rpm/low-load cells rather than trimming the 9.0.

**On the VE table (uniform 80%):** fine to start on, and largely irrelevant at cranking anyway since
375 rpm sits below its lowest RPM bin (800) and the cranking/WUE enrichments dominate. Expect to
tune it heavily once running — a two-stroke's effective VE swings far more with rpm than a
four-stroke's, since the expansion chamber genuinely stuffs charge back in near its tuned frequency
(realistically ~50-60% down low, past 100% on the pipe). A flat table will be right at idle and
wrong everywhere else.

**No O2 sensor — deliberate.** A wideband in a water-injected marine two-stroke exhaust can't reach
its ~350 °C light-off temperature, gets thermally shocked by steam, fouled by premix oil, and
corroded by salt water. **EGT is the correct instrument for a two-stroke** (it is what kart/PWC/sled
tuners actually use, not a fallback): mount **3-6 inches from the exhaust port, upstream of the
water injection point** — downstream readings are meaningless. **Per-cylinder, not single-point**:
on a 3-cylinder two-stroke a single lean cylinder is how a piston seizes, and a shared probe won't
see it. Target roughly **1100-1250 °F at WOT**; a steady upward trend under load is the warning
sign. This is consistent with the per-cylinder fault isolation the ignition side already has.

## Cranking speed drives trigger quality — measured 2026-08-03

Four telemetry captures of the landmark decoder (`bench_logs/ign_telemetry*.txt`,
`starter_landmark_pll_2026-07-31.txt`, `calib_2026-08-01.txt`), all same firmware family:

| session | mean rpm | rpm spread | residual stdev | edges/spark | clamped |
|---|---|---|---|---|---|
| 08-01 calibration | **437** | 33 | **47.6°** | — | — |
| 08-03 charged | 384 | **21** | **43.1°** | 3.8 | 17% |
| 07-31 | 366 | 57 | 75.3° | — | — |
| 08-03 discharged | 385 | 54 | 86.7° | — | 40% |
| 08-03 oiled + boost | **226** | 32 | 65.2° | **8.9** | 16% |

**Cranking *steadiness* matters more than raw speed.** The discharged and charged runs had the same
mean rpm (385 vs 384) but the charged run's spread was less than half — and residual stdev halved
with it (86.7 → 43.1), while clamping fell 40% → 17%. A tired battery cranks unevenly, not just
slowly.

**Below ~250 rpm the trigger degrades badly.** Pre-lubing via the plug holes cut cranking to 226 rpm
(liquid in the bores is real drag) and edges-per-spark more than doubled, 3.8 → 8.9, with `EARLY`
events tripling. This is the original documented failure mode resurfacing: the VR burst has a fixed
*duration*, so as rpm falls it occupies a larger share of each revolution and the decoder drowns in
burst edges. Crank it through until the excess oil clears before drawing conclusions from a log.

### The max-advance clamp explains "the strobe went jumpy"

`MAX_ADVANCE_BTDC = 25` with `TRIGGER_ANGLE_BTDC = 330` means the clamp engages whenever the
**measured** rev time exceeds the **model** period by more than **6.6%**:

```
minFrac   = (delta/n)   x (330-25)/360 = (delta/n)   x 0.847
fracTicks = phasePeriod x 325/360      = phasePeriod x 0.903
```

At cranking, normal residual jitter exceeds that routinely, so the clamp fires on 17-40% of revs and
pushes **retard** — the safe direction. Spark angle then reads 325-431° instead of a constant 325°.

**This is not a regression.** The clamp was added in `242b44f`, *after* the 07-31 and 08-01 captures.
Those sessions report `angle_stdev = 0.0` **by construction**: the old firmware always fired at
exactly `AFTER_EDGE` of the model, so the telemetry could not reveal model-vs-reality divergence no
matter how large (07-31 ran a residual stdev of 75°). The clamp is the first thing in this firmware
that references measured crank position, so it is the first thing that can *show* that divergence.
The underlying tracking on 08-03 charged (residual 43.1°) is the best of any session recorded.

**Do not raise `MAX_ADVANCE_BTDC` to quieten the strobe.** It is 25° against an intended 5° cranking
advance, and kickback is precisely the cranking-speed hazard it guards. Expect clamping to fall away
once running above 500 rpm, where the engine turns far more steadily than on a starter.

## Simultaneous two-board capture — the best channel diagnostic (2026-08-03)

Logging two boards **during the same crank** removes every confounder at once: identical rpm,
battery, oil and engine state, so any difference is necessarily the channel. `bench_logs/
ign_dual_2026-08-03.txt`, tagged per port. This found a fault that sequential captures could not.

| | COM9 | COM10 |
|---|---|---|
| **reported rpm** | **375** (370-377) | **492** (482-504) |
| residual stdev | 24.9° | **108.5°** |
| spark angle stdev | 8.7° | **37.0°** |
| angle range | 325-371 | **315-447** |
| clamped | 4% | **39%** |

**They disagree about engine speed in the same crank, so one is wrong.** COM9's 375 matches every
other capture; **COM10 over-reads by ~31%**, meaning it is locking onto an interval shorter than a
full revolution — a burst edge mistaken for the landmark.

Two independent confirmations it is mis-syncing rather than merely noisy:
- **Residual stdev 108.5°**, worse than the flat-battery run.
- **Its angle range starts at 315, not 325.** 315 is `AFTER_EDGE_RUN`; because COM10 believes it is
  doing 492 rpm it straddles the 500 rpm `CRANK_RPM` threshold and flips between cranking and
  running advance rev to rev. That is a *downstream symptom of the wrong rpm*, not a second fault.

### ROOT CAUSE: untwisted sensor leads — FIXED 2026-08-03

The damping resistor was **ruled out** (a later run had identical resistors on both channels and
COM10 still failed). The actual cause was wiring dress: **one channel's pulser lead was twisted with
the ground return, the other was not.**

An untwisted pair forms a loop, and that loop is an antenna for changing magnetic fields — during
cranking there is a starter drawing hundreds of amps, ignition coils, and charge coils swinging
90-100 V within inches. The induced noise adds **extra edges**, and extra edges make `refBig` decay
faster (`refBig>>6` per non-peak edge) until a ~31 ms burst interval clears the `> 0.6 x refBig`
landmark threshold and is accepted as a whole revolution.

**Fix: twist all four pulser conductors together** (3 signals + shared ground return). Result:

| | before | after |
|---|---|---|
| COM9 rpm | 374 (366-378) | 370 (363-380) |
| **COM10 rpm** | **629 (358-2017)** | **370 (364-381)** |
| COM10 impossible readings (>450 rpm) | **9/56** | **0/43** |
| COM10 residual stdev | 56.4° | 28.0° |
| COM10 angle stdev | 22.4° | **3.2°** |

Both boards now agree on engine speed, the 1900-2000 rpm cluster is gone entirely, and **COM9 was
unaffected** (angle stdev 5.8 → 5.2) — so the theoretical crosstalk penalty of bundling three
signals together did not materialise at these impedances.

> **Diagnostic chain worth reusing:** reads *high* not low → extra edges rather than missed ones →
> distribution is *bimodal* at a specific sub-interval rather than broadly noisy → a burst interval
> is being latched as the landmark → something is adding edges. That narrowed a vague "one channel
> is worse" into a single wiring defect. **Ideal would be three separate twisted pairs** (each
> signal with its own return, grounds joined at one point); twisting all four shares one return but
> proved sufficient here.

> **Method worth reusing:** a disagreement in *reported rpm* between two boards on the same crank is
> a far sharper fault signal than any single-board metric, because the true value is shared and one
> reading must be wrong. Sequential captures cannot do this — rpm, battery and engine state drift
> between runs and every difference becomes arguable.

## Cylinder ID jumpers (D10 / D11) — added 2026-08-03

Boards are byte-identical, so once they are in the harness nothing distinguishes them — and COM port
numbers renumber constantly (COM8 → COM9 → COM9/10 → COM11/12/13 in a single day), so they are
useless as identity when logging several boards at once.

**Identity comes from a jumper, not a `#define`.** A compile-time constant would produce three
different binaries and lose the "pre-flashed spare drops into any position" guarantee. Reading a
jumper keeps one image and puts cylinder identity in the harness — exactly where the 120° phasing
already lives.

| Cylinder | Jumper |
|---|---|
| 1 | nothing (both pins open) |
| 2 | **D10 → GND** |
| 3 | **D11 → GND** |

Internal pull-ups, so open = not asserted. **Read once at boot — power-cycle after changing a
jumper.** Both pins grounded reports `CYL=?` rather than guessing, so a wiring error is visible
instead of silently mislabelling.

Result: the banner reads `... CYL=2`, and every telemetry line is prefixed `cyl=2 seq=...`, making
logs self-identifying regardless of port assignment.

**The ignition path is untouched** — `cylId` is read once at boot and nothing in the decoder,
scheduling, or safety logic consults it.

## Three-channel bring-up — diagnostic log (OPEN, 2026-08-03)

With all three channels wired (leads twisted on all three), a three-board simultaneous capture:

| Port | Cyl | Sensor | rpm | residual | angle sd | impossible |
|---|---|---|---|---|---|---|
| COM11 | 1 | W/G | **372** | 18.4° | **1.4°** | 0/45 |
| COM12 | 2 | W/R | **372** | 21.2° | 7.5° | 0/43 |
| COM13 | 3 | W/B | **722** | 85.2° | 27.9° | **65/65** |

Cylinders 1 and 2 agree exactly and are done. Cylinder 3 reads ~2x true speed on every single
revolution — a *stable* fault, unlike the earlier intermittent one, so it is a structural signal
difference rather than noise crossing a threshold.

**Swap test (W/B ↔ W/G) puts it on the conditioner, not the sensor:**

| Port | now fed | rpm | impossible |
|---|---|---|---|
| COM11 | W/B | 412 | 4/49 |
| COM12 | W/R (unchanged) | 374 | 0/44 |
| COM13 | **W/G** | **495** | **49/49** |

**COM13 fails with either sensor** — including W/G, which is spotless on COM11. Cylinder 3 is the
only channel on the **second (new) conditioner board**, so that board's channel is the fault.
W/B may also be marginal (it cost COM11 its clean sheet: 372/0-impossible → 412/4-impossible), but
that is secondary and less certain.

> **Caveat on reading small differences:** COM12 was untouched between the two runs yet its residual
> went 21 → 34 and angle stdev 7.5 → 19.2. There is real crank-to-crank variation, so trust the
> 49/49-vs-0/44 distinction and treat small shifts as noise.

**Next test:** the new conditioner board is 2-channel and only one half is in use — move cylinder 3
to the unused channel. Works → that channel is faulty (component or solder). Also fails → the whole
board is suspect and wants comparing against the old one component by component.

**Not yet ruled out:** W/B pulser coil resistance (spec 248-372 Ω, compare against W/R and W/G), and
the damping R/C values on the new board versus the old — mismatched resistors have already been
found once on the old board with no explanation.

### Run-by-run record (each row = one crank, all boards logged simultaneously)

| # | configuration | cyl 1 | cyl 2 | cyl 3 |
|---|---|---|---|---|
| A | all original, cyl3 on new-ch1 | **372, 0/45** | **372, 0/43** | 722, 65/65 |
| B | W/B ↔ W/G swapped | 412, 4/49 | **374, 0/44** | 495, 49/49 |
| C | back to original, cyl3 on new-**ch2** | 412, 1/59 | **704, 66/66** | 816, 66/74 |
| D | new board **fully disconnected** | **371, 0/74** | **743, 106/106** | (floating) |

*(format: mean rpm, impossible-readings/total. "Impossible" = >450 rpm against a true ~371.)*

**What each run eliminated:**
- **B** — cyl 3's fault followed the *channel*, not the sensor: COM13 still failed on W/G, a wire
  that is spotless on cyl 1. Sensor exonerated.
- **D** — disconnecting the new board did **not** fix cyl 2, so the "new board pollutes a shared
  ground/supply" theory is **dead**. Cyl 1 simultaneously produced its best-ever result.
- **Cyl 2 broke between B and C** — it was perfect through two prior cranks and failed immediately
  after the wiring was "put back to original". Its own channel and board never changed. That dates
  the fault to a **physical connection disturbed during rewiring**, not to anything about the new
  board.

| E | after full wiring retrace | **374, 0/44** | **730, 56/56** | **725, 49/49** |

**Run E overturns the "two independent problems" reading.** Cyl 2 and cyl 3 now fail with
*near-identical* statistics — rpm 730 vs 725, residual 88.0 vs 86.9, angle stdev 28.7 vs 28.8 — on
**two different conditioner boards**. Two independent faults do not match to three significant
figures. Something common drives both.

### Leading hypothesis: CROSSTALK from bundling all three signals

Twisting all four conductors together (3 signals + shared ground) was adopted earlier because it
fixed a genuine noise problem. **It appears to have introduced a different one.**

W/R sits at 180° and W/B at 300° — 120° apart. If those two couple in the bundle, each channel sees
**its own pulse plus the other's**, one third of a revolution later: **two landmarks per rev**, which
is exactly the clean ~2x both report. W/G at 60° is evidently coupling less — different position in
the bundle, or more separation.

The timeline fits: cyl 2 was perfect through runs A and B, then failed immediately after the rewire
that bundled everything. This was an explicitly predicted risk at the time ("*if twisting all four
causes crosstalk, you would see the previously good channel degrade*") and it was not watched for
once the first fix worked.

**Fix:** separate the three signals — each twisted with **its own** ground return, grounds joined at
a single point at the source. Cheap confirmation first: physically separate the W/R and W/B runs by
a few inches without rebuilding anything and re-crank. Both returning to ~371 confirms it.

> **Lesson:** the shared-return bundle traded one failure mode for another. "Twist all four
> together" is fine for a *single* sensor pair but not for three signals sharing one return — with
> three sensors 120° apart, crosstalk manifests as an exact 2x or 3x rpm error, which looks like a
> decoder fault rather than a wiring one.

**Current state:** cyl 1 verified good (371-374 rpm, 0 impossible, repeatedly). Cyl 2 and cyl 3 both
faulty with a shared signature.

**Next:** reseat/inspect cyl 2's channel — W/R lead at the conditioner input, twist integrity,
connectors and crimps disturbed during the rewire. Cyl 1 gives a known-good reference in the same
crank.

> **A floating input produces confident-looking garbage.** With cyl 3's input disconnected, COM13
> reported 3613 rpm with residual stdev 7.0 and angle stdev 1.6 — *tighter than any real channel*.
> Never read a stable result as proof a channel is actually connected; check the rpm is plausible.

## FALSE HALF-LOCK: the real cause of the "bad channels" (2026-08-03)

After a day of wiring changes that each half-worked, the fault turned out to be **in the decoder,
not the harness**. Telemetry made it unambiguous:

| | model period | delta since last landmark | implied n |
|---|---|---|---|
| cyl 1 (good) | **161.9 ms** | 163.0 ms | **1.01** |
| cyl 2 (bad) | **81.1 ms** | 123.8 ms | 1.53 |
| cyl 3 (bad) | **85.3 ms** | 135.2 ms | 1.59 |

The bad channels had locked on **half the true period**, and **n-rounding then sustained it**: a real
landmark 161 ms later is counted as n=2, which *confirms* the wrong 80 ms period instead of
correcting it. A stable false lock, invisible to every wiring test because nothing in the harness
caused it.

**Mechanism.** A landmark is any interval > `LM_NUM/LM_DEN × refBig`. At the original **0.6**, with
`refBig` tracking the true ~90 ms landmark, anything over ~54 ms qualified — and on some channels
the *remainder* of the revolution survives as a single gap that long. Two landmarks per rev →
acquisition locks at half. The `0.6*P` refractory cannot help, because the false lock happens during
**acquisition**, before any model exists.

**Threshold tuning alone could not fix it** — the safe window differs per channel:

| threshold | cyl 1 | cyl 2 | cyl 3 |
|---|---|---|---|
| 0.6 (orig) | ✅ | ❌ 730 rpm | ❌ 725 rpm |
| **0.8** | ✅ | ✅ 373 rpm | ⚠️ 50/50 coin flip |
| 0.9 | ⚠️ rejects (FIRED 57→24) | ⚠️ rejects (57→40) | ✅ |

**Fix: threshold at 0.8 + a half-lock detector.** When the model is locked at half, real landmarks
arrive at `n==2` *consistently*; a genuinely missed landmark gives only an isolated one. So after
`HALFLOCK_STREAK` (4) consecutive `n==2` landmarks, double the model period and re-anchor.

Detection and correction only — **nothing in the firing path changed**, and if it never trips the
behaviour is identical to plain 0.8.

**Result:** all three channels now lock on the correct period (161.7 / 162.3 / 162.0 ms). Cylinder 3
tripped the detector once and was corrected — its median period went 85 ms → 162 ms.

### RESOLVED on a charged battery — all three channels clean

| Cylinder | rpm | range | residual | angle stdev | bad |
|---|---|---|---|---|---|
| 1 | 375 | 370-389 | 24.0° | 8.5° | **0/60** |
| 2 | 374 | 371-376 | 33.1° | **0.0°** | **0/28** |
| 3 | 373 | 359-384 | 31.9° | 2.7° | **0/42** |

**All three agree on engine speed within 2 rpm, zero bad readings across 130 firings.** Cyl 2's 0.0°
angle stdev means every spark landed at exactly the commanded angle.

> **Read the outliers by WHERE they occur, not how many.** The raw capture looked worse (18/80 and
> 12/82 "bad"), but plotting them against burst boundaries showed **every outlier sits in the first
> ~12-18 revolutions after a stall** — the engine spinning up from standstill while the decoder
> acquires:
> ```
> cyl1: |STALL| ******************.......................................... |STALL|
> cyl3: |STALL| ************............................ |STALL| .................. |STALL|
> ```
> That is expected behaviour, not a defect: the ÷32 frequency gain lags a hard acceleration *by
> design*, which is the same property that makes it immune to edge jitter once locked. Cyl 3's
> entire second burst was 30/30 clean. Counting acquisition transients as faults made the earlier
> "cyl 3 has 7 spurious in 50" look like a channel problem when it was mostly a tired battery plus
> a measurement artifact.

**Implication for starting:** expect a second or two of scattered timing during initial spin-up,
then it settles. The max-advance clamp bounds that window to 25° BTDC, so it is kickback-safe. Once
running above 500 rpm every factor improves — signal amplitude (3.2 V cranking → 21.1 V at 3500),
burst-as-fraction-of-revolution, and rotational steadiness.

**This is the ignition baseline to compare a running engine against.**

## Speeduino serial monitoring tools (`tools/`)

Two Python scripts (pyserial) for watching Speeduino telemetry during bench tests, added
2026-08-02. Neither has been run against the actual board yet — both are traced from the
`speeduino/speeduino` firmware source, not confirmed against Todd's board's actual firmware
responses. First real run doubles as the still-open "does the board respond to serial queries"
check from "Immediate next actions" above.

- **`tools/speeduino_monitor.py`** — polls the **secondary** serial port (Serial3,
  `secondarySerialProtocol = "Generic (Fixed List)"`) for RPM/MAP/CLT/battery/advance/TPS via the
  `'A'` command, reusing the same protocol/offsets as the `speeduino-dash` ESP32 project. Safe to
  run **alongside TunerStudio** — the secondary port's command set is deliberately restricted and
  doesn't conflict with a TunerStudio session on USB.
  ```
  python tools/speeduino_monitor.py COM7
  ```
- **`tools/composite_logger.py`** — talks to the **primary/USB** port instead, using `'J'`/`'T'`/`'j'`
  to arm and stream the composite trigger logger: 127 timestamped edges per read, each with
  pri/sec/cam pin state, which trigger fired, and sync status. This is the crank-signal-level view
  (vs. the aggregate RPM the secondary port gives), useful for cross-checking against `vr_logger`
  captures and the landmark decoder's own timing. **Requires TunerStudio closed** — only one client
  can hold the primary port, and the secondary port's protocol never implements tooth/composite
  logging regardless of its configured mode (confirmed by reading `comms_secondary.cpp`: even
  "TunerStudio protocol on secondary" mode works by redirecting the single global primary-serial
  handle, not by adding a second concurrent channel, so it's not safe to combine with a live
  TunerStudio session on USB either).
  ```
  python tools/composite_logger.py COM5
  ```

## Roadmap

- **Finish the landmark-decoder path** (see "Landmark decoder" above, on branch `experiment/longest-pulse-landmark`): get the calibration strobe bright enough, set `AFTER_EDGE_DEG` from a timing-light/strobe reading, characterize at higher rpm, then port the v5 PLL into production `one_cyl_ignition.ino`. This is the current front-runner for making the existing VR hardware work at cranking, ahead of the Hall swap.
- **~~Add Speeduino fuel-only EFI to get it started~~ — DONE 2026-08-02.** Trigger proven (D9 3-pulse/rev, zero sync losses), injection commanded on all 3 channels, all sensors + enrichment tables validated. See "EFI trigger output", "Sensor punch-list" and "Enrichment tables" above. Remaining: injector impedance check, relay for injector +12V, plumb fuel, attempt start.
- ~~Verify the ground-loop fix~~ — superseded. The remaining cranking noise turned out to be an **untwisted sensor lead**, not a ground loop; see "ROOT CAUSE". Two channels now agree on rpm with zero impossible readings, so the Hall-swap fallback is no longer indicated.
- **Confirm actual starter cranking rpm is reliably above ~50 rpm** (see "Known hardware limitation" above) — the single most important pre-fuel check given the current trigger angle, though this should be a very comfortable margin for any real starter.
- Verify `ADVANCE_BTDC` per cylinder with a timing light **once running** — precise advance is not readable at cranking, where the max-advance clamp and normal PLL jitter spread the mark. `TRIGGER_ANGLE_BTDC=330` is already confirmed three ways (strobe on all three wires, magnet geometry, and two channels agreeing on rpm).
- ~~Confirm all three flywheel magnets sit at the same angle relative to their own cylinder's TDC~~ — **DONE 2026-08-03, strobe-measured.** Each sensor sits 60° ahead of its own TDC, identically for all three. See "Pulser coil identification and positions".
- Build/verify the VR conditioner circuit against a real sensor — `pulse_simulator` only validates the ignition board's digital capture/blanking/timing logic, not the analog front end (waveform clamping, threshold, twin-pulse gap width). Use `vr_logger/vr_logger.ino` (drill-cranked, no scope required) to measure the real twin-pulse gap and pulse width, then tune `FIXED_BLANK_TICKS`/`TWIN_GAP_DEG` to match.
- Install the pin-5 pulldown resistor on the actual deployed boards (skipped during bench debug sessions where it doesn't matter, but matters for a running engine where brownouts can occur).
- **Set the BOD fuse to 4.3V before deployment.** Stock Arduino Mega fuses ship with `BODLEVEL` at 2.7V, but an ATmega2560 at 16MHz is only in spec down to 4.5V. That leaves a 2.7–4.5V window where the MCU keeps executing instead of resetting, and it can execute anything. The pin-5 pulldown's whole safety argument assumes a supply sag produces a clean reset, and at the stock fuse setting it doesn't — which matters in a marine cranking environment. Set `BODLEVEL` to 4.3V via ISP, in the same session as removing the bootloader (next item) since both need the same hardware.
- Consider flashing the deployed boards via ISP with no bootloader instead of the stock Mega bootloader. The pulldown already makes a brownout/reset electrically safe (coil goes to OFF, not stuck charging), and the firmware won't fire at a wrong angle coming out of a reset (it re-establishes sync first) — but every reset still has to sit through the bootloader's wait-for-upload delay (roughly a second or more) before the sketch even starts running again, which is long enough to fully stall a small running engine rather than just stumble through a brief dip. Removing the bootloader (flash directly via the ICSP header) makes recovery near-instant instead. Not a safety fix — a reliability one, since the failure mode either way is "stalls, needs a restart," never a hazard.
- Add per-cylinder advance curves for the top-end split (e.g. F 22° / C 19° / R 17° at 5500 rpm) — at that point the three boards stop being identical and each needs its own advance table.
- Optional future EFI integration (Speeduino) would want a proper multi-tooth crank wheel (e.g. 36-1) rather than deriving all cylinders from a single once-per-rev pulse — see "Alternative sensor architectures considered" above for why the previous Speeduino attempt's 12-1 wheel stalled and whether it's worth reviving for this project specifically.
