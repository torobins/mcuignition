# MCU Ignition — 3-Cylinder Rotary Engine

Standalone electronic ignition for a 3-cylinder two-stroke engine, built from three identical Arduino Mega 2560 boards — one per cylinder. Each board reads its own VR (pulse coil) sensor, cleans up the trigger signal internally, and fires a smart ignition coil at a fixed advance angle. No shared "cleaner" board and no cross-wiring between cylinders — every board is a complete, self-sufficient ignition channel.

## Why the boards are identical

Each board only ever needs to know one thing: the angle from *its own* sensor edge to *its own* cylinder's TDC. Because the three pulse coils are mounted 120° apart on the crank, and the three TDCs are also 120° apart, the offset is the same for every cylinder — the 120° cancels out of the math. The firmware is therefore identical on all three boards; the only thing that makes "board 2" fire "cylinder 2" is that it's plugged into cylinder 2's sensor and cylinder 2's coil.

This means:
- All three boards run the exact same `.ino` — one build, flashed three times.
- A pre-flashed spare Mega can be swapped into any of the three positions.
- The 120° cylinder phasing lives entirely in the wiring harness, not in the code. **Do not cross the harness connectors between cylinders.**

This holds for fixed timing (start/idle, all cylinders at the same advance). It stops holding once per-cylinder advance curves are added (see Roadmap).

## Hardware per channel

| Signal | Connection |
|---|---|
| VR pulse coil → conditioner output | ICP5 / pin 48 (Timer5 input capture) |
| Ignition coil (Dyna D514A smart coil) | Pin 5 / PE3 — HIGH = charging, LOW = fire |
| Pulldown | 10 kΩ, pin 5 to GND, at the pin (holds coil OFF through reset/brownout) |

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

### Alternative sensor architectures considered

If the ground fix doesn't fully resolve the noise, two sensor-hardware alternatives came up:

- **Revive the old 12-1 magnet wheel + single Hall sensor**, originally built for a Speeduino attempt on this engine. That attempt stalled on Speeduino needing a separate cam sensor to disambiguate a 3-cylinder crank — a problem specific to Speeduino's 4-stroke-oriented decoder architecture, not to this engine (a two-stroke fires every revolution, so there's no 4-stroke half-speed cam ambiguity to resolve) or to this project's firmware (each board already knows its own cylinder from wiring, not from decoding cylinder identity out of a shared signal). Reviving it here wouldn't hit the same wall, but it's still the bigger change: one shared crank sensor instead of three independent per-cylinder ones (a Hall sensor failure now stalls all three cylinders, not one — a real regression from the "Why the boards are identical" fault-isolation argument above, unless the single Hall output is fanned out to all three boards, each computing its own TDC independently), plus new firmware for missing-tooth sync detection and per-board TDC-offset constants (the boards would no longer be byte-identical, same as the per-cylinder-advance-curve roadmap item below).
- **1 magnet + 3 Hall sensors** — closer to the stock Yamaha trigger config, and a much smaller change than the wheel: same one-trigger-per-cylinder-per-rev physical layout as the current VR coils, just a Hall sensor instead of a VR coil at each of the three existing sensor positions. No cam sync, no per-board TDC redesign, and it sidesteps the same ground-loop/analog-noise class of problem the wheel option does. It would also simplify the firmware: a Hall sensor sees the magnet pass as a single clean high period, not a leading-edge-plus-echo, so most of the "twin pulse" handling (bugs #5, #8, #9, #10, #11, #12 above) wouldn't apply to a from-scratch Hall version — the rate-of-change gate, 16-bit capture aliasing fix, watchdog races, and prescaler floor (bugs #1–#4, #6) would still be relevant since those are general timer/capture robustness, not VR-specific.

## Future EFI option (Speeduino) — under consideration, not decided

One direction being weighed for adding fuel injection: keep ignition entirely on the current 3 independent MCUs (unchanged), and add Speeduino purely for EFI — fuel maps and injector scheduling only, with its ignition output channels left unconfigured/unwired. This would preserve the ignition system's fault isolation (see "Why the boards are identical" above) instead of moving spark control into a centralized ECU.

Open questions if this path is taken:
- Speeduino still needs its own crank/rpm reference to schedule injection, even without controlling spark. Injection timing tolerance is generally looser than spark timing for a 2-stroke, so it doesn't need the same angular precision the ignition boards do, but it needs something. Tapping the same physical trigger signal the ignition boards use (rather than adding a second dedicated wheel) is the likely path.
- If tapping the same signal, the tap needs to be buffered/isolated (e.g. a unity buffer or optocoupler) rather than simply spliced on — otherwise Speeduino's own wiring/grounding becomes a new coupling path back into the ignition boards' sensor reference, the same class of problem as the ground-loop investigation above.
- A single shared magnet/wheel feeding 3 Hall sensors (see "Alternative sensor architectures considered" above) would map cleanly onto Speeduino's **Basic Distributor** trigger mode if ever needed for the EFI side — that mode expects exactly N pulses per revolution (one per cylinder), no missing tooth, no cam signal, which fits a 3-cylinder 2-stroke directly. The 3 Hall outputs would need to be combined onto Speeduino's single crank-input pin — trivial if the Hall ICs are open-collector (wire all three to one line with a shared pull-up), otherwise diode-OR'd.

## Roadmap

- **Verify the ground-loop fix** (single-point ground at the flywheel-casing bolt, see "Starter-cranking noise investigation" above) actually cleans up the VR signal under real starter cranking. If it doesn't, fall back to the 1-magnet/3-Hall-sensor swap discussed there before reviving the full 12-1 wheel.
- **Confirm actual starter cranking rpm is reliably above ~50 rpm** (see "Known hardware limitation" above) — the single most important pre-fuel check given the current trigger angle, though this should be a very comfortable margin for any real starter.
- Verify `TRIGGER_ANGLE_BTDC` and `ADVANCE_BTDC` per cylinder with a timing light before running on fuel.
- Confirm all three flywheel magnets sit at the same angle relative to their own cylinder's TDC (assumed, should be checked).
- Build/verify the VR conditioner circuit against a real sensor — `pulse_simulator` only validates the ignition board's digital capture/blanking/timing logic, not the analog front end (waveform clamping, threshold, twin-pulse gap width). Use `vr_logger/vr_logger.ino` (drill-cranked, no scope required) to measure the real twin-pulse gap and pulse width, then tune `FIXED_BLANK_TICKS`/`TWIN_GAP_DEG` to match.
- Install the pin-5 pulldown resistor on the actual deployed boards (skipped during bench debug sessions where it doesn't matter, but matters for a running engine where brownouts can occur).
- **Set the BOD fuse to 4.3V before deployment.** Stock Arduino Mega fuses ship with `BODLEVEL` at 2.7V, but an ATmega2560 at 16MHz is only in spec down to 4.5V. That leaves a 2.7–4.5V window where the MCU keeps executing instead of resetting, and it can execute anything. The pin-5 pulldown's whole safety argument assumes a supply sag produces a clean reset, and at the stock fuse setting it doesn't — which matters in a marine cranking environment. Set `BODLEVEL` to 4.3V via ISP, in the same session as removing the bootloader (next item) since both need the same hardware.
- Consider flashing the deployed boards via ISP with no bootloader instead of the stock Mega bootloader. The pulldown already makes a brownout/reset electrically safe (coil goes to OFF, not stuck charging), and the firmware won't fire at a wrong angle coming out of a reset (it re-establishes sync first) — but every reset still has to sit through the bootloader's wait-for-upload delay (roughly a second or more) before the sketch even starts running again, which is long enough to fully stall a small running engine rather than just stumble through a brief dip. Removing the bootloader (flash directly via the ICSP header) makes recovery near-instant instead. Not a safety fix — a reliability one, since the failure mode either way is "stalls, needs a restart," never a hazard.
- Add per-cylinder advance curves for the top-end split (e.g. F 22° / C 19° / R 17° at 5500 rpm) — at that point the three boards stop being identical and each needs its own advance table.
- Optional future EFI integration (Speeduino) would want a proper multi-tooth crank wheel (e.g. 36-1) rather than deriving all cylinders from a single once-per-rev pulse — see "Alternative sensor architectures considered" above for why the previous Speeduino attempt's 12-1 wheel stalled and whether it's worth reviving for this project specifically.
