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

### Optional bench LED indicator (no firmware change)

An LED + ~220-330 Ω series resistor tapped straight onto pin 5 (anode to pin 5, cathode to GND, in parallel with the existing 10 kΩ pulldown) blinks once per revolution whenever the board is actually issuing charge/fire commands — no code needed, since you're just observing the real coil-control signal. The 3 ms dwell window is long enough to read as a clear flash by eye (unlike the sub-millisecond pulse-coil edges on the simulator board), visible at both cranking and idle speed. Useful as a quick "is this board actually firing" check; disconnect before the real bench spark test so nothing extra is sharing the coil trigger line.

## Firmware

`one_cyl_ignition/one_cyl_ignition.ino` — flash this unmodified to all three boards.

`one_cyl_ignition_debug/one_cyl_ignition_debug.ino` — byte-for-byte the same ignition logic, plus non-blocking serial telemetry (115200 baud) that logs every capture-ISR decision (`BLANKED`/`REJECTED`/`RATEJUMP`/`SYNCFIRST`/`FIRED`/`UNSCHED`) and watchdog trip. Use this for bench bring-up or diagnosing sync/timing issues; flash the plain (non-`_debug`) sketch for actual engine use, since it has zero Serial overhead.

Key parameters (top of the file):

| Parameter | Value | Meaning |
|---|---|---|
| `TRIGGER_ANGLE_BTDC` | 330° | Leading trigger edge, ~30° ATDC (provisional — verify with timing light) |
| `ADVANCE_BTDC` | 15° | Fixed spark advance for start/idle (all cylinders) |
| `AFTER_EDGE_DEG` | 315° | Derived: edge-to-spark angle |
| `DWELL_US` | 3000 µs | Coil dwell (D514A) |
| `MAX_DWELL_US` | 5000 µs | Dwell watchdog ceiling |
| `STALL_US` | 1,500,000 µs | No-edge timeout → force coil safe. Must stay comfortably above the period of the slowest speed the ignition is expected to run at (see `PERIOD_MAX_TICKS` below) or the watchdog force-drops sync between every legitimate slow revolution. |
| `PERIOD_MAX_TICKS` | ~1,100,000 µs (~54.5 rpm) | Slowest period the capture ISR will accept as plausible. Chosen with margin below the true ~50rpm scheduling ceiling (see "Known hardware limitation" below) so the measurement gate and the scheduling gate line up. Safe at this speed only because capture timestamps are 32-bit extended (see below) — a bare 16-bit register can't represent periods this long. |
| `FIXED_BLANK_TICKS` | ~3 ms floor | Startup blanking window before a period is learned; tune to measured magnet/twin-pulse width |

Timebase is Timer5 at `/256` prescale (16 MHz → 16 µs/tick). The capture ISR keeps the leading edge of each pulse coil's signal and blanks the trailing-edge "twin" that a VR sensor produces per revolution, using `max(fixed floor, 3/8 of last period)`. Once synced, a rate-of-change gate also rejects any interval outside roughly 0.5×–2× the last measured period (catches a dropped or spurious extra edge immediately, rather than accepting it as a legitimate speed change). Spark and dwell are scheduled with Timer5 output compare (`OCR5A`/`OCR5B`), computed as an integer fraction of the last measured revolution period — no floating point in the ISR path.

Safety watchdogs in `loop()`:
- Forces the coil low if dwell exceeds `MAX_DWELL_US` (prevents coil damage from an unfired charge).
- Forces the coil low and drops sync if no trigger edge arrives within `STALL_US` (engine stopped/stalled).
- Hardware watchdog timer (AVR WDT): if `loop()` itself ever wedges (a firmware hang, not covered by the two software watchdogs above, which depend on `loop()` still running), the MCU force-resets and recovers rather than leaving the coil in whatever state it was in indefinitely. Production sketch uses `WDTO_15MS` (loop normally completes in low microseconds, so this is huge margin); the debug build uses a longer `WDTO_250MS` because its `Serial.print` calls can block for a few ms at high event rates and a tight WDT there would falsely reset mid-bench-test instead of only catching genuine hangs. Both sketches disable the WDT immediately on boot via a `.init3` init function before it's deliberately re-enabled in `setup()` — this is required on this MCU/bootloader combination, since the stock Mega bootloader doesn't clear `MCUSR`/disable the WDT on entry, so without the early-disable a WDT-triggered reset can trap the board in a boot loop rather than actually recovering. Note this only covers a firmware hang — it has no effect on brownout/power-loss recovery time, which is dominated by the bootloader's own wait-for-upload delay on every reset (see Roadmap).

### Bugs found and fixed during bench bring-up (2026-07-26)

Bench testing with `pulse_simulator` surfaced two real correctness bugs, both confirmed on real hardware before and after the fix:

1. **16-bit capture aliasing at low rpm.** `ICR5` (Timer5's input-capture register) is only 16 bits, so a raw `cap - lastCapture` difference can only represent periods up to ~262 ms (~229 rpm) before silently wrapping to a wrong, too-small value. Below that speed the ignition board would either lock onto a stable but wildly wrong interpretation of the signal (observed firing at a computed "2400–4600 rpm" while the real speed was ~200 rpm) or get stuck in a permanent stall/resync loop producing zero sparks (observed at 100–150 rpm). Fixed by replacing the raw 16-bit capture with a proper 32-bit extended timestamp (`extendCapture()`): the 16-bit register plus a software-tracked overflow count, combined with the standard capture-vs-overflow race handling for this MCU's interrupt priority order. Interval measurement is now correct from ~10,000 rpm down to `PERIOD_MAX_TICKS`'s floor — verified by a scripted rpm ramp against `pulse_simulator`.
2. **Unsigned-subtraction race in the `loop()` watchdogs.** Both the stall and max-dwell checks computed `now - lastEdge` (or `now - highAt`) as unsigned `uint32_t` math. A capture interrupt landing in the few-microsecond gap between reading `now` and reading the shared timestamp could make the timestamp *newer* than `now`, underflowing the subtraction to ~4.29 billion and instantly (and falsely) tripping the watchdog — observed causing sync to drop on almost every single revolution. Fixed by casting both differences to `int32_t` before comparing, the standard idiom for handling both real 32-bit timer wraparound and small negative races safely.
3. **No hardware watchdog.** Both software watchdogs above depend on `loop()` still running, so a genuine firmware hang had no recovery path at all — the coil could stay in whatever state it was in (e.g. stuck charging) indefinitely. Fixed by adding the AVR hardware WDT (see the `loop()` watchdogs list above for the timeout/bootloader details) — verified with a 60s live bench run showing zero unexpected resets.
4. **Prescaler-limited scheduling floor.** `OCR5A`/`OCR5B` are 16-bit compare registers, so the edge-to-spark delay (`fracTicks`) has to fit in 16 bits to be scheduled correctly — at the original `/64` prescale (4µs/tick) that capped real minimum speed at ~200rpm; below that, the ISR safely declined to fire (`fracTicks > 0xFFFF`) rather than risk a wrong-angle spark, but also created an asymmetry where `PERIOD_MAX_TICKS`'s old ~80rpm floor let the ISR keep *measuring* correctly in the 80–200rpm range while never actually firing there. Moved the prescaler to `/256` (16µs/tick), which drops the true scheduling floor to ~50rpm — cost is coarser angular resolution (0.67° at 7000rpm, 0.08° at idle), negligible for this application. `PERIOD_MAX_TICKS` was raised to match (~54.5rpm) so the two gates line up again. Directly verified on hardware: temporarily set the simulator to a sustained 65rpm (previously impossible to fire at all) and confirmed clean, correct `FIRED` events every revolution.
5. **Rate-of-change plausibility gate added.** The existing absolute-bounds gate (`PERIOD_MIN/MAX_TICKS`) would accept a single dropped edge (~2× the true period) or a spurious extra edge (~0.5× or less) as a "legitimate" speed change, since 2× a normal period is often still inside the absolute bounds — this self-corrected on the next capture, but could schedule one or two wrong-angle sparks in the meantime. Added a check that also rejects anything outside roughly 0.5×–2× the last measured period once synced, catching this immediately.
6. **Compare-register write-ordering race.** The spark-scheduling code cleared `TIFR5`'s compare-match flags *before* writing the new `OCR5A`/`OCR5B` values. If `TCNT5` happened to pass through the stale leftover `OCR5A` value from the previous revolution in that narrow window, the flag would be set again, and the immediately-following `TIMSK5` enable would fire `COMPA` right away on the stale match — dropping the coil low and eating that revolution's real spark. Fixed by writing the compare registers first, then clearing flags, then enabling the interrupt (safe because `fracTicks` is always far enough out that the real match can't have already passed).

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
| `p` | print current target/actual rpm, period, jitter state |

Suggested test sequence: start with `j` off and `c`, confirm each ignition board sees exactly one spark event per revolution (validates blanking is rejecting the trailing twin), then turn `j` on to stress-test the plausibility gate under cranking-like speed noise, then `i` to confirm behavior carries cleanly through to idle speed.

`TWIN_GAP_DEG` (default 30°) in the simulator should be kept in sync with whatever real magnet width you eventually measure, and compared against `FIXED_BLANK_TICKS` in the ignition sketch — these two values are meant to track each other.

## Roadmap

- **Confirm actual starter cranking rpm is reliably above ~50 rpm** (see "Known hardware limitation" above) — the single most important pre-fuel check given the current trigger angle, though this should be a very comfortable margin for any real starter.
- Verify `TRIGGER_ANGLE_BTDC` and `ADVANCE_BTDC` per cylinder with a timing light before running on fuel.
- Confirm all three flywheel magnets sit at the same angle relative to their own cylinder's TDC (assumed, should be checked).
- Build/verify the VR conditioner circuit against a real sensor — `pulse_simulator` only validates the ignition board's digital capture/blanking/timing logic, not the analog front end (waveform clamping, threshold, twin-pulse gap width). Measure the real twin-pulse gap on a scope and tune `FIXED_BLANK_TICKS`/`TWIN_GAP_DEG` to match.
- Install the pin-5 pulldown resistor on the actual deployed boards (skipped during bench debug sessions where it doesn't matter, but matters for a running engine where brownouts can occur).
- Consider flashing the deployed boards via ISP with no bootloader instead of the stock Mega bootloader. The pulldown already makes a brownout/reset electrically safe (coil goes to OFF, not stuck charging), and the firmware won't fire at a wrong angle coming out of a reset (it re-establishes sync first) — but every reset still has to sit through the bootloader's wait-for-upload delay (roughly a second or more) before the sketch even starts running again, which is long enough to fully stall a small running engine rather than just stumble through a brief dip. Removing the bootloader (flash directly via the ICSP header) makes recovery near-instant instead. Not a safety fix — a reliability one, since the failure mode either way is "stalls, needs a restart," never a hazard.
- Add per-cylinder advance curves for the top-end split (e.g. F 22° / C 19° / R 17° at 5500 rpm) — at that point the three boards stop being identical and each needs its own advance table.
- Optional future EFI integration (Speeduino) would want a proper multi-tooth crank wheel (e.g. 36-1) rather than deriving all cylinders from a single once-per-rev pulse.
