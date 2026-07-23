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

## Firmware

`one_cyl_ignition/one_cyl_ignition.ino` — flash this unmodified to all three boards.

Key parameters (top of the file):

| Parameter | Value | Meaning |
|---|---|---|
| `TRIGGER_ANGLE_BTDC` | 330° | Leading trigger edge, ~30° ATDC (provisional — verify with timing light) |
| `ADVANCE_BTDC` | 15° | Fixed spark advance for start/idle (all cylinders) |
| `AFTER_EDGE_DEG` | 315° | Derived: edge-to-spark angle |
| `DWELL_US` | 3000 µs | Coil dwell (D514A) |
| `MAX_DWELL_US` | 5000 µs | Dwell watchdog ceiling |
| `STALL_US` | 300000 µs | No-edge timeout → force coil safe |
| `FIXED_BLANK_TICKS` | ~3 ms floor | Startup blanking window before a period is learned; tune to measured magnet/twin-pulse width |

The capture ISR keeps the leading edge of each pulse coil's signal and blanks the trailing-edge "twin" that a VR sensor produces per revolution, using `max(fixed floor, 3/8 of last period)`. Spark and dwell are scheduled with Timer5 output compare (`OCR5A`/`OCR5B`), computed as an integer fraction of the last measured revolution period — no floating point in the ISR path.

Safety watchdogs in `loop()`:
- Forces the coil low if dwell exceeds `MAX_DWELL_US` (prevents coil damage from an unfired charge).
- Forces the coil low and drops sync if no trigger edge arrives within `STALL_US` (engine stopped/stalled).

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

### Usage

Flash `pulse_simulator.ino` to its own Mega (or any Arduino — no special peripherals used) and open a serial monitor at 115200 baud:

| Key | Action |
|---|---|
| `c` | drop target speed to cranking rpm (default 250) |
| `i` | ramp target speed to idle rpm (default 800) |
| `s` | stop — simulates stall/kill switch, all outputs go low |
| `j` | toggle rev-to-rev speed jitter on/off |
| `+` / `-` | nudge target rpm by 50 |
| `p` | print current target/actual rpm, period, jitter state |

Suggested test sequence: start with `j` off and `c`, confirm each ignition board sees exactly one spark event per revolution (validates blanking is rejecting the trailing twin), then turn `j` on to stress-test the plausibility gate under cranking-like speed noise, then `i` to confirm behavior carries cleanly through to idle speed.

`TWIN_GAP_DEG` (default 30°) in the simulator should be kept in sync with whatever real magnet width you eventually measure, and compared against `FIXED_BLANK_TICKS` in the ignition sketch — these two values are meant to track each other.

## Roadmap

- Verify `TRIGGER_ANGLE_BTDC` and `ADVANCE_BTDC` per cylinder with a timing light before running on fuel.
- Confirm all three flywheel magnets sit at the same angle relative to their own cylinder's TDC (assumed, should be checked).
- Add per-cylinder advance curves for the top-end split (e.g. F 22° / C 19° / R 17° at 5500 rpm) — at that point the three boards stop being identical and each needs its own advance table.
- Optional future EFI integration (Speeduino) would want a proper multi-tooth crank wheel (e.g. 36-1) rather than deriving all cylinders from a single once-per-rev pulse.
