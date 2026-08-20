# Carrier board (3x Mega 2560 Pro ignition carrier)

Single PCB carrying all three ignition MCUs, both VR conditioners, the 5V/7V supply and
the pulser/coil terminations. Replaces the breadboard that made every bench result
unrepeatable (see README "breadboard" notes).

Layout image: `carrier_board.png`

## Board summary

| Block | Detail |
|---|---|
| MCU sockets | 3x **Mega 2560 Pro**, mounted **vertical** on right-angle headers (`CYLINDER1/2/3`) |
| VR conditioners | 2x dual-channel modules (3 channels used, 1 spare) |
| Power in | `12V+` / `-` terminal -> fuse -> TVS -> Schottky -> buck -> **8V** -> each Mega **VIN** |
| 5V rail | From **cylinder 1's Mega 5V pin** -> both VR conditioner modules |
| Pulser in | 4-pos terminal: **B, W/G, W/R, W/B** |
| Coil out | `IGN1`, `IGN2`, `IGN3` terminals (trigger signal only) |
| Strobe | `LEDC1/2/3` + series R, one per cylinder, driven from **D3** |
| Pin 48 leads | `J1`/`J2`/`J3`, 2-pin: signal + GND, flying twisted pair to each Mega |
| Mounting | 4x corner holes, M3 (3.2mm) |

## Component values (on silkscreen)

- `R1 = 470 Ohm` - VR damping resistor, across each pulser pair
- `R2 = 100 Ohm` - strobe LED series resistor
- `C1 = 10 nF` - VR filter cap, across each pulser pair
- Buck set to **7V** (Mega 2560 Pro silkscreen spec: "Power In 7-9V, peak 18V")

## Grounding topology

This is the part that took the longest to get right, so it is written down explicitly.

- **Back-side GND pour** fills the whole board.
- **VR- does NOT connect to board GND.** The conditioner inputs are differential; VR+/VR-
  go straight from the terminal to the module input. Keeping VR- off the pour is deliberate.
- **The pulser `B` (black) return is star-split at the entry terminal**, then routed as a
  parallel pair alongside each of W/G, W/R, W/B up to its conditioner input. Small loop
  area from the terminal all the way to the module.
- **Conditioner module GND (the pin beside its 5V) DOES connect to the pour.** That is the
  module's supply return, a different thing from VR-.
- **Coil primaries do not run through this board.** The coils take +12V and ground straight
  from the battery; the `IGN` terminals carry only the trigger signal. No coil discharge
  current ever crosses the pour.
- **Requires a ground bond from the board GND terminal to the same battery negative post
  the coils use.** The coil trigger input is referenced to battery negative while its drive
  signal is referenced to board ground; if those differ, the trigger threshold shifts under
  cranking load. One heavy wire, one point - not a random engine bracket.

## Harness dress (do not lose this in a tidy rebuild)

**All four pulser wires (B + the three signals) twisted as a bundle** from engine to board.
Untwisted sensor leads were the *actual* root cause of the multi-channel failures, not the
damping resistor and not the conditioner boards. A neat-looking harness that unwinds them
will bring the fault straight back.

The `J1/J2/J3` -> pin 48 flying leads are **twisted pairs** (signal + GND). On the Mega 2560
Pro, D48 sits in the isolated upper-left block with no adjacent GND; the nearest ground is
the **GND pad in the six-pad ICSP group** (`RESET/SCK/MISO/GND/MOSI/5V`). Note this differs
from a full-size Mega, which has GND at the far end of the digital header.

That flying lead carries the **conditioner's logic-level output**, not the raw pulser, so it
is far less noise-sensitive than the VR pairs - twist it anyway, but the VR pairs are where
the care matters.

## Known single point of failure

Both VR conditioner modules are powered from **cylinder 1's Mega 5V pin**.

- On the engine this is harmless: all three Megas power up together off the same buck.
- **On the bench it will mislead you.** Unplug or reset MCU1 to reflash and cylinders 2 and 3
  silently lose their trigger conditioning - presenting as two dead channels, which looks
  exactly like a decoder bug.

Recommend a silkscreen note: *"5V for both VR boards - MCU1 must be powered."*

To decouple it later, this does **not** need a second buck: a 78L05 (TO-92) or MCP1700
(SOT-23) plus two caps straight off 12V is enough, since the conditioners draw only tens of
mA. Small enough to fit the remaining space.

## Why the three Megas' 5V pins must NOT be commoned

Each Mega has its own onboard linear regulator. Tying their 5V pins together parallels three
regulators: the highest-output one hogs the load, the others sit at current limit or get
back-driven, and the result is thermal and stability problems that present as random resets.
Only **one** Mega's 5V pin may feed the conditioner rail.

Powering via **VIN (7V)** rather than injecting into the 5V pin also keeps the Mega's
USB/external auto-switch working - which matters because bench telemetry runs over USB
constantly. Injecting 5V directly would leave the buck fighting USB 5V.

## PORTE note (firmware coupling)

The strobe moved from D6/D7/D8 (PORTH) to **D3 = PE5** for PCB routing. PE5 shares **PORTE**
with the coil pin **PE3**. `STROBE_LOW()` runs in `loop()` while `COIL_HIGH()/COIL_LOW()` run
in the Timer5 compare ISRs, so a read-modify-write race on PORTE could clobber the coil bit -
a stuck-on or dropped dwell.

It is safe **only** because `STROBE_MASK` is a single compile-time bit and PORTE is in the
sbi/cbi-addressable I/O range, so gcc emits one atomic instruction. **Adding a second strobe
bit on PORTE breaks that guarantee** and would require wrapping the `loop()` write in
`ATOMIC_BLOCK`. See the comment at the `STROBE_*` defines.

Single LED is ~1/3 the light of the old 3-LED bank. If it is unreadable in daylight, do not
go back to multiple pins - drive one high-output LED from +12V through an NPN switched by D3.

## Power rails - two SEPARATE nets

| Rail | Source | Goes to |
|---|---|---|
| **8V** (net currently named `5v+`) | buck OUT | **VIN** on all three Mega headers - nothing else |
| **5V** | cylinder 1's Mega **5V pin** | both VR conditioner modules |

These must not merge. If the 8V rail ever touched a Mega 5V pin it would back-drive three
linear regulators; if it touched the conditioner 5V it would exceed the MAX9926's ~6V
absolute max and kill both modules.

**Naming hazard:** the buck output net is called `5v+` but carries **8V**. Rename to
`VIN_8V`. Anyone reading `5v+` and tapping it for a 5V device gets 8V.

## Pre-fab checklist

- [x] **Verify the LEDC nets land on D3**, not the old D6/D7/D8.
- [x] **Verify J1/J2/J3 signal pins land on the D48 net** and pin 2 on the pour.
- [x] **EFI terminal** (was `2SPEED`) renamed; fed from one cylinder's D9 only.
- [x] Power input: TVS orientation, Schottky net break (`12V+` -> `12vprot`), electrolytic
      polarity, solid pour connections on the TVS and 100uF ground pads.
- [ ] **Rename `5v+` -> `VIN_8V`** (carries 8V, not 5V).
- [ ] **Confirm terminal silkscreen matches pads.** A mislabeled pulser terminal puts that
      cylinder 120/240 deg out and will not fail obviously - same class of error as the
      W/B vs B/W transposition.
- [ ] **Mounting holes**: confirm NPTH (unplated). Plated holes tied to GND on a metal bracket
      give a second ground path back to the engine - a ground loop alongside the deliberate
      battery-negative bond.
- [ ] Re-fill zones (`B`) after any edit, then DRC.

**Deliberately NOT fixed:** `R1`, `R2`, `C1` each appear 3x. Board is hand-populated with
identical parts in each position, so duplicate refdes costs nothing at fab (gerbers carry no
refdes) or at assembly. It *will* collide if a schematic is added later and
*Update PCB from Schematic* is run.

## Buck module: MP1584EN mini

The fitted buck is an **MP1584EN** module (silver "150" inductor, SS34 freewheel diode,
trimpot-adjustable). 3A rated; actual load here is ~250mA for three Megas plus conditioners,
so current is a non-issue.

**It carries none of the input protection.** Its onboard caps are small ceramics and the SS34
is the freewheel diode, not input protection. All four of these are still required:

- **Fuse on 12V+** (inline or PTC). Nothing limits a short; a failed MP1584 shorts input to
  output.
- **Reverse-polarity protection** - series Schottky (1N5822 on hand) or a P-MOSFET. The
  MP1584 dies instantly on reverse input.
- **TVS across 12V+** (e.g. SMAJ24A). The MP1584's **absolute max input is 28V**, and a
  marine 12V system with an alternator can throw load-dump transients well past that.
- **Input bulk cap** (~100uF electrolytic) at the module input. A long battery run is an
  inductor; hot-plugging it rings the input above 12V. Pairs with the TVS, does not replace
  it. Plus 0.1uF decoupling near each module.

**Set and lock the trimpot.**

1. Set to **8V** and verify **under load, with a meter, before plugging in any Mega**.
   These modules ship at whatever the previous user set. 8V sits mid-range in the Mega 2560
   Pro's stated 7-9V window, buying margin against the pot drifting down toward brownout at
   the cost of slightly more heat in each Mega's linear regulator - a good trade.
2. Then lock it - nail polish or epoxy - or replace the feedback divider with fixed
   resistors once the value is known.

It is a single-turn pot on a board living on a running engine. Drift **up** just runs the
Megas' linear regulators hotter (Mega 2560 Pro tolerates to 18V peak). Drift **down** below
~6.2V causes **brownout resets - dead ignition, mid-run, intermittently**. That is the
failure mode to design against.

## `2SPEED` terminal = EFI trigger to Speeduino

Despite the name, this is the **D9 EFI trigger output** feeding Speeduino's primary trigger
input (pin 19 / `Crank` / VR1). **Rename it on the silkscreen** - "2SPEED" tells a future
reader nothing.

- Must come from **ONE cylinder's D9 only**. All three bussed together = three boards
  fighting one line. One board synthesises all 3 pulses/rev by subdividing its own PLL
  model (`EFI_PULSES_PER_REV = 3`).
- **10k pulldown required on the Speeduino side.** D9 floats for ~1s on every Mega reset
  (all pins come up as inputs before `setup()` runs); a floating trigger input is
  unpredictable.
- The diode in the original plan is **no longer needed** - that was for diode-OR'ing three
  board outputs, and there is now a single source.
- Speeduino decoder: **Basic Distributor**, 3 cylinders, trigger edge **RISING**.
- D9 only pulses while that board is **synced**. On the stock-CDI path the tapped board must
  still acquire lock or Speeduino sees no trigger and will not inject, even with spark.

## Pulser -> cylinder mapping (strobe-measured)

| Cylinder | Pulser wire | Sensor position |
|---|---|---|
| 1 | **W/G** (white/green) | 60 deg |
| 2 | **W/R** (white/red) | 180 deg |
| 3 | **W/B** (white/black) | 300 deg |

Each board takes its cylinder's pulser **and** that cylinder's coil. Crossing them puts the
cylinder 120 or 240 deg out. **W/B is white/black** - the manual's **B/W** is a CDI *output*
to ignition coil 2, a different wire entirely.
