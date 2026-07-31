# Yamaha 65U Stator / Electrical Specs (factory service manual)

Transcribed from the factory "Electrical System" pages (ELEC section 7) so the
numbers are on hand without digging up the photos. Reference tool: Peak Voltage
Adapter YU-39991 / 90890-03169.

## Peak Voltage Measurement Chart

| Component (test harness) | Connector color | Open cranking | Cranking | Connected 2000 rpm | 3500 rpm |
|---|---|---|---|---|---|
| A — Charge Coil Output (06777) | L–B/R | 90.2 | 90.9 | 95.9 | 97.1 |
| A — Charge Coil Output (06777) | Br–B/R | 22.3 | 21.0 | 46.4 | 65.2 |
| B — Lighting Coil Output | G–G | 5.6 | 5.6 | 23.8 | 28.1 |
| C — Rectifier Regulator Output | R–B | — | 5.0 | 23.0 | 27.3 |
| D — Pulser Coil Output (06778) | W/B–B, W/G–B, W/R–B | 3.2 | 2.4 | 11.1 | 21.1 |
| E — CDI Output | B/W–B | — | 95.5 | 100.5 | 101.4 |

Note (from manual): specified values indicate the *lower* limit — component is OK
when readings are equal or higher. "—" = not specified for that condition.

## Coil Resistance Specs (at 68°F / 20°C)

**Charge Coil** — one center-tapped winding, 3 leads (L=blue end, Br=brown end,
B/R=black/red center tap):
- Black/Red (B/R) – Brown (Br): 172.0–258.0 Ω  (lower-voltage segment)
- Black/Red (B/R) – Blue (L):   656.0–984.0 Ω  (higher-voltage segment, ~90–101V)

**Pulser Coil** — three independent coils, one per cylinder, sharing a common
Black (B) return (factory config is already per-cylinder, same fault-isolation
philosophy as this project's 3 independent ignition boards):
- White/Red (W/R) – Black (B):   248–372 Ω
- White/Black (W/B) – Black (B):  248–372 Ω
- White/Green (W/G) – Black (B):  248–372 Ω

## Safety / usage notes
- Charge coil open-circuit leads carry ~90–100V+ at any speed — short all three
  leads together to disable safely (see README "Charge coil" section).
- Pulser coil peak-voltage column (2.4V cranking → 21V at 3500 rpm) is the OEM
  trigger-signal level — useful reference for what a healthy pulse signal looks
  like, distinct from the VR-conditioner noise investigation.
