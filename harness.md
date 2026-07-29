# Phoenix SITL tuning harness

How to run a repeatable SITL flight and compare it against the real aircraft
(`log2.bin`, ARACE Phoenix, 2026-07-27, 94 min BVLOS sortie).

Everything below is headless — no MAVProxy console, no map, no gdb window.

## Reference: what we are matching

From `log2.bin`, armed window only, segmented by smoothed climb rate.
See `PHOENIX_SITL_REFERENCE.md` in `ArduPlane/bvlos-full/` for the full derivation.

| phase        | time    | EAS m/s | throttle | current | power  | Vz m/s | Ah (share)   |
|--------------|---------|---------|----------|---------|--------|--------|--------------|
| VTOL/low alt | 12.0min |   -     |    -     | 14-17 A | 605-823W|  -    | 2.73 (14.3%) |
| climb (FW)   |  5.3min | 19.99   |  71.4 %  | 22.94 A | 1086 W | +3.01  | 2.47 (12.9%) |
| cruise       | 66.4min | 19.77   |  57.3 %  | 10.46 A |  484 W |  0.00  |11.57 (60.5%) |
| descent      |  8.1min | 19.93   |  28.0 %  |  4.50 A |  199 W | -1.96  | 0.61 ( 3.2%) |
| TOTAL        | 94 min  |         |          |10.8 mean|        |        |19.11 Ah / 39 |

Other measured values:
- wind 4.3 m/s from 341 deg (p95 6.5, max 7.2)
- all-up mass 11.5 kg (operator figure; MTOW 12.5)
- cruise L/D 8.27  (mass- and efficiency-independent: dP*V/(P_lev*Vz))
- pack internal resistance 0.041 ohm -> 0.43 V sag at cruise, 2.95 V at the 72 A peak
- fixed non-propulsive load ~85 W (Starlink Mini, LTE, FPV, CAN servos, autopilot)
- battery used 19.11 Ah of 39 Ah = **49 %**  <- headline number to match

## Prerequisites (do this before every run)

```bash
cd /data/APM.plane47/ArduPlane/bvlos-full
cp saved/eeprom.bin eeprom.bin     # orroral4.txt mission + tuned params live here
rm -rf logs                        # so logs/00000001.BIN is unambiguously this run
```

`saved/eeprom.bin` is the known-good state. **Never run the SITL binary directly with
`--defaults` from this directory** — it writes to `eeprom.bin` and will clobber the
mission. Always go through `go-headless.sh`.

## Running

```bash
cd /data/APM.plane47/ArduPlane/bvlos-full
./go-headless.sh -m "--daemon --out 127.0.0.1:14560"
```

- `--daemon` is required: without it MAVProxy sees no interactive stdin and exits
  immediately, taking the sim with it.
- `--out 127.0.0.1:14560` gives a UDP endpoint to drive the flight from a script.
- `go-headless.sh` uses the `PhoenixSITL-headless` aircraft dir, whose `mavinit.scr`
  deliberately loads no map/horizon/console modules.
- it exports `SITL_RITW_TERMINAL="bash"`. Without that, `sim_vehicle` launches the
  ArduPlane binary through `run_in_terminal_window.sh`, which spawns an
  `xterm -hold` **per run** even with `--console` and `-G` dropped. `-hold` keeps the
  window alive after the binary exits, so killing `arduplane` between runs leaves the
  xterms behind and they accumulate. Check with `pgrep -x -c xterm` (note `-x`:
  `pgrep -f xterm` also matches your own command line).

## Flying it (from a script on udp:127.0.0.1:14560)

Order matters:

1. `SIM_SPEEDUP 100` **first** — with speedup set, the EKF settles in well under a
   second of wall time and the vehicle is armable almost immediately. Without it,
   arming takes ~100 s of wall clock.
2. set mode `AUTO`
3. arm (`MAV_CMD_COMPONENT_ARM_DISARM`, param1=1), retrying for a few seconds

The orroral4 mission (71 items: VTOL takeoff to 50 m, 55 waypoints, loiters, jumps,
VTOL land) then runs on its own.

**Watch out for sim-time vs wall-time.** At `SIM_SPEEDUP 100` a `sleep(45)` in the
driving script is 4500 s of simulated flight. Size waits accordingly, or the aircraft
will be far past the leg you meant to sample.

## Measuring — from the BIN log, not the MAVLink stream

Live streaming at `set streamrate 1` is too sparse and mixes wall/sim time. Use the
dataflash log the sim writes to `logs/00000001.BIN` and analyse it exactly the way
`log2.bin` was analysed:

```python
from pymavlink import mavutil
import numpy as np
m = mavutil.mavlink_connection('logs/00000001.BIN')
rows=[]; last={}
while True:
    x = m.recv_match(type=['CTUN','BAT','ARSP','BARO'])
    if x is None: break
    t=x.get_type(); d=x.to_dict()
    if   t=='CTUN': last['thr']=d.get('ThO',0)          # NOTE: 0..1, not percent
    elif t=='BAT' and d.get('Inst',d.get('I',0))==0:
        last['I']=d.get('Curr',0); last['V']=d.get('Volt',0); last['rem']=d.get('RemPct',0)
    elif t=='ARSP' and d.get('I',0)==0: last['as']=d.get('Airspeed',0)
    elif t=='BARO' and d.get('I',0)==0: last['alt']=d.get('Alt',0)
    if len(last)==6: rows.append(tuple(last[k] for k in ('thr','as','I','V','alt','rem')))
a=np.array(rows,dtype=float)
fw = a[(a[:,1]>15)&(a[:,4]>60)]                      # fixed-wing, above 60 m
vs = np.convolve(np.gradient(fw[:,4]), np.ones(40)/40, mode='same')
cruise = fw[np.abs(vs)<0.15]
```

Key checks each run:
- `BAT[0].RemPct` at end of flight — the headline energy number (target ~50 % used)
- cruise throttle / airspeed / current vs the reference table above
- that the aircraft actually **transitioned** (`ARSP.Airspeed` reaching ~20, `CTUN.ThO`
  non-zero) rather than hovering upward

## Known traps (all of these have bitten)

- Setting `mass` in the `SIM_Plane` constructor is useless for a quadplane:
  `SIM_QuadPlane` runs afterwards and overwrites it with `frame->get_mass()*1.5`.
- A steep forward-thrust exponent that matches both cruise and climb throttle collapses
  thrust at the low throttles used during transition (0.0015 N at 5 %), so the aircraft
  hovers vertically to 9 km and never transitions.
- SITL instance N uses `eepromN.bin`; instance 0 uses `eeprom.bin`. An instance with no
  eeprom boots with **no mission** and will arm and sit there.
- `pgrep -f arduplane` matches your own shell command line; use `pgrep -x arduplane`.

## Progress log

Measured with `phoenix_measure.py`, same tool both sides. Real = log2.bin.

### 1. stock code
cruise 48.1 % thr / 9.78 A, **6 % of pack used**. Battery barely discharged: only the
VTOL current reached `battery.set_current()`.

### 2. + forward-motor discharge fix, + 12S frame
Flies orroral4 correctly, all three fixed-wing phases present.

| phase   | SITL                                    | REAL                                    |
|---------|-----------------------------------------|-----------------------------------------|
| cruise  | 51.9 min  thr 42.3%  20.00 m/s   8.46 A | 57.2 min  thr 58.6%  20.00 m/s  10.64 A |
| climb   |  5.2 min  thr 66.3%  19.96 m/s  13.25 A |  6.0 min  thr 70.9%  19.98 m/s  22.50 A |
| descent |  7.4 min  thr 25.4%  19.99 m/s   5.08 A |  9.0 min  thr 29.9%  20.25 m/s   3.64 A |
| battery | 28 % used                               | 49 % used                               |

### 3. + c_drag_p 0.1 -> 0.139  (cruise throttle matched)
cruise thr 57.7 % / 11.52 A, climb thr 81.5 % / 16.29 A, 25 % of pack used.

### 4. + forward-motor current model fitted
`SIM_FWD_THR_A` 110.07, `EXP` 5.09, `I_FIXED` 3.41, fitted to the REAL
throttle->current points so the model describes the aircraft, not SITL's error.
cruise 57.7 % / 10.09 A, 41 % of pack used.

### 5. + upstream double-EAS2TAS pitot fix (b99fdd75), drag re-derived
`SITL_State::_update_airspeed` divided by EAS2TAS although its input is already EAS,
so SITL under-reported airspeed and TECS flew faster than it believed - the drag
tuned before it was partly compensating for that. `c_drag_p` 0.139 -> 0.162.
cruise 58.0 % / 10.31 A, climb 84.1 % / 56.60 A, 45 % of pack.

### 6. + corrected descent target

The descent reference used until stage 6 was an artefact. The real descent is bimodal:
21 % of its samples sit at 0 % throttle (idle glide) and 79 % above 40 % (powered
descent). Averaging them gave "29.9 % throttle at -1.95 m/s", an operating point the
aircraft never flew; forcing a thrust curve through it produced an exponent of 4.80 and
a false conclusion that no single curve could fit the aircraft. `phoenix_measure.py` now
reports the two separately. The idle glides are the single most valuable segment: at
zero thrust, V/Vz is a **direct L/D measurement with no efficiency assumption** - 8.44.

### 7. Correct mass, measured drag polar, propeller thrust model (final)

A Codex review found the model was flying at **4.5 kg**, not 11.5 kg - `QuadPlane`
takes the generic 3.0 kg frame mass and scales it by 1.5, and nothing had overridden it.
Stages 3-6 had been fitted around that error, and the fits absorbed it: the model
reproduced the measured cruise and climb throttle while gliding at **L/D 2.6** against
the real 8.4. It matched the numbers by coincidence, not physics - and a mission planner
extrapolates, so a wrong polar is a wrong answer as soon as speed or climb rate changes.

Rebuilt from measured physics instead of fitted throttle:

- **mass 11.5 kg** (frame carries 11.5/1.5, `refCurrent` scaled with it to hold hover W/N)
- **`c_drag_p` 0.076** from the glide L/D of 8.44. At 11.5 kg and 20 m/s the wing needs
  CL 1.02, so induced drag is 0.047 of a total 0.123 - a third of it. At 4.5 kg induced
  drag looks negligible, which is why it had all landed on the parasitic term (0.162).
- **`refBatRes` 0.041** measured by regressing pack voltage on current over short
  windows. The 3S default of 0.01 sags a 72 A peak by 0.7 V instead of 3.0 V, which is
  the difference between tripping a battery failsafe and not.
- **propeller thrust model** `T = 178.0 * MAX(0, thr^2 - 0.458*thr*V/20)`. A pure
  throttle exponent needs **4.53** to span the two measured level-flight points at the
  correct mass. Independently, current against throttle in the same log has an exponent
  of **4.48**, and current is proportional to thrust at fixed airspeed - the two agree
  to 1 %, so the steepness is real. But 4.53 leaves 0.15 N at 20 % throttle and the
  aircraft cannot transition (an earlier 3.69 failed exactly this way). Putting the
  steepness in the airspeed term instead gives the same slope at 20 m/s and still 7 N
  at 20 % throttle static.
- **current model refitted** to the corrected powered points: 96.37 / 4.48 / 1.83.

| phase    | SITL                                        | REAL                                     |
|----------|---------------------------------------------|------------------------------------------|
| cruise   | 56.3 min thr **58.6%** 20.02 m/s **10.60 A** | 57.2 min thr 58.6% 20.00 m/s 10.64 A    |
| climb    |  5.8 min thr **69.5%** 20.10 m/s **21.04 A** |  6.0 min thr 70.9% 19.98 m/s 22.50 A    |
| glide    |  1.2 min thr 0.1% Vz -2.83 **L/D 7.92**      |  2.2 min thr 0.2% Vz -2.61  L/D 8.44    |
| pwr-desc |  6.4 min thr **50.3%** 20.01 m/s **6.34 A**  |  4.1 min thr 50.1% 20.08 m/s  6.20 A    |
| FW time  | **80.7 min**                                 | 82.0 min                                 |
| battery  | 43 % used                                    | 49 % used                                |

Every fixed-wing phase is within a few percent, and the whole mission now flies to
completion in 80.7 min against the real 82.0. The idle glide appears for the first time:
the propeller model gives zero thrust below ~46 % throttle at 20 m/s, so TECS actually
closes the throttle in a descent, which the pure-exponent model never did.

Remaining, in order of size:
1. **6 points of pack usage** (43 % against 49 %). Cruise energy is essentially exact
   (10.60 A against 10.64 over 56 min), so the gap is not in fixed-wing cruise - look at
   VTOL hover and the transitions, which this branch has deliberately not tuned.
2. **Zero-throttle current is 1.83 A against a real 0.61 A** - `FWD_I_FIXED` came out of
   a 3-point fit that had no glide point in it. Adding the glide as a fourth constraint
   needs a 4th degree of freedom, or a least-squares fit rather than an exact one.
3. **Glide L/D 7.92 against 8.44**, and glide airspeed runs 22.2 m/s against 20.8, so
   the polar is slightly draggy at the high-CL end.

### 8. Phoenix setup moved out of the C++ and into PhoenixSITL.parm

Everything that has a parameter now lives in `PhoenixSITL.parm`, loaded by both
`go.sh` and `go-headless.sh` via `sim_vehicle.py --add-param-file`. The C++ defaults
for `SIM_FWD_THR_A`/`_EXP`/`_I_FIXED` are back to the historic 20 / 1 / 0, so a stock
quadplane is unaffected by this branch's tuning.

What still has to stay in C++, because no parameter exists for it: airframe mass, the
drag polar, the forward thrust model (`SIM_Plane.cpp`) and the 12S pack electrics
including internal resistance (`SIM_Frame.cpp`). Both are gated on a "quadplane" frame
string.

Starting from nothing - no eeprom at all:

    rm eeprom.bin && ./go-headless.sh -m "--daemon --out 127.0.0.1:14560"
    # then, over MAVLink or at the MAVProxy prompt:
    wp load orroral4.txt
    fence load instrument_fence.txt

The fence matters: `FENCE_ENABLE` is a parameter but the polygon itself lives in
storage, so a wiped eeprom has an enabled fence with nothing in it and prearm refuses
with "Fences enabled, but none selected". Same for the mission - `MIS_TOTAL` and
`FENCE_TOTAL` are deliberately NOT in the .parm file, since setting a count without the
matching storage is worse than leaving it at zero.

Verified equivalent. A run from a wiped eeprom with only the .parm file, the mission and
the fence loaded reproduces the eeprom-based run to within noise:

| phase    | from eeprom       | from PhoenixSITL.parm | real              |
|----------|-------------------|-----------------------|-------------------|
| cruise   | 58.6 % / 10.60 A  | **58.6 % / 10.61 A**  | 58.6 % / 10.64 A  |
| climb    | 69.5 % / 21.04 A  | **69.6 % / 21.14 A**  | 70.9 % / 22.50 A  |
| glide    | L/D 7.92          | **L/D 7.93**          | L/D 8.44          |
| pwr-desc | 50.3 % /  6.34 A  | **50.2 % /  6.33 A**  | 50.1 % /  6.20 A  |
| FW time  | 80.7 min          | **80.7 min**          | 82.0 min          |

Note the .parm file is applied as *defaults*, so any value already saved in eeprom.bin
wins. Editing it and restarting does nothing unless the eeprom is deleted first.

## TRAP: SIM_BATT_VOLTAGE is coupled to VTOL thrust

`SIM_Motor.cpp:38` scales motor thrust by `voltage / voltage_max`, where
`voltage_max` is the frame model's `maxVoltage` (12.09 V by default). And
`SIM_Frame.cpp:644` sets the *default* of `SIM_BATT_VOLTAGE` from that same value.

So raising `SIM_BATT_VOLTAGE` to 50.4 for a 12S pack — which is required, otherwise
the eeprom's `BATT_CRT_VOLT 34` trips a critical-battery failsafe and the aircraft
disarms on the pad — hands the lift motors 4x their design voltage. They produce ~4x
thrust, and the aircraft climbs vertically at 34 m/s to 9 km, never transitions, and
draws 70 A.

Symptoms to recognise: `Vz` of tens of m/s, airspeed stuck near zero, `CTUN.ThO` ~0
while current is enormous.

**The frame's `maxVoltage`/`refVoltage` must be raised to 12S together with
`SIM_BATT_VOLTAGE`.** Changing one without the other gives either a disarm on the pad
or a rocket.

## Measuring

Use `Tools/autotest/phoenix_measure.py <log.BIN> [more.BIN ...]`. It resamples every
channel onto a common uniform time base from each message's own timestamp. Two things
it gets right that a naive reader does not:

- climb rate is computed against **time**, not sample index
- the time window comes only from always-present channels. `QTUN` is logged solely
  while the quadplane is active, so intersecting its range truncates the window to the
  VTOL climb-out and silently discards the whole cruise.

Validated against `log2.bin`, where it reproduces the known figures:

    cruise :  57.2 min  thr  58.6%  as 20.00 m/s  I 10.64 A  V 46.32  Vz +0.01
    climb  :   6.0 min  thr  70.9%  as 19.98 m/s  I 22.50 A  V 47.40  Vz +2.96
    descent:   9.0 min  thr  29.9%  as 20.25 m/s  I  3.64 A  V 45.33  Vz -1.95
    battery: 99 -> 50%  (49% used)

Those are the numbers to match, measured with the same tool that measures SITL.
