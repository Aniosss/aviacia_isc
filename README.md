# aviacia_isc

## ICS simulator tools

## Required airborne startup

The simulator does not apply control commands just because UDP telemetry reports
`AgentIsActive=1`. Start every airborne control session in this order:

1. In IOS, disable Flight Freeze, configure ILS, and enable `SCREEN -> ICS`.
2. Verify `AgentIsActive=1` and `RadioAltitude > 400 ft`.
3. Send `ModeAIReady=1` with `ControlMode=Off (0)` continuously for at least 2 seconds.
4. Keep `ModeAIReady=1` and transition `ControlMode` from `Off (0)` to `Approach (1)`.
5. Continue sending control packets at a steady rate.

`tools/ics_control_pulse.py` performs this handshake automatically. Its default
arming time is 2.2 seconds and values below 2 seconds are rejected.

Confirmed working example:

```powershell
python tools\ics_control_pulse.py --duration 12 --rate-hz 30 --aileron -5
```

During the confirmed test, this moved roll from about `-0.11` to `12.85` degrees.

The datapool does not specify a numeric range for `ElevatorCmd`, only `float32`,
unit `g`, and positive direction `+TED`. Test authority above the controller's
normal flare limit only at the 2800-ft spawn with short pulses:

```powershell
python tools\ics_elevator_authority_sweep.py
```

The tool sends one `0.25 s` pulse, holds the measured throttle position, records
pitch/rate/Nz/elevator response, and aborts below `1000 ft` or on excessive
attitude, speed, or sink rate. Test each value in a separate run after resetting
the aircraft to the same position, for example `--step 0.5` and `--step 2.0`.
Sequential pulses are not comparable because pitch and vertical speed continue
changing during recovery.

The airborne controller marks only elevator, aileron, rudder, and the two throttle
rate commands as valid (`ControlValidMask=31`). Other command fields are excluded
so zero/default ground commands and absolute throttle positions cannot conflict.

## Clear-weather baseline PID

`tools/run_ics_pid.py` adapts the clear/no-fault baseline architecture from
<https://github.com/Aniosss/aviacia/tree/master> to the ICS UDP protocol:

- localizer DDM -> intercept heading -> target roll -> aileron PID;
- glideslope DDM -> target vertical speed -> target pitch -> normal-load command;
- indicated airspeed -> symmetric throttle-rate (`deg/s`) and normalized position commands;
- adaptive reference angle of attack captured from valid flight data and only
  updated slowly while the aircraft is stabilized near the glideslope;
- filtered derivative, output limiting, and conditional anti-windup;
- automatic airborne `ModeAIReady / Off -> Approach` handshake;
- CSV logging and safe deactivation on telemetry, ICS, or ILS loss outside the
  final `80 ft` terminal window.

The flare profile is adapted from the accepted `flare_vs_hold` implementation
in [`Aniosss/aviacia` at commit `3d66dc1`](https://github.com/Aniosss/aviacia/blob/3d66dc152fb06e042dd3c99454043d83f95328e5/xp_pid_bridge/glideslope_controller.py#L5000-L5025):

- linear radio-altitude progress through roundout;
- vertical-speed target from `-2.4 m/s` to `-0.35 m/s`;
- pitch target from VS error with attitude and pitch-rate damping;
- a `6 deg` pitch ceiling and accelerated nose-up transition.

Flare changes only the pitch target. The same Pitch PID used on the approach,
with the same integral state and the same configured `-0.5..+0.5 g` output
limits, converts that target into `ElevatorCmd`. There is no flare-only
feed-forward, emergency command floor, direct elevator override, or separate
throttle/rudder law.

The simulator datapool documents `ElevatorCmd` as a longitudinal normal-load/elevator
command in `g`; the adapter stores it as `ElevatorCmdG`. `AileronCmd` and `RudderCmd`
are degree commands. Throttle positions are normalized from 0 to 1, while throttle
rates are actuator-lever rates in `deg/s` with a typical range of `-8..+8`. The speed
loop integrates its normalized demand into an absolute throttle target and converts
the transmitted rate to `deg/s`. When a run starts above VAPP, its speed setpoint
moves toward VAPP at `0.25 kt/s` instead of demanding an immediate deceleration.
Both throttle-rate inputs use the same physical sign. Absolute normalized throttle
positions are outside the documented airborne command set and are excluded by the
validity mask. Because current simulator builds may ignore that mask, both absolute
fields still carry the same shared target; per-engine rate feedback then drives both
measured lever angles toward it without creating asymmetric thrust. Lever differences
above `1 deg` use a temporary, still rate-limited synchronization boost. Console output
prints measured engine thrust as `T=L/R` and the actual difference as `dT`, separately
from lever angles `thr=L/R`.
The CSV log also records both measured throttle-lever angles and both engine
thrust values, so an ignored command can be distinguished from an aerodynamic
or energy-management problem.

Dry-run is the default and does not send commands:

```powershell
python tools\run_ics_pid.py --duration 30
```

Live control must be requested explicitly:

```powershell
python tools\run_ics_pid.py --send --duration 15
```

Add `--dashboard` to plot the live value, setpoint, controller output, and
integral term for the roll, pitch, and speed loops. The sliders update Kp, Ki,
and Kd while the controller is running:

```powershell
python tools\run_ics_pid.py --send --duration 300 --dashboard
```

Open <http://127.0.0.1:8765>. The dashboard also works in the default dry-run
mode when you only want to inspect telemetry and tune the visualization. Its
status line shows measured/reference angle of attack and left/right throttle
lever angles; the complete run remains available for scrolling after landing.

The controller keeps `ControlMode=Approach` for the entire airborne run because
changing the control mode during flare disengages the simulator autopilot. It
also keeps every transmitted mode flag identical to the high-altitude PID
approach. Flare is an internal target-profile phase only; `ModeFlare*`,
`ModeAlign*`, and `ModeRollout*` remain zero throughout the airborne run, and
the rudder command remains zero.
The pitch loop also stays continuous through flare: the same Pitch PID, including
its integral state, follows the smoothly changing flare pitch target. The target
profile starts from the configured `flare_initial_vs_fpm` reference instead of
accepting the instantaneous measured sink rate. Its elevator output stays bounded
to the same `+/-0.5 g` range used above flare altitude.
At the first weight-on-wheels indication from either main landing gear, the
runner sends no further flight command, exits the loop, and transmits only the
normal `ControlMode=Off` deactivation packet. There is no post-touchdown
`Landing` mode or nose-lowering hold.
If `AgentIsActive` drops below `80 ft` before main-gear contact, the runner logs
the event and continues sending the same airborne `Approach` packet until
touchdown instead of deactivating early. Loss of ILS validity is also tolerated
inside this terminal window.

### MC-21 approach envelope

The approach controller uses the supplied MC-21 failure-criticality appendix
for the limits that apply to approach and touchdown. Landing weight defaults to
`69277 kg` and can be overridden at startup:

```powershell
python tools\run_ics_pid.py --send --dashboard --landing-weight-kg 69277
```

For that weight, the next conservative `70000 kg` table row gives `VAPP=136 kt`
with FLAPS FULL and `VAPP=140 kt` with FLAPS 3. The controller detects those
configurations from flap angle, monitors VFE, VSR1, alpha protection, load
factor, roll, and touchdown pitch limits, and logs every envelope warning.
The baseline configuration is FLAPS 3 (`27 deg`). Live command transmission is
refused when telemetry reports a different landing-flap configuration, avoiding
a speed target that does not match the aircraft's actual aerodynamics.
For an A/B run with FLAPS FULL (`36 deg`), select FULL in IOS before activation
and start the controller with `--landing-flaps FULL`. At the default landing
weight this changes the documented `VAPP` target from `140 kt` to `136 kt`.
The alpha-protection threshold is interpolated by estimated Mach. The appendix
defines the safety envelope, not the autopilot control law, so the flare target
is an engineering profile constrained to remain inside its touchdown limits.

## Ground startup

For taxi activation, all landing gears must report weight-on-wheels and ground
speed must be below 2 knots. Hold `ModeAIReady=1` for at least 2 seconds, then
transition `ControlMode` from `Off (0)` to `Taxi (4)`. After rollout, the alternate
transition is `Rollout (3)` to `Taxi (4)`.
