# Controls and robot functions

Driver controller (Xbox, USB port 0). Buttons are standard Xbox names.

## Driving
| Input | What it does |
|---|---|
| Left stick | Drive (field-centric), translation |
| Right stick X | Rotate |
| **A** | Toggle drive speed: 80% (normal) <-> 100% (turbo). Always 75% while the intake trigger is held. Shown in Elastic under `Drive/` (`SpeedPercent`, `TurboOn (A button)`, `Mode`). |
| **Start** | Re-seed the field-centric heading (point the robot downfield first) |

## Intake / outtake
| Input | What it does |
|---|---|
| **Left trigger** (hold) | Intake: deploys the intake, runs the rollers in, floor goes flat |
| **Right trigger** (hold) | Outtake: deploys the intake, rollers out. **Does not touch the hopper floor** |
| Release a trigger | Idle: rollers stop; the intake and the floor stay exactly where they are |
| **Back** | Stow the intake (pull it in). Only with the hopper empty: pulling it in against fuel can stall the pivot. The floor goes flat first if it is up. |

## Hopper floor
| Input | What it does |
|---|---|
| **Y** | Toggle the floor: down -> all the way up; up -> down. The floor waits for the intake to be down before it lifts. |
| **D-pad up** (hold) | Manual jog: floor up |
| **D-pad down** (hold) | Manual jog: floor down. (While the robot is **disabled**, D-pad down instead zeroes the floor: only with it resting flat on its down stop.) |

## Intake pivot manual
| Input | What it does |
|---|---|
| **D-pad left** (hold) | Manual jog: deploy the intake |
| **D-pad right** (hold) | Manual jog: toward stowed |
| **X** (robot **disabled** only) | Re-zero the intake: tells it "I am stowed right now". Use with the intake physically stowed. |

Manual jogs stop at the soft limits, never run ahead of the real position by more than 6 deg / 0.012 rot, obey the intake/floor interlock, and are slowed by safe mode. Releasing holds position; the next normal control takes over.

## Outpost
| Input | What it does |
|---|---|
| **Both bumpers at once** | Drives to the outpost (over a bump, never under a trench, around the tower), slows into the wall, lifts the intake slightly, then pulses the outtake 2 s on / 2 s off. After the 4th pulse the floor raises. Press both bumpers again to cancel (the intake stays where it is). Also an Elastic button: `Outpost Feed`. |

## Safety built into the robot
* **Intake/floor interlock**: the intake cannot pivot up past 80 deg unless the floor is flat; the floor cannot lift unless the intake is clear. Applies to every control, jogs included.
* **Soft limits**: floor stays 3.6 deg short of its up stop and above its down stop; intake stays inside 0..120 deg.
* **Position-lost warning**: if a motor reboots or a reading jumps, the controller rumbles 1.5 s and `Warnings/PositionLost` goes true. Re-zero (floor: D-pad down while disabled; intake: X while disabled).
* **Safe mode** (`Tuning/Safety/SafeMode`): caps current limits and scales speeds. Use it for any first test.
* **No automatic re-homing**: nothing drives a mechanism into a hard stop to find its zero. Zero only happens at boot (robot must start stowed / flat) or by the disabled-only buttons.

## Elastic / dashboard
* `Auto Mode` chooser, `Field`
* `Drive/...` speed and mode
* `Tuning/...` everything tunable (see TUNING.md): gains, setpoints, limits, speeds, outpost numbers (`Outpost/PulseOnSeconds`, `PulseOffSeconds`, `PulsesBeforeFloorRaise`, pose, speeds), `Manual/IntakeDegPerSec`, `Manual/FloorRotPerSec`, `Drive/*SpeedScale`
* Buttons: `Outpost Feed`, `FourBar Raise`, `FourBar Flat`, `FourBar Zero Here`, `Intake Deploy`, `Intake Stow`, `Tuning/FourBar Square Wave`, `Tuning/IntakePivot Square Wave`, `Tuning/Actions/ResetAllToDefaults`, `Tuning/Actions/PrintTunedConstants`
* Warnings: `Warnings/PositionLost`, `Warnings/PositionLostWhere`

## Autonomous named commands (PathPlanner events)
`Intake`, `Poop` (outtake, rollers only), `Idle` (rollers off, intake and floor stay), `Jerk` (quick reverse with the intake down).

## Robot structure (code)
* `SuperStructure`: the state machine (INTAKE, POOP, JERK, FEED, FEED_PULSE, IDLE, STOWED) that coordinates:
  * `IntakeHandler` (rollers + kicker), `IntakePivotHandler` (intake pivot: STOWED / FEED / ENGADGED + manual jog),
  * `FourBarHandler` (hopper floor: FLAT / RAISED + manual jog)
* `MechanismInterlock`: the intake/floor collision rules
* `tuning/`: live tunables over NetworkTables, telemetry, safe mode
* Simulation: `FuelSimulation` (fuel, 160-ball capacity), 3D model for AdvantageScope, and the full-robot sim tests in `src/test`
