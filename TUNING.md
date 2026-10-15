# Tuning the robot live from Elastic

Everything tunable is published under **/Tuning** in NetworkTables. Open Elastic's NetworkTables tab (or drag entries onto a
tab) and edit values; they apply to the motors immediately, are clamped to safe ranges, and **survive reboots**.

## The tree
| Folder | What's in it |
|---|---|
| `Safety/` | **SafeMode** (turn on before testing a new gain: caps current, slows motions), SafeCurrentLimitAmps, SafeSpeedScale |
| `FourBar/Gains` `Motion` `Setpoints` `Tolerance` `Limits` | kP kI kD kS kV kA kG, Motion Magic, FlatRot/RaisedRot, tolerances, current limit, soft-limit margins |
| `FourBar/3D` | the floor linkage dimensions used to draw it in AdvantageScope |
| `IntakePivot/Gains` `Motion` `Setpoints` `Limits` `Tolerance` | same idea in degrees; `UseArmCosineGravity` switches kG to the cosine model |
| `IntakePivot/3D` | pivot point, CAD angle, direction, hopper slide travel |
| `Rollers/Intake` `Rollers/Kicker` | Fast / Slow / RevFast / RevSlow duty cycles |
| `Hopper/` | **LiftWhenBelowBalls** (how full before the floor lifts), CapacityBalls, floor-weighing calibration (Empty/FullHoldCurrentAmps) with calibrate buttons, fallback timer |
| `Outpost/` | outpost pose (X, Y, **heading 180 = facing the wall**), approach distance and speed, drive limits, pulse interval and duration (read each time you press the button) |
| `FuelSim/` | hopper capacity, robot size, intake pickup box, outtake rate/speed/spawn (sim only) |
| `Tests/` | SquareWaveDwellSeconds |

## Read-only things to graph
- `FourBar/Live/PositionRot` vs `TargetRot` (and ErrorRot, MotorVolts, StatorCurrentAmps); same under `IntakePivot/Live`.
- `Hopper/Live/EstimatedBalls` vs `LiftThresholdBalls` (how full the hopper is, from the floor's holding current), plus `FilteredHoldCurrentAmps` and `LiftThresholdHoldCurrentAmps`.

## The cool tuning aids
- **Square-wave tests** (SmartDashboard commands in Elastic): `Tuning/FourBar Square Wave` (deploys the intake first, because the floor cannot raise into a stowed intake), `Tuning/IntakePivot Square Wave`.
  They bounce between the two positions so you can change a gain and watch the response. Cancel to stop.
- **Every move is measured** (even normal driving): `FourBar/Result` and `IntakePivot/Result` give SettleSeconds, Overshoot,
  SteadyStateError, PeakCurrent and a plain-English **Advice** line ("lower kP or raise kD", ...).
- **kG finder**: with the floor still on target, `FourBar/Suggested/kG` is the kG the mechanism actually needs. Press
  `FourBar/Actions/ApplySuggestedKG` to copy it into kG. It changes with the fuel load, so try it empty and full.
- **Print what you tuned**: flip `/Tuning/Actions/PrintTunedConstants`. It prints (and saves to `tuned-constants.txt`) paste-ready Java for
  every value that differs from `Constants.java`. Do this before you leave, or the values only live on the roboRIO.
- **Reset**: `/Tuning/Actions/ResetAllToDefaults` puts everything back to the code. `Status/ValuesChangedFromCode` counts how many differ.

## Suggested order for the 4-bar
1. Turn on `Safety/SafeMode`. Check `FlatRot`/`RaisedRot` and the soft limits. If the floor lost its position, put it by hand flat on its down stop, DISABLE the robot, and press D-pad down (or the "FourBar Zero Here" button): it re-seeds the position and drives nothing.
2. Set **kG** (use the suggestion), then **kV**, then **kP/kD** with the square wave running. Watch `Result/Advice`.
3. Turn SafeMode off and raise cruise velocity / acceleration until it is as fast as is safe.
4. Repeat with a full hopper; note how kG changes (a loaded gain slot may be worth adding).
5. Press PrintTunedConstants and paste into `Constants.java`.

## Tuning when the floor lifts (floor weighing)
1. Floor flat and still, hopper **empty**: press `Hopper/Actions/SetEmptyToCurrentReading`.
2. Fill the hopper completely, floor flat and still: press `Hopper/Actions/SetFullToCurrentReading`. Set `CapacityBalls` to how many balls that is.
3. Now `Hopper/Live/EstimatedBalls` tracks the load. Set **`Hopper/LiftWhenBelowBalls`** to how many balls may still be in the
   hopper when the floor is allowed up: a low number waits for the rollers to clear more room, a high number lifts sooner.
   (`LiftThresholdHoldCurrentAmps` shows the same threshold in amps.)

## Outpost route
- The route is made from a field map (`src/main/deploy/pathplanner/navgrid.json`) that blocks both trench lanes, the hub and the towers,
  so the robot can only cross over a bump. Regenerate it with `python3 tools/generate_navgrid.py` if the robot size changes.
- The robot pathfinds to a staging point `ApproachDistanceMeters` out from the wall, then drives straight in facing the wall.
  Tune where it stops with `Outpost/PoseXMeters` (smaller = closer to the wall) and `PoseYMeters`.
- While feeding, the intake sits at `IntakePivot/Setpoints/FeedDeg` (about 10 degrees: almost deployed, slightly raised for the 1 inch bump).

## Intake / floor interlock
The intake and the hopper floor cannot hit each other: the intake may only pivot past `Interlock/IntakeClearOfFloorBelowDeg` (80 degrees; 0 = deployed)
while the floor is flat, and the floor may only raise once the intake is at or below that angle (minus `IntakeSlackDeg`). The numbers come from the CAD
(`tools/collision_map.py`) and are good to a centimeter or two, so check them on the robot. `Interlock/Enabled` turns it off; while it is holding
something back, `Interlock/IntakeHeldBackByFloor` / `FloorHeldBackByIntake` read true.
