package frc.robot;
import static edu.wpi.first.units.Units.* ;

import edu.wpi.first.units.measure.Angle;

public class Constants {

    public static String UpperCanBus = "Upper";

        public static abstract class IntakeConstants{
            public static final int IntakeLeaderID = 53;
            public static final int IntakefollowerID = 54;
            public static final int ReversedIntakeID = 55;
            public static final int ReversedIntakeID2 = 56;

            public static final double SlowSpeed = 0.5;
            public static final double Fast = 1.0;
            public static final double RevSlowSpeed = -0.5;
            public static final double RevFast = -1.0;

            public static final int IntakePivotMotorID = 50;
            public static final Angle PivotLowSetpoint = Degrees.of(0); //based on pivot 
            public static final Angle PivotHighSetpoint = PivotLowSetpoint.plus(Degrees.of(120)); //also starting config

            // ---- 3D pose for AdvantageScope. TODO: GUESSES from the CAD, check visually ----
            // Intake pivots about this point in robot coordinates (X forward, Y left, Z up), taken from the hinge bracket on the frame in the CAD (the lower front shaft put the stowed intake below the ground).
            // Must stay in sync with "zeroedPosition" for model_1 in advantagescope/Robot_BBot/config.json.
            public static final double PivotXMeters = 0.266;
            public static final double PivotYMeters = 0.0;
            public static final double PivotZMeters = 0.187;
            // Pivot angle at which the CAD draws the intake. TODO: guess (deployed); the model is rotated by (angle - this).
            public static final Angle CadPoseAngle = PivotLowSetpoint;
            // The hopper slides forward/back with the intake: out when the intake is deployed (how the CAD is drawn),
            // retracted by this distance when stowed. TODO: GUESS (checked by eye in the sim: 0.23 stopped a little short).
            public static final double HopperSlideTravelMeters = 0.27;

        }

        public static abstract class KickerConstants{
            public static final int KickerLeaderID = 51;
            public static final int KickerFollowerID = 52;

            public static final double SlowSpeed = 0.5;
            public static final double Fast = 1.0;
            public static final double RevSlowSpeed = -0.5;
            public static final double RevFast = -1.0;
        }

        /**
         * 4-bar linkage constants. Angles are mechanism rotations of the driven link, 0 = horizontal, positive = up.
         * Values marked TODO are still placeholders / guesses and need real numbers or on-robot tuning.
         */
        public static abstract class FourBarConstants{
            // ---- CAN (TODO: confirm bus and IDs; 20/21 are unused by the other subsystems and TunerConstants) ----
            public static final String CanBus = UpperCanBus; // TODO: bus the Krakens are actually on
            public static final int LeaderID = 20;           // TODO
            public static final int FollowerID = 21;         // TODO
            public static final boolean LeaderInverted = false; // TODO: set true if positive output drives the link DOWN
            // Follower always runs opposed to the leader.

            // ---- Mechanism ----
            // Parallel 4-bar; the motors drive the back long link. Short links are 0.0660 m (2.598 in).
            public static final double GearReduction = 192.0;     // motor : driven link
            public static final double LinkLengthMeters = 0.2939; // long (driven) links, 11.569 in
            public static final double MovingMassEmptyKg = 3.67;  // 8.1 lb
            public static final double MovingMassLoadedKg = 27.2; // ~60 lb, full of balls
            public static final double CgDistanceMeters = LinkLengthMeters; // TODO: CG location unknown; assumed at the end of the driven link (worst case)

            // ---- Hard stops (physical), mechanism rotations ----
            public static final double UpStopRot = 0.173785;    // 62.562735 deg. Robot boots against this; position is seeded here
            public static final double DownStopRot = -0.010537; // -3.793269 deg
            public static final double SoftLimitMarginRot = 0.002; // soft limits sit this far inside each hard stop

            // ---- Setpoints ----
            public static final double StowRot = 0.17;      // just under the up stop
            public static final double IntakeRot = -0.008;  // just above the down stop

            // ---- Current limits ----
            public static final double StatorCurrentLimitAmps = 60.0; // TODO: confirm

            // ---- Control gains. TODO: SIM-TUNED STARTING VALUES, MUST BE TUNED ON THE REAL ROBOT ----
            // One gain slot covers empty and loaded in sim because kP is high enough to absorb the kG mismatch;
            // the sim has no friction or backlash, so re-check on the robot. If it is not good enough, add a loaded slot (Slot1).
            // Units are volts per mechanism rotation (kP, kD), volts per rot/s (kV), volts (kS, kG).
            public static final double kP = 400.0;
            public static final double kI = 0.0;
            public static final double kD = 15.0;
            public static final double kS = 0.0;
            public static final double kV = 23.0;  // ~12 V / (Kraken free speed / reduction)
            public static final double kA = 0.0;
            public static final double kG = 0.4;   // Arm_Cosine. Compromise: empty needs ~0.09 V, fully loaded ~0.7 V (CG at link end). Sim-checked for both with this one slot
            public static final double CruiseVelocityRotPerSec = 0.35;
            public static final double AccelerationRotPerSecSq = 1.5;
            public static final double JerkRotPerSecCubed = 15.0;

            // ---- atSetpoint() tolerances (setpoints are only a few thousandths of a rotation from the stops) ----
            // The single compromise kG leaves the empty link ~0.0008 rot off its target at the intake setpoint, so this has
            // to be wider than that. A separate loaded gain slot would let this be tightened.
            public static final double PositionToleranceRot = 0.002;
            public static final double VelocityToleranceRotPerSec = 0.02;

            // ---- Re-home (TODO: tune on the robot) ----
            public static final double HomingVolts = 1.5;                   // slow drive toward the up stop
            public static final double HomingStallCurrentAmps = 25.0;       // stator current that means "pushing on the stop"
            public static final double HomingStallVelocityRotPerSec = 0.01; // and basically not moving
            public static final double HomingStallDebounceSeconds = 0.15;
            public static final double HomingTimeoutSeconds = 6.0;

            // ---- 3D pose for AdvantageScope ----
            // The pivoting floor rotates about this point, in robot coordinates (X forward, Y left, Z up, origin on the
            // floor under the robot center; the CAD origin is at ground level, Z flipped). TODO: GUESS taken from the CAD (floor's front fastener, next to the intake); check it visually
            // and adjust. Must stay in sync with "zeroedPosition" in the AdvantageScope model config (see advantagescope/).
            public static final double PivotXMeters = 0.143;
            public static final double PivotYMeters = 0.0;
            public static final double PivotZMeters = 0.206;
            public static final double MountYawRadians = 0.0; // rotation of the swing plane about robot Z
            // 4-bar angle at which the CAD model draws the floor (the robot starts at the up stop = stow). The floor is
            // shown tilted by (this - current angle) about the pivot: lowering the link raises the back of the floor so
            // balls roll toward the intake. TODO: confirm the direction and amount against the real mechanism.
            public static final double CadPoseAngleRot = StowRot;
        }
}
