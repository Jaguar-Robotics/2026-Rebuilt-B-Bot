package frc.robot;
import static edu.wpi.first.units.Units.* ;

import edu.wpi.first.units.measure.Angle;

public class Constants {

    public static String DriveBus = "Drive";

        public static abstract class IntakeConstants{
            public static final int IntakeLeaderID = 30; //lower left 
            public static final int IntakefollowerID = 31; //top left 
            public static final int ReversedIntakeID = 32; //lower right
            public static final int ReversedIntakeID2 = 33; //top right

            public static final double SlowSpeed = 0.5;
            public static final double Fast = 1.0;
            public static final double RevSlowSpeed = -0.5;
            public static final double RevFast = -1.0;

            public static final int IntakePivotMotorID = 20;
            // The real motor turns the other way from the code's convention (0 deg = deployed, 120 deg = stowed): after seeding 120 at stowed,
            // the deployed intake read 235 deg. Inverting makes deploying decrease the angle. Real robot only (the sim already uses the code's convention).
            public static final boolean PivotInverted = true;
            public static final Angle PivotLowSetpoint = Degrees.of(0); //based on pivot 
            public static final Angle PivotHighSetpoint = PivotLowSetpoint.plus(Degrees.of(120)); //also starting config
            // Feeding the outpost: the intake is almost fully deployed (0 deg) and pressed against the wall, raised a little so the balls
            // clear the 1 inch bump at the outpost. TODO: tune on the robot
            public static final Angle PivotFeedSetpoint = PivotLowSetpoint.plus(Degrees.of(10));

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
            public static final int KickerLeaderID = 23;
            public static final int KickerFollowerID = 24;
            public static final boolean LeaderInverted = true; // TODO: confirm on the robot (it spun the wrong way when this was false)

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
            public static final String CanBus = DriveBus; // TODO: bus the Krakens are actually on
            public static final int LeaderID = 21;           
            public static final int FollowerID = 22;
            public static final boolean LeaderInverted = false; // TODO: set true if positive output drives the link DOWN
            // Follower always runs opposed to the leader.

            // ---- Mechanism ----
            // 4-bar with the hopper floor as the coupler; the motors drive the back long link (angle 0 = that link horizontal,
            // pointing back). Short links are 0.0660 m (2.598 in). Link up = floor RAISED, link down = floor FLAT.
            public static final double GearReduction = 106.667;   // each side: motor : driven link (106.667:1, confirmed by the team after the first on-robot test)
            public static final double LinkLengthMeters = 0.2939; // long (driven) links, 11.569 in
            public static final double MovingMassEmptyKg = 3.67;  // 8.1 lb
            public static final double MovingMassLoadedKg = 27.2; // ~60 lb, full of balls
            public static final double CgDistanceMeters = LinkLengthMeters; // TODO: CG location unknown; assumed at the end of the driven link (worst case)

            // ---- Hard stops (physical), mechanism rotations ----
            public static final double UpStopRot = 0.173785;    // 62.562735 deg. Link up: the floor is RAISED (tilted for outtake)
            public static final double DownStopRot = -0.010537; // -3.793269 deg. Link down: the floor is FLAT. The robot boots against
                                                                // this stop and the position is seeded here (the Back button re-seeds it while disabled)
            public static final double SoftLimitMarginRot = 0.002; // the down soft limit sits this far inside the down stop
            public static final double UpSoftLimitMarginRot = 0.010; // the up soft limit sits this far (3.6 deg) inside the up stop, so nothing can command the floor into it

            // ---- Setpoints ----
            // The floor is flat except while outtaking.
            public static final double FlatRot = -0.008;    // just above the down stop: floor flat (default, boot, intaking, stowed)
            public static final double RaisedRot = 0.16;    // 5 deg under the up stop (0.17 bent the SRPP plates a little on the real robot): floor tilted up for outtake

            // ---- Current limits ----
            public static final double StatorCurrentLimitAmps = 40.0; // per motor. With 106:1 on each side even this is a lot of force at a hard stop, so keep it low

            // ---- Control gains. TODO: SIM-TUNED STARTING VALUES, MUST BE TUNED ON THE REAL ROBOT ----
            // One gain slot covers empty and loaded in sim because kP is high enough to absorb the kG mismatch;
            // the sim has no friction or backlash, so re-check on the robot. If it is not good enough, add a loaded slot (Slot1).
            // Units are volts per mechanism rotation (kP, kD), volts per rot/s (kV), volts (kS, kG).
            public static final double kP = 400.0;
            public static final double kI = 0.0;
            public static final double kD = 15.0;
            public static final double kS = 0.0;
            public static final double kV = 12.8;  // ~12 V / (Kraken free speed / reduction) = 12 / (100 rps / 106.667)
            public static final double kA = 0.0;
            public static final double kG = 0.45;  // Arm_Cosine. Re-measured in the sim at 106.667: empty needs ~0.08 V, fully loaded ~0.62 V. Leaning toward loaded so a full floor does not sag toward the down stop. Compromise: empty needs ~0.09 V, fully loaded ~0.7 V (CG at link end). Sim-checked for both with this one slot
            public static final double CruiseVelocityRotPerSec = 0.22;
            public static final double AccelerationRotPerSecSq = 0.7;
            public static final double JerkRotPerSecCubed = 7.0;

            // ---- atSetpoint() tolerances (setpoints are only a few thousandths of a rotation from the stops) ----
            // The single compromise kG leaves the empty link ~0.0008 rot off its target at the intake setpoint, so this has
            // to be wider than that. A separate loaded gain slot would let this be tightened.
            public static final double PositionToleranceRot = 0.002;
            public static final double VelocityToleranceRotPerSec = 0.02;

            // ---- 3D pose for AdvantageScope: the floor is the coupler of the 4-bar, so its pose is solved from the linkage ----
            // Robot coordinates: X forward, Z up, origin on the floor under the robot center. TODO: from the CAD / pictures; check visually.
            // The floor's two pivot points in the FLAT pose (the CAD model is drawn flat). They are 0.318 m apart, which matches the
            // coupler length read off the side-view pictures.
            public static final double FloorBackPivotX = -0.170;   // where the long (driven) link attaches
            public static final double FloorBackPivotZ = 0.151;
            public static final double FloorFrontPivotX = 0.143;   // where the short link attaches
            public static final double FloorFrontPivotZ = 0.206;
            public static final double ShortLinkLengthMeters = 0.066;
            // Fixed pivot of the short link relative to the motor-driven hub, from the side-view picture. At the flat pose this
            // puts the short link at 0.0666 m, matching ShortLinkLengthMeters.
            public static final double ShortLinkGroundDX = 0.0854;
            public static final double ShortLinkGroundDZ = 0.0248;
        }

        /** Fuel simulation (simulation only). TODO: robot size, intake zone and outtake values are guesses from the CAD. */
        public static abstract class FuelConstants{
            public static final int HopperCapacity = 160;

            // Robot body for fuel collisions (centered on the robot pose)
            public static final double RobotWidthMeters = 1.06;   // left-right
            public static final double RobotLengthMeters = 0.65;  // front-back
            public static final double BumperHeightMeters = 0.13; // fuel above this height is not picked up

            // Intake pickup zone, robot coordinates (X forward, Y left): the roller carriage out front
            public static final double IntakeMinX = 0.30;
            public static final double IntakeMaxX = 0.70;
            public static final double IntakeMinY = -0.40;
            public static final double IntakeMaxY = 0.40;

            // Outtake: fuel leaves the front of the intake while the outtake (POOP) state is active
            public static final double OuttakeRatePerSecond = 40.0; // a full 160 hopper empties in about 4 seconds
            public static final double OuttakeSpawnX = 0.62;
            public static final double OuttakeSpawnHalfWidthY = 0.30;
            public static final double OuttakeSpawnZ = 0.20;
            public static final double OuttakeSpeedMetersPerSecond = 2.5;
        }

        /**
         * Drive-to-outpost and feed. The pose is where the robot's center ends up, in blue-alliance coordinates (flipped for red).
         * The robot arrives FACING THE WALL (the intake is the front), driving straight in from a staging point.
         */
        public static abstract class DriveSpeedConstants{
            public static final double NormalScale = 0.80; // driving with nothing going on
            public static final double IntakeScale = 0.6; // while intaking
            public static final double TurboScale = 1.00;  // after th e Back button toggle
        }

        public static abstract class OutpostConstants{
            public static final double OutpostXMeters = 0.68;       // intake rollers reach ~0.6 m in front of the center: ~8 cm off the wall. TODO: tune
            public static final double OutpostYMeters = 0.65;
            public static final double OutpostHeadingDegrees = 180.0; // blue wall is at x = 0, so face -x
            public static final double ApproachDistanceMeters = 1.2;  // staging point this far straight out from the pose, along the heading

            // Pathfinding limits while driving there (the auto paths use 3.5 m/s and 7 m/s^2)
            public static final double MaxVelocityMetersPerSecond = 3.0;
            public static final double MaxAccelerationMetersPerSecondSq = 3.5;
            public static final double MaxAngularVelocityDegreesPerSecond = 540.0;
            public static final double MaxAngularAccelerationDegreesPerSecondSq = 720.0;
            // The slow straight run into the wall
            public static final double ApproachVelocityMetersPerSecond = 1.0;
            public static final double ApproachAccelerationMetersPerSecondSq = 1.5; // gentle: slows down smoothly into the wall

            // Final alignment: after the approach path, creep onto the exact pose (the path follower stops a few centimeters early)
            public static final double AlignKp = 3.0;                  // m/s per meter of error
            public static final double AlignMaxSpeedMetersPerSecond = 0.4;
            public static final double AlignHeadingKp = 5.0;           // rad/s per radian of error
            public static final double AlignToleranceMeters = 0.03;
            public static final double AlignToleranceDegrees = 2.0;
            public static final double AlignTimeoutSeconds = 3.0;

            // Give up (and stow) if the robot has not arrived this long after the button was pressed, or is not within this
            // distance of the outpost pose when the drive command ends.
            public static final double PathfindTimeoutSeconds = 25.0;
            public static final int MaxApproachAttempts = 3;           // if bumped off course and not there when an attempt ends, re-plan from where it is
            public static final double ArrivedToleranceMeters = 0.35;

            // Feeding: wait this long, then run the outtake rollers for a short pulse, repeat. TODO: tune
            public static final double PulseOffSeconds = 2.0;          // rollers off between pulses
            public static final double PulseOnSeconds = 2.0;           // each pulse: rollers run this long
            public static final int PulsesBeforeFloorRaise = 4;        // the hopper floor lifts after this many pulses
            public static final double IntakeLiftTimeoutSeconds = 2.0; // wait at most this long for the intake to reach its slightly-raised position
        }

        /** Where the robot's pose starts in simulation (blue-alliance field coordinates). The real robot has no pose until an auto or vision sets one. */
        public static abstract class SimConstants{
            public static final double StartXMeters = 3.0;
            public static final double StartYMeters = 4.0;
            public static final double StartHeadingDegrees = 0.0;
        }

        /**
         * Keeps the intake and the hopper floor from hitting each other. From the CAD (tools/collision_map.py): with the floor flat the intake
         * is clear all the way up to about 80 degrees; from about 85 degrees to stowed it is within a centimeter or two of the floor, and a raised
         * floor would hit a stowed intake. So: the intake may only go past IntakeClearBelowDeg when the floor is flat, and the floor may only
         * raise once the intake is at or below it. TODO: confirm on the robot (the CAD pivot and linkage are estimates good to a centimeter or two).
         */
        public static abstract class InterlockConstants{
            public static final boolean Enabled = true;
            public static final double IntakeClearBelowDeg = 80.0;
            public static final double FloorCountsAsFlatBelowRot = FourBarConstants.FlatRot + 0.010;
            public static final double IntakeSlackDeg = 3.0; // the floor waits until the intake is this far inside the clear zone
        }
}
