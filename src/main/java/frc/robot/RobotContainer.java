// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Handlers.FourBarHandler;
import frc.robot.Handlers.IntakePivotHandler;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningTelemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import frc.robot.Handlers.SuperStructure;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SimAwareXboxController joystick = new SimAwareXboxController(0); // normal Xbox controller on the robot; remaps axes in sim

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision = new Vision(drivetrain);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final SuperStructure superStructure = SuperStructure.getInstance(); //creates a new SuperStructure; try to create all handlers in superstructure and each of their subsystems in their handlers, but like sometimes that wont work so its whateva

    public final FourBarHandler fourBarHandler = FourBarHandler.getInstance();

    public final Field2d field = new Field2d();
    private String previewedAuto = null;
    private boolean previewedRed = false;


    /** Fuel on the field, simulation only (null on the real robot). */
    private final FuelSimulation fuelSimulation = RobotBase.isSimulation()
        ? new FuelSimulation(drivetrain, superStructure, fourBarHandler) : null;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        configurePathPlanner();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Field", field);


        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        configurePathPlanner();
        configureTuning();
        if (RobotBase.isSimulation()) {
            // The real robot gets its pose from an auto or vision; in simulation start somewhere valid on the field.
            drivetrain.resetPose(new Pose2d(Constants.SimConstants.StartXMeters, Constants.SimConstants.StartYMeters,
                Rotation2d.fromDegrees(Constants.SimConstants.StartHeadingDegrees)));
        }

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        if (RobotBase.isSimulation()) DriverStation.silenceJoystickConnectionWarning(true);
    }


    private void configurePathPlanner() {

        NamedCommands.registerCommand("Intake",
         new InstantCommand(() -> superStructure.setDesiredState(SuperStructure.SuperStructureState.INTAKE)));

         NamedCommands.registerCommand("Poop",
         new InstantCommand(() -> superStructure.setDesiredState(SuperStructure.SuperStructureState.POOP)));

        // "Idle" stops the rollers and lowers the floor but leaves the intake where it is (it would stall pulling in against a full hopper)
        NamedCommands.registerCommand("Idle",
         new InstantCommand(() -> superStructure.setDesiredState(SuperStructure.SuperStructureState.IDLE)));

        NamedCommands.registerCommand("Jerk",
         new InstantCommand(() -> superStructure.setDesiredState(SuperStructure.SuperStructureState.JERK)));
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        //THIS IS WHERE THE DEFAULT DRIVE COMMAND IS, AKA THIS IS THE ACTUAL THING THATS LETTING THE ROBOT DRIVE
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(JoystickDriveUtil.scaleAxis(MathUtil.applyDeadband(-joystick.getLeftY(), 0.1)) * MaxSpeed * driveSpeedScale()) // Drive forward with negative Y (forward)
                    .withVelocityY(JoystickDriveUtil.scaleAxis(MathUtil.applyDeadband(-joystick.getLeftX(), 0.1)) * MaxSpeed * driveSpeedScale()) // Drive left with negative X (left)
                    .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /* yippe I commented out all of CTRES bumish code 
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight  .withVelocityX(-0.5).withVelocityY(0))
        );
        */

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // Start (the small right button) re-seeds the field-centric heading
        
        drivetrain.registerTelemetry(logger::telemeterize);

        //EVERYTHING ABOVE IS PREGENERATED, EVERTHING BELOW IS MY CODE WOOO
        //in pressing the A button on the controller, set superstructure state to STATE1
        // Hold left trigger: intake. Hold right trigger: outtake. Release both: idle (rollers off, intake and floor stay where they are).
        // Left trigger = intake. Right trigger = outtake (rollers only: the floor is not touched). Y toggles the hopper floor.
        joystick.leftTrigger().onTrue(new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.INTAKE)));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.POOP)));
        joystick.leftTrigger().onFalse(new InstantCommand(this::idleOrKeepHeldState));
        joystick.rightTrigger().onFalse(new InstantCommand(this::idleOrKeepHeldState));
        // Back: stow the intake. Only do this with the hopper empty: pulling the intake in against a full hopper can stall the pivot motor.
        joystick.back().onTrue(stowIntakeButton()); // Back (the small left button)
        // Y: toggle the hopper floor. Down -> all the way up; up -> down. (It waits for the intake to be down before lifting.)
        joystick.y().onTrue(new InstantCommand(superStructure::toggleFloor));
        // X: re-zero the intake pivot (only press it with the intake physically stowed against its stop, e.g. if the intake lost its position)
        joystick.x().onTrue(new InstantCommand(superStructure::rezeroIntakeToStowed).ignoringDisable(true));

        // Press BOTH bumpers at once: drive to the outpost, then keep feeding it on its own. Press both again to cancel (the intake stays where it is).
        joystick.leftBumper().and(joystick.rightBumper()).toggleOnTrue(outpostFeedCommand());

        // A: toggle the drive speed between 80% (normal) and 100% (turbo). While intaking it is always 75%. Shown on the dashboard under Drive/.
        joystick.a().onTrue(new InstantCommand(() -> turboDrive = !turboDrive).ignoringDisable(true));

        // If the floor or intake lose their zero (a motor rebooted, or the reading jumped), warn the driver with a long rumble.
        new edu.wpi.first.wpilibj2.command.button.Trigger(superStructure::isPositionLost).onTrue(
            Commands.startEnd(() -> joystick.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0),
                              () -> joystick.setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0)).withTimeout(1.5).ignoringDisable(true));

        // 4-bar (temporary test bindings; the floor normally follows the SuperStructure states above)
        // D-pad down while DISABLED: zero the floor (resting flat on its down stop). While enabled, D-pad down is the manual floor jog below.
        joystick.povDown().and(RobotModeTriggers.disabled()).onTrue(fourBarHandler.zeroHere());

        // Manual overrides for jams (enabled only; held = keep moving, released = hold). Soft limits, the intake/floor interlock and
        // safe mode all still apply, so these cannot drive anything into a stop or into each other.
        joystick.povLeft().whileTrue(Commands.run(() -> IntakePivotHandler.getInstance().manualJog(-1, 0.02)).withName("Intake jog: deploy"));
        joystick.povRight().whileTrue(Commands.run(() -> IntakePivotHandler.getInstance().manualJog(+1, 0.02)).withName("Intake jog: stow"));
        joystick.povUp().whileTrue(Commands.run(() -> fourBarHandler.manualJog(+1, 0.02)).withName("Floor jog: up"));
        joystick.povDown().and(RobotModeTriggers.disabled().negate()).whileTrue(Commands.run(() -> fourBarHandler.manualJog(-1, 0.02)).withName("Floor jog: down"));
        SmartDashboard.putData("FourBar Raise", new InstantCommand(() -> fourBarHandler.setDesiredState(FourBarHandler.FourBarState.RAISED)).ignoringDisable(true));
        SmartDashboard.putData("FourBar Flat", new InstantCommand(() -> fourBarHandler.setDesiredState(FourBarHandler.FourBarState.FLAT)).ignoringDisable(true));
        SmartDashboard.putData("FourBar Zero Here (disabled only, floor resting flat)", fourBarHandler.zeroHere());
        // Same as left trigger / left bumper, for testing without a controller
        SmartDashboard.putData("Intake Deploy", new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.INTAKE)).ignoringDisable(true));
        SmartDashboard.putData("Intake Stow (hopper must be empty)", new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.STOWED)).ignoringDisable(true));
    }

    // live-tunable outpost feeding (/Tuning/Outpost/...): read each time the button is pressed
    private final TunableNumber outpostX = Tunables.number("Outpost/PoseXMeters", Constants.OutpostConstants.OutpostXMeters, 0, 16.6, "OutpostConstants.OutpostXMeters");
    private final TunableNumber outpostY = Tunables.number("Outpost/PoseYMeters", Constants.OutpostConstants.OutpostYMeters, 0, 8.2, "OutpostConstants.OutpostYMeters");
    private final TunableNumber outpostHeading = Tunables.number("Outpost/PoseHeadingDegrees", Constants.OutpostConstants.OutpostHeadingDegrees, -180, 180, "OutpostConstants.OutpostHeadingDegrees");
    private final TunableNumber outpostApproachDistance = Tunables.number("Outpost/ApproachDistanceMeters", Constants.OutpostConstants.ApproachDistanceMeters, 0.3, 3, "OutpostConstants.ApproachDistanceMeters");
    private final TunableNumber outpostApproachVel = Tunables.number("Outpost/ApproachVelocityMetersPerSecond", Constants.OutpostConstants.ApproachVelocityMetersPerSecond, 0.2, 3, "OutpostConstants.ApproachVelocityMetersPerSecond");
    private final TunableNumber outpostApproachAccel = Tunables.number("Outpost/ApproachAccelerationMetersPerSecondSq", Constants.OutpostConstants.ApproachAccelerationMetersPerSecondSq, 0.3, 6, "OutpostConstants.ApproachAccelerationMetersPerSecondSq");
    private final TunableNumber outpostMaxVel = Tunables.number("Outpost/MaxVelocityMetersPerSecond", Constants.OutpostConstants.MaxVelocityMetersPerSecond, 0.2, 4.5, "OutpostConstants.MaxVelocityMetersPerSecond");
    private final TunableNumber outpostMaxAccel = Tunables.number("Outpost/MaxAccelerationMetersPerSecondSq", Constants.OutpostConstants.MaxAccelerationMetersPerSecondSq, 0.2, 10, "OutpostConstants.MaxAccelerationMetersPerSecondSq");
    private final TunableNumber outpostMaxAngVel = Tunables.number("Outpost/MaxAngularVelocityDegreesPerSecond", Constants.OutpostConstants.MaxAngularVelocityDegreesPerSecond, 30, 720, "OutpostConstants.MaxAngularVelocityDegreesPerSecond");
    private final TunableNumber outpostMaxAngAccel = Tunables.number("Outpost/MaxAngularAccelerationDegreesPerSecondSq", Constants.OutpostConstants.MaxAngularAccelerationDegreesPerSecondSq, 30, 1440, "OutpostConstants.MaxAngularAccelerationDegreesPerSecondSq");
    private final TunableNumber pulseOff = Tunables.number("Outpost/PulseOffSeconds", Constants.OutpostConstants.PulseOffSeconds, 0.2, 30, "OutpostConstants.PulseOffSeconds");
    private final TunableNumber pulseOn = Tunables.number("Outpost/PulseOnSeconds", Constants.OutpostConstants.PulseOnSeconds, 0.05, 10, "OutpostConstants.PulseOnSeconds");
    private final TunableNumber pulsesBeforeFloorRaise = Tunables.number("Outpost/PulsesBeforeFloorRaise", Constants.OutpostConstants.PulsesBeforeFloorRaise, 0, 50, "OutpostConstants.PulsesBeforeFloorRaise");
    // set the robot's field pose from Elastic (the pose starts at 0,0 until an auto or vision sets it): /Tuning/Pose/...
    private final TunableNumber setPoseX = Tunables.number("Pose/SetXMeters", Constants.SimConstants.StartXMeters, 0, 16.5, null);
    private final TunableNumber setPoseY = Tunables.number("Pose/SetYMeters", Constants.SimConstants.StartYMeters, 0, 8.0, null);
    private final TunableNumber setPoseHeading = Tunables.number("Pose/SetHeadingDegrees", Constants.SimConstants.StartHeadingDegrees, -180, 180, null);
    private final TunableNumber testDwell = Tunables.number("Tests/SquareWaveDwellSeconds", 2.0, 0.3, 10, null);

    /**
     * Pathfinds to the outpost (flipped automatically for red), then deploys the intake slightly and pulses the outtake
     * every few seconds to feed it, until cancelled (right bumper again). Stows when cancelled. The tuned values are read
     * when the button is pressed, and the waits re-read them every pulse.
     */
    private Command outpostFeedCommand() {
        // Pathfind (around the hub and over a bump, never through a trench) to a staging point out from the wall, then follow a short
        // straight path into the wall so the robot arrives square, facing the wall, at a gentle speed. Built each time the button is
        // pressed so tuned values are used; the whole thing is flipped automatically for the red alliance.
        Command drive = driveToOutpostWithRetries(Constants.OutpostConstants.MaxApproachAttempts);

        // At the wall: lift the intake slightly (to the feed angle), then pulse the outtake (on, then off), and raise the hopper floor once
        // it has pulsed PulsesBeforeFloorRaise times.
        final int[] pulses = {0};
        Command liftIntake = Commands.runOnce(() -> superStructure.setDesiredState(SuperStructureState.FEED))
            .andThen(Commands.waitUntil(() -> Math.abs(IntakePivotHandler.getInstance().getPositionDeg() - IntakePivotHandler.getInstance().getFeedDeg()) < 3.0)
                .withTimeout(Constants.OutpostConstants.IntakeLiftTimeoutSeconds));
        Command onePulse = Commands.runOnce(() -> superStructure.setDesiredState(SuperStructureState.FEED_PULSE))
            .andThen(Commands.defer(() -> Commands.waitSeconds(pulseOn.get()), Set.of()))
            .andThen(Commands.runOnce(() -> {
                superStructure.setDesiredState(SuperStructureState.FEED);
                pulses[0]++;
                TuningTelemetry.put("Outpost/PulsesDone", pulses[0]);
                if (pulses[0] >= pulsesBeforeFloorRaise.get()) superStructure.raiseFloor();
            }))
            .andThen(Commands.defer(() -> Commands.waitSeconds(pulseOff.get()), Set.of()));
        Command feed = Commands.runOnce(() -> { pulses[0] = 0; TuningTelemetry.put("Outpost/PulsesDone", 0); })
            .andThen(liftIntake)
            .andThen(Commands.repeatingSequence(onePulse));

        Command couldNotArrive = Commands.runOnce(() -> {
            DogLog.logFault("Outpost feed: did not reach the outpost, giving up");
            TuningTelemetry.put("Outpost/Status", "Did not reach the outpost (no route, or timed out)");
        });
        Command noPose = Commands.runOnce(() -> {
            DogLog.logFault("Outpost feed: robot pose is not on the field. Set it first (auto start, vision, or Tuning/Pose/Actions/SetRobotPoseNow)");
            TuningTelemetry.put("Outpost/Status", "Pose is not on the field: set the robot pose first");
        });

        Command drivesThenFeeds = Commands.runOnce(() -> TuningTelemetry.put("Outpost/Status", "Driving to the outpost"))
            .andThen(drive)
            .andThen(Commands.either(
                Commands.runOnce(() -> TuningTelemetry.put("Outpost/Status", "Feeding")).andThen(feed),
                couldNotArrive,
                () -> drivetrain.getState().Pose.getTranslation().getDistance(outpostTargetForAlliance().getTranslation())
                    < Constants.OutpostConstants.ArrivedToleranceMeters));

        return Commands.either(drivesThenFeeds, noPose, () -> poseLooksValid(drivetrain.getState().Pose))
            .finallyDo(() -> {
                superStructure.setDesiredState(SuperStructureState.IDLE); // the intake stays where it is; press A to stow it when the hopper is empty
                fourBarHandler.setDesiredState(FourBarHandler.FourBarState.FLAT); // the outpost feeding raised the floor: put it back
            })
            .withName("Outpost feed");
    }

    private boolean atOutpost() {
        return drivetrain.getState().Pose.getTranslation().getDistance(outpostTargetForAlliance().getTranslation())
            < Constants.OutpostConstants.ArrivedToleranceMeters;
    }

    /**
     * One attempt is: pathfind, follow the approach path, final align. PathPlanner's path follower ends on a timer, not on position, so if the robot
     * was bumped off course it can end far from the outpost; when that happens we re-plan from wherever it is now, up to {@code attemptsLeft} times.
     */
    private Command driveToOutpostWithRetries(int attemptsLeft) {
        Command attempt = Commands.defer(this::buildOutpostApproach, Set.of(drivetrain))
            .withTimeout(Constants.OutpostConstants.PathfindTimeoutSeconds)
            .andThen(finalAlignToOutpost());
        if (attemptsLeft <= 1) return attempt;
        return attempt.andThen(Commands.either(Commands.none(),
            Commands.runOnce(() -> TuningTelemetry.put("Outpost/Status", "Knocked off course: re-planning"))
                .andThen(Commands.defer(() -> driveToOutpostWithRetries(attemptsLeft - 1), Set.of(drivetrain))),
            this::atOutpost));
    }

    private final SwerveRequest.ApplyFieldSpeeds alignRequest = new SwerveRequest.ApplyFieldSpeeds();

    /**
     * Creeps onto the exact outpost pose (position and heading) after the approach path, which stops a few centimeters early.
     * Only runs if the robot is already close; field-relative (blue-origin) speeds, so it works on either alliance.
     */
    private Command finalAlignToOutpost() {
        final edu.wpi.first.math.controller.PIDController headingPid = new edu.wpi.first.math.controller.PIDController(Constants.OutpostConstants.AlignHeadingKp, 0, 0);
        headingPid.enableContinuousInput(-Math.PI, Math.PI);
        return Commands.either(
            drivetrain.run(() -> {
                Pose2d target = outpostTargetForAlliance();
                Pose2d now = drivetrain.getState().Pose;
                double max = Constants.OutpostConstants.AlignMaxSpeedMetersPerSecond;
                double vx = edu.wpi.first.math.MathUtil.clamp((target.getX() - now.getX()) * Constants.OutpostConstants.AlignKp, -max, max);
                double vy = edu.wpi.first.math.MathUtil.clamp((target.getY() - now.getY()) * Constants.OutpostConstants.AlignKp, -max, max);
                double omega = headingPid.calculate(now.getRotation().getRadians(), target.getRotation().getRadians());
                drivetrain.setControl(alignRequest.withSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds(vx, vy, omega)));
            }).until(this::alignedWithOutpost).withTimeout(Constants.OutpostConstants.AlignTimeoutSeconds),
            Commands.none(),
            () -> drivetrain.getState().Pose.getTranslation().getDistance(outpostTargetForAlliance().getTranslation()) < 0.8);
    }

    private boolean alignedWithOutpost() {
        Pose2d target = outpostTargetForAlliance();
        Pose2d now = drivetrain.getState().Pose;
        return now.getTranslation().getDistance(target.getTranslation()) < Constants.OutpostConstants.AlignToleranceMeters
            && Math.abs(now.getRotation().minus(target.getRotation()).getDegrees()) < Constants.OutpostConstants.AlignToleranceDegrees;
    }

    /** The approach: pathfind to the start of a straight path ending at the outpost pose, then follow it. Blue-alliance coordinates. */
    private Command buildOutpostApproach() {
        Rotation2d heading = Rotation2d.fromDegrees(outpostHeading.get());
        Pose2d end = new Pose2d(outpostX.get(), outpostY.get(), heading);
        // staging point: straight back from the wall along the robot's heading axis (the robot drives toward the wall in the heading direction)
        Pose2d staging = new Pose2d(
            end.getX() - heading.getCos() * outpostApproachDistance.get(),
            end.getY() - heading.getSin() * outpostApproachDistance.get(),
            heading);
        PathConstraints approachConstraints = new PathConstraints(
            outpostApproachVel.get(), outpostApproachAccel.get(),
            Math.toRadians(outpostMaxAngVel.get()), Math.toRadians(outpostMaxAngAccel.get()));
        PathPlannerPath approach = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(java.util.List.of(staging, end)),
            approachConstraints,
            new IdealStartingState(0.0, heading),   // arrives at the staging point already facing the wall
            new GoalEndState(0.0, heading));
        PathConstraints pathfindConstraints = new PathConstraints(
            outpostMaxVel.get(), outpostMaxAccel.get(),
            Math.toRadians(outpostMaxAngVel.get()), Math.toRadians(outpostMaxAngAccel.get()));
        return AutoBuilder.pathfindThenFollowPath(approach, pathfindConstraints);
    }

    /** Sets the drivetrain's pose to the tuned Pose/Set values (blue-alliance coordinates). */
    public void setRobotPoseFromTunables() {
        drivetrain.resetPose(new Pose2d(setPoseX.get(), setPoseY.get(), Rotation2d.fromDegrees(setPoseHeading.get())));
    }

    /** true if the pose is somewhere on the field. At (0, 0) (nothing has set it yet) the pathfinder cannot plan a route. */
    static boolean poseLooksValid(Pose2d pose) {
        return pose.getX() > 0.3 && pose.getX() < 16.2 && pose.getY() > 0.3 && pose.getY() < 7.7;
    }

    /** The outpost pose for the current alliance. */
    private Pose2d outpostTargetForAlliance() {
        Pose2d blue = new Pose2d(outpostX.get(), outpostY.get(), Rotation2d.fromDegrees(outpostHeading.get()));
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
            ? com.pathplanner.lib.util.FlippingUtil.flipFieldPose(blue) : blue;
    }

    /** Buttons in Elastic (SmartDashboard commands) for tuning tests. */
    private void configureTuning() {
        Tunables.action("Pose/Actions/SetRobotPoseNow", this::setRobotPoseFromTunables);
        // The whole outpost sequence from the dashboard, so it never depends on the controller's button numbering.
        SmartDashboard.putData("Outpost Feed", outpostFeedCommand());
        // The floor cannot raise while the intake is stowed (they would collide), so this test deploys the intake first and leaves it deployed.
        SmartDashboard.putData("Tuning/FourBar Square Wave",
            Commands.runOnce(() -> IntakePivotHandler.getInstance().setDesiredState(IntakePivotHandler.IntakePivotState.ENGADGED))
                .andThen(Commands.waitUntil(() -> IntakePivotHandler.getInstance().getPositionDeg() < Constants.InterlockConstants.IntakeClearBelowDeg - 5).withTimeout(4))
                .andThen(fourBarHandler.squareWave(testDwell::get))
                .withName("FourBar square wave (deploys the intake first)"));
        SmartDashboard.putData("Tuning/IntakePivot Square Wave", IntakePivotHandler.getInstance().squareWave(testDwell::get));
    }

    /** For simulation tests. */
    FuelSimulation getFuelSimulation() {
        return fuelSimulation;
    }

    /** Called from Robot.simulationPeriodic(). */
    public void simulationPeriodic() {
        if (fuelSimulation != null) fuelSimulation.update();
    }

    /** Logs the controller inputs that drive the intake bindings, to debug mapping problems. Call every loop. */
    private String lastPressedRaw = "";
    private long lastButtonMask = -1;

    // ---- drive speed: 80% normally, 75% while intaking, and the Back button toggles 100% / 80% ----
    private boolean turboDrive = false;
    private final TunableNumber normalSpeedScale = Tunables.number("Drive/NormalSpeedScale", Constants.DriveSpeedConstants.NormalScale, 0.1, 1.0, "DriveSpeedConstants.NormalScale");
    private final TunableNumber intakeSpeedScale = Tunables.number("Drive/IntakeSpeedScale", Constants.DriveSpeedConstants.IntakeScale, 0.1, 1.0, "DriveSpeedConstants.IntakeScale");
    private final TunableNumber turboSpeedScale = Tunables.number("Drive/TurboSpeedScale", Constants.DriveSpeedConstants.TurboScale, 0.1, 1.0, "DriveSpeedConstants.TurboScale");

    private boolean isIntaking() {
        return superStructure.getDesiredState() == SuperStructureState.INTAKE;
    }

    /** The fraction of full translation speed the driver gets right now. */
    private double driveSpeedScale() {
        if (isIntaking()) return intakeSpeedScale.get();
        return turboDrive ? turboSpeedScale.get() : normalSpeedScale.get();
    }

    private void publishDriveSpeed() {
        double scale = driveSpeedScale();
        SmartDashboard.putNumber("Drive/SpeedPercent", Math.round(scale * 100.0));
        SmartDashboard.putBoolean("Drive/TurboOn (A button)", turboDrive);
        SmartDashboard.putString("Drive/Mode", isIntaking() ? "Intaking (slowed)" : turboDrive ? "TURBO (toggled on)" : "Normal");
    }

    public void logInputs() {
        publishDriveSpeed();
        // Which raw buttons the driver station sees pressed, 1-based (what the sim GUI numbers them). In simulation this is how
        // you find out which number a physical button has, for the SIM_BUTTON_* table in SimAwareXboxController.
        long mask = DriverStation.getStickButtons(0);
        if (mask != lastButtonMask) { // only rebuild the text when a button actually changed
            lastButtonMask = mask;
            StringBuilder pressed = new StringBuilder();
            for (int i = 0; i < 32; i++) {
                if ((mask & (1L << i)) != 0) pressed.append(pressed.length() == 0 ? "" : ",").append(i + 1);
            }
            String now = pressed.toString();
            TuningTelemetry.put("Controller/PressedRawButtons", now);
            if (RobotBase.isSimulation() && !now.equals(lastPressedRaw)) {
                System.out.println("[Controller] raw buttons pressed: " + (now.isEmpty() ? "(none)" : now));
            }
            lastPressedRaw = now;
        }
        DogLog.log("Controller/LeftTrigger", joystick.getLeftTriggerAxis());
        DogLog.log("Controller/RightTrigger", joystick.getRightTriggerAxis());
        DogLog.log("Controller/LeftTriggerPressed", joystick.leftTrigger().getAsBoolean());
        DogLog.log("Controller/RightTriggerPressed", joystick.rightTrigger().getAsBoolean());
    }

    /** The stow button. Only use it with the hopper empty. */
    private Command stowIntakeButton() {
        return Commands.runOnce(() -> superStructure.setDesiredState(SuperStructureState.STOWED))
            .ignoringDisable(true)
            .withName("Stow intake");
    }

    /** Called when a trigger is released: if the other trigger is still held, go to its state, otherwise go idle (intake stays put). */
    private void idleOrKeepHeldState() {
        if (joystick.leftTrigger().getAsBoolean()) {
            superStructure.setDesiredState(SuperStructureState.INTAKE);
        } else if (joystick.rightTrigger().getAsBoolean()) {
            superStructure.setDesiredState(SuperStructureState.POOP);
        } else {
            superStructure.setDesiredState(SuperStructureState.IDLE);
        }
    }

    /** Call every loop (cheap). Redraws the preview only when the selected auto or alliance changes. */
    public void updateAutoPreview() {
        field.setRobotPose(drivetrain.getState().Pose);

            // Hide the preview once teleop starts, and keep it hidden
        if (DriverStation.isTeleopEnabled()) {
            if (previewedAuto != null) {
                previewedAuto = null;
                field.getObject("Auto Preview").setPoses(new ArrayList<Pose2d>());
                field.getObject("Active Path").setPoses(new ArrayList<Pose2d>());
                field.getObject("Target Pose").setPoses(new ArrayList<Pose2d>());
            }
            return;
        }

        Command selected = autoChooser.getSelected();
        String name = (selected == null) ? "" : selected.getName();
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        if (name.equals(previewedAuto) && red == previewedRed) return;
        previewedAuto = name;
        previewedRed = red;

        List<Pose2d> poses = new ArrayList<>();
        try {
            for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(name)) {
                if (red) path = path.flipPath();
                poses.addAll(path.getPathPoses());
            }
        } catch (Exception e) {
            // "None", a missing file, etc.: just show nothing
        }
        field.getObject("Auto Preview").setPoses(poses);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}