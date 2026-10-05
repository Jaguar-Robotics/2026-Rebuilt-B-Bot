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

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Handlers.FourBarHandler;
import frc.robot.Handlers.SuperStructure;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final SuperStructure superStructure = SuperStructure.getInstance(); //creates a new SuperStructure; try to create all handlers in superstructure and each of their subsystems in their handlers, but like sometimes that wont work so its whateva

    public final FourBarHandler fourBarHandler = FourBarHandler.getInstance();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        configurePathPlanner();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
        configurePathPlanner();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        if (RobotBase.isSimulation()) DriverStation.silenceJoystickConnectionWarning(true);
    }


    private void configurePathPlanner() {

        NamedCommands.registerCommand("Intake",
         new InstantCommand(() -> superStructure.setDesiredState(SuperStructure.SuperStructureState.INTAKE)));

         NamedCommands.registerCommand("Poop",
         new InstantCommand(() -> superStructure.setDesiredState(SuperStructure.SuperStructureState.POOP)));

        NamedCommands.registerCommand("Idle",
         new InstantCommand(() -> superStructure.setDesiredState(SuperStructure.SuperStructureState.STOWED)));

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
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        drivetrain.registerTelemetry(logger::telemeterize);

        //EVERYTHING ABOVE IS PREGENERATED, EVERTHING BELOW IS MY CODE WOOO
        //in pressing the A button on the controller, set superstructure state to STATE1
        // Hold left trigger: intake. Hold right trigger: outtake. Release both: stow.
        // (Left bumper no longer stows; it only re-seeds the field-centric heading above.)
        joystick.leftTrigger().onTrue(new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.INTAKE)));
        joystick.rightTrigger().onTrue(new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.POOP)));
        joystick.leftTrigger().onFalse(new InstantCommand(this::stowOrKeepHeldState));
        joystick.rightTrigger().onFalse(new InstantCommand(this::stowOrKeepHeldState));

        // 4-bar (temporary test bindings; INTAKE/STOWED are driven by the SuperStructure states above)
        joystick.back().onTrue(fourBarHandler.rehome());
        SmartDashboard.putData("FourBar Intake", new InstantCommand(() -> fourBarHandler.setDesiredState(FourBarHandler.FourBarState.INTAKE)).ignoringDisable(true));
        SmartDashboard.putData("FourBar Stow", new InstantCommand(() -> fourBarHandler.setDesiredState(FourBarHandler.FourBarState.STOWED)).ignoringDisable(true));
        SmartDashboard.putData("FourBar Rehome", fourBarHandler.rehome());
        // Same as left trigger / left bumper, for testing without a controller
        SmartDashboard.putData("Intake Deploy", new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.INTAKE)).ignoringDisable(true));
        SmartDashboard.putData("Intake Stow", new InstantCommand(() -> superStructure.setDesiredState(SuperStructureState.STOWED)).ignoringDisable(true));
    }

    /** Logs the controller inputs that drive the intake bindings, to debug mapping problems. Call every loop. */
    public void logInputs() {
        DogLog.log("Controller/LeftTrigger", joystick.getLeftTriggerAxis());
        DogLog.log("Controller/RightTrigger", joystick.getRightTriggerAxis());
        DogLog.log("Controller/LeftTriggerPressed", joystick.leftTrigger().getAsBoolean());
        DogLog.log("Controller/RightTriggerPressed", joystick.rightTrigger().getAsBoolean());
    }

    /** Called when a trigger is released: if the other trigger is still held, go to its state, otherwise stow. */
    private void stowOrKeepHeldState() {
        if (joystick.leftTrigger().getAsBoolean()) {
            superStructure.setDesiredState(SuperStructureState.INTAKE);
        } else if (joystick.rightTrigger().getAsBoolean()) {
            superStructure.setDesiredState(SuperStructureState.POOP);
        } else {
            superStructure.setDesiredState(SuperStructureState.STOWED);
        }
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
