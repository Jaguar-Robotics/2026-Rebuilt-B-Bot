// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
import frc.robot.tuning.PerfMonitor;
import frc.robot.tuning.Tunables;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    /** For simulation tests that drive the whole robot. */
    public RobotContainer getRobotContainer() {
        return m_robotContainer;
    }

    @Override
    public void robotPeriodic() {
        long loopStartNanos = PerfMonitor.now();
        robotPeriodicInner();
        PerfMonitor.endLoop(loopStartNanos);
    }

    private void robotPeriodicInner() {
        long loopStart = PerfMonitor.now();
        long t = loopStart;
        m_timeAndJoystickReplay.update();
        t = PerfMonitor.lap("Loop.Replay", t);
        Tunables.periodic(); // apply anything edited live in Elastic
        t = PerfMonitor.lap("Loop.Tunables", t);
        CommandScheduler.getInstance().run();
        t = PerfMonitor.lap("Loop.Scheduler(all subsystems)", t);
        Visualizer.log();
        t = PerfMonitor.lap("Loop.Visualizer", t);
        m_robotContainer.logInputs();
        t = PerfMonitor.lap("Loop.LogInputs", t);

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        long t = PerfMonitor.now();
        m_robotContainer.simulationPeriodic();
        PerfMonitor.lap("Sim.FuelSim", t);
    }
}
