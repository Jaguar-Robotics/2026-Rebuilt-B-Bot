// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import frc.robot.tuning.PerfMonitor;
import frc.robot.tuning.Tunables;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


        ShiftInfo official = HubShiftUtil.getOfficialShiftInfo();

        // Official shift info 
        DogLog.log("Shift/Official/CurrentShift",   official.currentShift().toString());
        DogLog.log("Shift/Official/RemainingTime", Math.round(official.remainingTime()*10)/10.0);
        DogLog.log("Shift/Official/ElapsedTime",   Math.round(official.elapsedTime()*10)/10.0);
        DogLog.log("Shift/Official/MatchTime",        HubShiftUtil.getMatchTime());
        DogLog.log("Shift/Official/Active",        official.active());
    
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
