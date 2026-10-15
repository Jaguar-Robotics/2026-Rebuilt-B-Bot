package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Test;

/** Reproduction on the RED alliance (the sim GUI's default), starting where the sim starts the robot. */
class OutpostRedSimTest {
    @Test
    void reproduceOnRed() {
        RobotSim sim = RobotSim.get(false);
        sim.enableTeleop();
        sim.run(1.0);
        System.out.printf("[SIM] alliance=%s start pose %s%n", DriverStation.getAlliance(), sim.rc.drivetrain.getState().Pose);
        sim.pressButton(RobotSim.kRightBumper);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kRightBumper);
        for (int i = 0; i < 25; i++) {
            sim.run(1.0);
            Pose2d p = sim.rc.drivetrain.getState().Pose;
            System.out.printf("[SIM] t=%2ds pose=(%.2f, %.2f) speed=%.2f state=%s status='%s' cmd=%s%n", i + 1, p.getX(), p.getY(),
                Math.hypot(sim.rc.drivetrain.getState().Speeds.vxMetersPerSecond, sim.rc.drivetrain.getState().Speeds.vyMetersPerSecond),
                sim.rc.superStructure.getCurrentState(), RobotSim.ntString("/Tuning/Outpost/Status"),
                sim.rc.drivetrain.getCurrentCommand() == null ? "none" : sim.rc.drivetrain.getCurrentCommand().getName());
        }
    }
}
