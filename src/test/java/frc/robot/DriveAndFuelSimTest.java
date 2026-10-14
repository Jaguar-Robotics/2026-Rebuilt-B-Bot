package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

/** Driving with the sticks, picking up fuel, and the hopper capacity limit. */
class DriveAndFuelSimTest {
    private static Pose2d pose(RobotSim sim) {
        return sim.rc.drivetrain.getState().Pose;
    }

    @Test
    void sticksDriveTheRobotAndSmallStickInputIsIgnored() {
        RobotSim sim = RobotSim.get();
        Pose2d p0 = pose(sim);
        sim.drive(0.05, 0.05, 0.05); // inside the 10% deadband
        sim.run(1.0);
        Pose2d p1 = pose(sim);
        System.out.printf("[SIM] tiny stick input moved the robot %.3f m, %.1f deg%n", p1.getTranslation().getDistance(p0.getTranslation()),
            p1.getRotation().minus(p0.getRotation()).getDegrees());
        assertTrue(p1.getTranslation().getDistance(p0.getTranslation()) < 0.05, "the deadband should hold the robot still");

        sim.drive(0.5, 0, 0);
        sim.run(1.5);
        Pose2d p2 = pose(sim);
        System.out.printf("[SIM] forward stick: moved %.2f m forward (dy %.2f)%n", p2.getX() - p1.getX(), p2.getY() - p1.getY());
        assertTrue(p2.getX() - p1.getX() > 0.8, "forward stick should drive forward");
        assertEquals(0.0, p2.getY() - p1.getY(), 0.25);

        sim.drive(0, 0.5, 0);
        sim.run(1.5);
        Pose2d p3 = pose(sim);
        System.out.printf("[SIM] left stick: moved %.2f m left%n", p3.getY() - p2.getY());
        assertTrue(p3.getY() - p2.getY() > 0.8, "left stick should strafe left");

        sim.drive(0, 0, 0.5);
        sim.run(1.0);
        Pose2d p4 = pose(sim);
        System.out.printf("[SIM] right stick: turned %.0f deg%n", p4.getRotation().minus(p3.getRotation()).getDegrees());
        assertTrue(p4.getRotation().minus(p3.getRotation()).getDegrees() > 20, "right stick should turn the robot");

        sim.drive(0, 0, 0);
        sim.run(1.0);
        assertTrue(Math.abs(sim.rc.drivetrain.getState().Speeds.vxMetersPerSecond) < 0.05, "releasing the sticks should stop it");
    }

    @Test
    void disablingStopsTheRobotAndNothingMovesWhileDisabled() {
        RobotSim sim = RobotSim.get();
        sim.drive(0.6, 0, 0);
        sim.run(1.0);
        sim.disable();
        sim.run(1.5);
        Pose2d stopped = pose(sim);
        sim.run(1.0);
        System.out.printf("[SIM] disabled: moved %.3f m in the last second%n", pose(sim).getTranslation().getDistance(stopped.getTranslation()));
        assertTrue(pose(sim).getTranslation().getDistance(stopped.getTranslation()) < 0.05);
    }

    @Test
    void deployedIntakeCollectsFuelDrivenOverUntilTheHopperIsFull() {
        RobotSim sim = RobotSim.get();
        var fuel = sim.rc.getFuelSimulation();
        fuel.setHopperCountForTesting(157); // room for exactly 3 more
        sim.rc.drivetrain.resetPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));
        sim.run(0.3);
        for (int i = 0; i < 6; i++) fuel.spawnFuelForTesting(4.1 + 0.25 * i, 4.0 + (i % 2 == 0 ? 0.1 : -0.1));
        assertEquals(6, fuel.getFuelOnFieldForTesting());

        sim.holdLeftTrigger(true);
        sim.runUntil(() -> sim.intakeAngleDeg() < 5.0, 5);
        sim.drive(0.35, 0, 0);
        sim.run(5.0);
        sim.drive(0, 0, 0);
        sim.run(0.5);
        System.out.printf("[SIM] drove over 6 fuel with room for 3: hopper %d/160, %d left on the field, robot at x=%.2f%n",
            fuel.getHopperCount(), fuel.getFuelOnFieldForTesting(), pose(sim).getX());
        assertEquals(160, fuel.getHopperCount(), "it should fill to exactly the capacity");
        assertEquals(3, fuel.getFuelOnFieldForTesting(), "the fuel it had no room for should be left on the field");
    }

    @Test
    void intakeCollectsNothingWhileStowed() {
        RobotSim sim = RobotSim.get();
        var fuel = sim.rc.getFuelSimulation();
        sim.rc.drivetrain.resetPose(new Pose2d(3.0, 4.0, Rotation2d.kZero));
        sim.run(0.3);
        for (int i = 0; i < 4; i++) fuel.spawnFuelForTesting(4.1 + 0.25 * i, 4.0);
        sim.drive(0.35, 0, 0); // intake stowed (no trigger held)
        sim.run(4.0);
        System.out.printf("[SIM] drove over fuel with the intake stowed: hopper %d%n", fuel.getHopperCount());
        assertEquals(0, fuel.getHopperCount());
    }

    @Test
    void collectingFuelAddsWeightThatTheFloorWeighingSees() {
        RobotSim sim = RobotSim.get();
        var fuel = sim.rc.getFuelSimulation();
        fuel.setHopperCountForTesting(80);
        sim.run(3.0);
        double balls = RobotSim.nt("/Tuning/Hopper/Live/EstimatedBalls");
        System.out.printf("[SIM] 80 balls loaded: floor weighing estimates %.0f%n", balls);
        assertEquals(80, balls, 12, "the estimate should track the load");
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
