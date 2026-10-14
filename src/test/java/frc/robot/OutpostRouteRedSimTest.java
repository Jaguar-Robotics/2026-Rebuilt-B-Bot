package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/** The same outpost route on the RED alliance: the outpost is in the opposite corner and the robot faces the red wall (heading 0). */
class OutpostRouteRedSimTest {
    @ParameterizedTest(name = "from ({0}, {1}) facing {2} deg")
    @CsvSource({
        "13.5, 4.0, 180",  // red alliance zone, in front of the hub
        "8.3, 4.0, 90",    // middle of the neutral zone
        "8.5, 7.2, 0",     // neutral zone beside the red trench lane the straight way home would use
        "8.0, 0.9, 0",     // neutral zone, far side
        "10.3, 5.6, 270",  // just outside the red bump lane
        "14.5, 1.0, 180"   // red alliance zone, far corner
    })
    void routeOverTheBumpAndArrivesFacingTheRedWall(double x, double y, double headingDeg) {
        RobotSim sim = RobotSim.get(false);
        sim.rc.drivetrain.resetPose(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
        sim.run(0.5);
        sim.pressButton(RobotSim.kRightBumper);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kRightBumper);

        double worst = Double.MAX_VALUE;
        String worstWho = "";
        Pose2d worstPose = null;
        List<String> crossings = new ArrayList<>();
        boolean arrived = false, wasInBand = false;
        double t0 = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        for (int i = 0; i < 28 * 50; i++) {
            sim.loop();
            Pose2d p = sim.rc.drivetrain.getState().Pose;
            var cl = FieldGeometry.clearance(p);
            if (cl.meters() < worst) { worst = cl.meters(); worstWho = cl.obstacle(); worstPose = p; }
            boolean inBand = FieldGeometry.inStructureBand(p);
            if (inBand && !wasInBand) crossings.add(String.format("(%.2f, %.2f)", p.getX(), p.getY()));
            wasInBand = inBand;
            if (sim.rc.superStructure.getCurrentState() == SuperStructureState.FEED) { arrived = true; break; }
        }
        Pose2d end = sim.rc.drivetrain.getState().Pose;
        System.out.printf("[ROUTE-RED] from (%.1f, %.1f): %s in %.1f s, ended (%.2f, %.2f) facing %.0f deg; entered the band at %s; closest approach %.2f m (%s) at (%.2f, %.2f)%n",
            x, y, arrived ? "ARRIVED" : "DID NOT ARRIVE", edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - t0, end.getX(), end.getY(),
            end.getRotation().getDegrees(), crossings, worst, worstWho, worstPose.getX(), worstPose.getY());

        assertTrue(arrived, "never reached the red outpost");
        assertTrue(worst > 0.08, "the robot got within " + worst + " m of " + worstWho + " at " + worstPose);
        assertEquals(FieldGeometry.FIELD_X - 0.68, end.getX(), 0.06);
        assertEquals(FieldGeometry.FIELD_Y - 0.65, end.getY(), 0.06);
        assertEquals(0.0, end.getRotation().getDegrees(), 4.0, "facing the red wall (+x) with the intake side");
        double frontX = end.getX() + end.getRotation().getCos() * 0.6;
        assertTrue(FieldGeometry.FIELD_X - frontX > 0.0 && FieldGeometry.FIELD_X - frontX < 0.25, "intake front should be at the wall: " + (FieldGeometry.FIELD_X - frontX));
        sim.runUntil(() -> sim.intakeAngleDeg() < 20.0, 4);
        sim.run(0.8);
        assertEquals(10.0, sim.intakeAngleDeg(), 5.0);
        sim.pressButton(RobotSim.kRightBumper);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kRightBumper);
        sim.run(1.0);
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
