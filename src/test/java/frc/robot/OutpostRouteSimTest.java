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

/** The outpost route from many starting points on the BLUE alliance: over the bumps, never under a trench, arriving square to the wall. */
class OutpostRouteSimTest {
    @ParameterizedTest(name = "from ({0}, {1}) facing {2} deg")
    @CsvSource({
        "3.0, 4.0, 0",     // alliance zone, in front of the hub
        "8.3, 4.0, 90",    // middle of the neutral zone
        "8.0, 0.9, 0",     // neutral zone beside the trench lane: the straight way home is UNDER the trench
        "8.0, 7.2, 180",   // neutral zone, far side
        "6.2, 2.5, 270",   // just outside the bump lane
        "2.0, 7.0, 0"      // alliance zone, far corner
    })
    void driesOverTheBumpNeverUnderTheTrenchAndArrivesFacingTheWall(double x, double y, double headingDeg) {
        RobotSim sim = RobotSim.get();
        sim.rc.drivetrain.resetPose(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
        sim.run(0.5);
        sim.pressBothBumpers();
        sim.run(0.1);
        sim.releaseBothBumpers();

        double worst = Double.MAX_VALUE;
        String worstWho = "";
        Pose2d worstPose = null;
        List<String> crossings = new ArrayList<>();
        boolean arrived = false;
        double t0 = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        boolean wasInBand = false;
        for (int i = 0; i < 28 * 50; i++) {
            sim.loop();
            Pose2d p = sim.rc.drivetrain.getState().Pose;
            var cl = FieldGeometry.clearance(p);
            if (cl.meters() < worst) {
                worst = cl.meters();
                worstWho = cl.obstacle();
                worstPose = p;
            }
            boolean inBand = FieldGeometry.inStructureBand(p);
            if (inBand && !wasInBand) crossings.add(String.format("(%.2f, %.2f)", p.getX(), p.getY()));
            wasInBand = inBand;
            if (sim.rc.superStructure.getCurrentState() == SuperStructureState.FEED) {
                arrived = true;
                break;
            }
        }
        Pose2d end = sim.rc.drivetrain.getState().Pose;
        double took = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - t0;
        System.out.printf("[ROUTE] from (%.1f, %.1f): %s in %.1f s, ended (%.2f, %.2f) facing %.0f deg; entered the trench/hub band at %s; "
            + "closest the robot's footprint came to an obstacle: %.2f m (%s) at (%.2f, %.2f)%n",
            x, y, arrived ? "ARRIVED" : "DID NOT ARRIVE", took, end.getX(), end.getY(), end.getRotation().getDegrees(), crossings,
            worst, worstWho, worstPose.getX(), worstPose.getY());

        assertTrue(arrived, "never reached the outpost");
        assertTrue(worst > 0.02, "the robot's footprint overlapped " + worstWho + " at " + worstPose);
        assertEquals(0.68, end.getX(), 0.06);
        assertEquals(0.65, end.getY(), 0.06);
        assertEquals(180.0, Math.abs(end.getRotation().getDegrees()), 4.0, "should be facing the wall (the intake side)");
        // the front of the intake (about 0.6 m ahead of the center) should be right at the wall
        double frontX = end.getX() + end.getRotation().getCos() * 0.6;
        assertTrue(frontX > 0.0 && frontX < 0.25, "intake front should be against the wall, was " + frontX + " m from it");
        sim.runUntil(() -> sim.intakeAngleDeg() < 20.0, 4);
        sim.run(0.8);
        assertEquals(10.0, sim.intakeAngleDeg(), 5.0, "intake almost fully deployed, slightly raised");
        assertTrue(worst > 0.08, "the closest approach to " + worstWho + " was only " + worst + " m");
        sim.pressBothBumpers();
        sim.run(0.1);
        sim.releaseBothBumpers();
        sim.run(1.0);
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
