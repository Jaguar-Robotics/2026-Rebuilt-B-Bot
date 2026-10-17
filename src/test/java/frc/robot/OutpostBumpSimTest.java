package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/**
 * The robot gets shoved off course while driving to the outpost. The shove is simulated by moving the robot (and so its pose estimate, as
 * if the Limelights had already corrected it). The sequence must still end up at the outpost and feed.
 */
class OutpostBumpSimTest {
    @ParameterizedTest(name = "shoved {0} m away in x, {1} m in y, turned {2} deg, when {3} m from the outpost")
    @CsvSource({
        "1.5, 0.3, 50, 2.5",   // mid approach
        "1.5, 0.3, 50, 1.0",   // on the final straight path (the path timer will run out with the robot off target)
        "2.0, 0.0, -90, 0.7",  // right at the wall
        "0.0, 1.5, 120, 1.5",  // pushed sideways along the wall
    })
    void getsBumpedOffCourseAndStillArrives(double dx, double dy, double dHeading, double triggerDistance) {
        RobotSim sim = RobotSim.get();
        sim.enableTeleop();
        sim.run(0.5);
        sim.pressBothBumpers();
        sim.run(0.1);
        sim.releaseBothBumpers();

        Pose2d target = new Pose2d(0.68, 0.65, Rotation2d.k180deg);
        double t = sim.runUntil(() -> sim.rc.drivetrain.getState().Pose.getTranslation().getDistance(target.getTranslation()) < triggerDistance, 20);
        assertTrue(t > 0, "never got within " + triggerDistance + " m");
        Pose2d before = sim.rc.drivetrain.getState().Pose;
        Pose2d shoved = new Pose2d(before.getX() + dx, before.getY() + dy, before.getRotation().plus(Rotation2d.fromDegrees(dHeading)));
        sim.rc.drivetrain.resetPose(shoved);
        System.out.printf("[BUMP] at %s, shoved to %s%n", before, shoved);

        double arrived = sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.FEED, 40);
        Pose2d at = sim.rc.drivetrain.getState().Pose;
        System.out.printf("[BUMP] %s after %.1f s at (%.2f, %.2f) heading %.0f deg, status: %s%n", arrived > 0 ? "ARRIVED" : "DID NOT ARRIVE", arrived,
            at.getX(), at.getY(), at.getRotation().getDegrees(), RobotSim.ntString("/Tuning/Outpost/Status"));
        assertTrue(arrived > 0, "never recovered: " + RobotSim.ntString("/Tuning/Outpost/Status"));
        assertEquals(0.68, at.getX(), 0.15);
        assertEquals(0.65, at.getY(), 0.15);
        assertEquals(180.0, Math.abs(at.getRotation().getDegrees()), 6.0);
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
