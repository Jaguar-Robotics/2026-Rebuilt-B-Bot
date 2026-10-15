package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.Test;

/** The 3D poses AdvantageScope draws: floor, intake and hopper slide, read back from NetworkTables while the robot runs. */
class VisualizerSimTest {
    private static Pose3d[] poses(StructArraySubscriber<Pose3d> sub) {
        return sub.get();
    }

    @Test
    void floorIntakeAndHopperSlideMoveCorrectlyAsTheRobotOperates() {
        RobotSim sim = RobotSim.get();
        StructArraySubscriber<Pose3d> sub = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/Robot/Visualizer/ComponentPoses", Pose3d.struct).subscribe(new Pose3d[0]);
        sim.run(1.0);
        Pose3d[] boot = poses(sub);
        assertEquals(3, boot.length, "floor, intake and hopper slide");
        System.out.printf("[SIM] boot poses: floor pitch %.1f deg, intake pitch %.1f deg, hopper slide x %.3f m%n",
            Math.toDegrees(boot[0].getRotation().getY()), Math.toDegrees(boot[1].getRotation().getY()), boot[2].getX());
        assertEquals(0.0, Math.toDegrees(boot[0].getRotation().getY()), 3.0, "floor flat at boot");
        assertEquals(-IntakeConstants.HopperSlideTravelMeters, boot[2].getX(), 0.01, "hopper slid in with the intake stowed");

        sim.holdLeftTrigger(true);
        sim.runUntil(() -> sim.intakeAngleDeg() < 3, 5);
        sim.run(0.5);
        Pose3d[] deployed = poses(sub);
        System.out.printf("[SIM] intake deployed: intake pitch %.1f deg, hopper slide x %.3f m, floor pitch %.1f deg%n",
            Math.toDegrees(deployed[1].getRotation().getY()), deployed[2].getX(), Math.toDegrees(deployed[0].getRotation().getY()));
        assertEquals(0.0, deployed[2].getX(), 0.02, "hopper slid out with the intake");
        assertEquals(0.0, deployed[1].getRotation().getAngle(), Math.toRadians(4), "intake drawn at its CAD (deployed) pose");
        assertEquals(0.0, Math.toDegrees(deployed[0].getRotation().getY()), 3.0, "floor stays flat while intaking");

        sim.holdLeftTrigger(false);
        sim.holdRightTrigger(true);
        sim.runUntil(() -> sim.floorRot() > Constants.FourBarConstants.RaisedRot - 0.01, 6);
        sim.run(0.5);
        Pose3d[] raised = poses(sub);
        double floorPitch = Math.toDegrees(raised[0].getRotation().getY());
        System.out.printf("[SIM] floor raised: floor pitch %.1f deg (positive = back up, front down), translation z %.3f m%n", floorPitch, raised[0].getZ());
        assertTrue(floorPitch > 30 && floorPitch < 55, "raised floor should tilt about 40 degrees back-up, was " + floorPitch);
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
