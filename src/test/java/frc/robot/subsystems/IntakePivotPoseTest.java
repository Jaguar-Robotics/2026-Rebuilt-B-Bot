package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.Test;

class IntakePivotPoseTest {
    private static final double px = IntakeConstants.PivotXMeters;
    private static final double pz = IntakeConstants.PivotZMeters;

    @Test
    void noMovementAtTheCadAngle() {
        var pose = IntakePivotSubsystem.getIntakePose(IntakeConstants.CadPoseAngle, px, pz, IntakeConstants.CadPoseAngle, -1);
        assertEquals(0.0, pose.getTranslation().getNorm(), 1e-9);
        assertEquals(0.0, pose.getRotation().getAngle(), 1e-9);
    }

    @Test
    void rotatesAboutThePivotAndKeepsItFixed() {
        var stowedAngle = IntakeConstants.CadPoseAngle.plus(Degrees.of(120));
        var pose = IntakePivotSubsystem.getIntakePose(stowedAngle, px, pz, IntakeConstants.CadPoseAngle, -1);
        // Stowing (+120 deg) swings the intake up and back: a negative rotation about Y.
        var expected = new Rotation3d(0, -Math.toRadians(120), 0);
        // Compare as rotations: Euler pitch past 90 degrees is reported with roll/yaw flipped.
        assertEquals(0.0, expected.minus(pose.getRotation()).getAngle(), 1e-9);
        var pivot = new Translation3d(px, IntakeConstants.PivotYMeters, pz);
        var moved = pivot.rotateBy(pose.getRotation()).plus(pose.getTranslation());
        assertEquals(0.0, moved.getDistance(pivot), 1e-9);
    }

    @Test
    void hopperSlideFollowsTheIntake() {
        double travel = IntakeConstants.HopperSlideTravelMeters;
        var out = IntakePivotSubsystem.getHopperSlidePose(IntakeConstants.PivotLowSetpoint, travel);
        var in = IntakePivotSubsystem.getHopperSlidePose(IntakeConstants.PivotHighSetpoint, travel);
        var mid = IntakePivotSubsystem.getHopperSlidePose(
            IntakeConstants.PivotLowSetpoint.plus(IntakeConstants.PivotHighSetpoint).div(2), travel);
        assertEquals(0.0, out.getX(), 1e-9);         // intake deployed: hopper out (CAD pose)
        assertEquals(-travel, in.getX(), 1e-9);      // intake stowed: hopper retracted
        assertEquals(-travel / 2, mid.getX(), 1e-9); // follows in between
        assertEquals(0.0, in.getY(), 1e-9);
        assertEquals(0.0, in.getZ(), 1e-9);
    }
}
