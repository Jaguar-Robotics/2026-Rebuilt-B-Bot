package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import frc.robot.Handlers.IntakeHandler.IntakeState;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Drive to the outpost and feed it, with the whole robot simulated. Blue alliance. */
class OutpostSimTest {
    private static String ntString(String key) {
        return NetworkTableInstance.getDefault().getEntry("/Tuning/" + key).getString("");
    }

    @Test
    void oneButtonPressDrivesToTheOutpostThenFeedsOnItsOwnAndASecondPressStows() {
        RobotSim sim = RobotSim.get();
        sim.enableTeleop();
        sim.run(1.0);
        Pose2d start = sim.rc.drivetrain.getState().Pose;
        System.out.printf("[SIM] robot starts at %s%n", start);
        assertTrue(RobotContainer.poseLooksValid(start), "the sim should start the robot somewhere valid on the field");

        sim.pressBothBumpers(); // one press...
        sim.run(0.1);
        sim.releaseBothBumpers(); // ...and let go: the driver walks away

        double arrived = sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.FEED, 20);
        Pose2d at = sim.rc.drivetrain.getState().Pose;
        System.out.printf("[SIM] reached FEED after %.1f s at (%.2f, %.2f) heading %.0f deg%n", arrived, at.getX(), at.getY(), at.getRotation().getDegrees());
        assertTrue(arrived > 0, "never started feeding");
        assertEquals(0.7, at.getX(), 0.4);
        assertEquals(0.65, at.getY(), 0.4);
        assertEquals("Feeding", ntString("Outpost/Status"));

        // Now it should lift the intake slightly, pulse 2 s on / 2 s off without any input, and raise the floor after the 4th pulse.
        List<SuperStructureState> seen = new ArrayList<>();
        boolean sawFloorRaised = false, sawRollersReverse = false;
        int pulsesWhenFloorRaised = -1;
        double intakeAtFirstPulse = -1;
        double firstPulseSeconds = -1, pulseStart = -1;
        for (int i = 0; i < 30 * 50; i++) { // 30 s
            sim.loop();
            var st = sim.rc.superStructure.getCurrentState();
            if (seen.isEmpty() || seen.get(seen.size() - 1) != st) {
                seen.add(st);
                if (st == SuperStructureState.FEED_PULSE) {
                    pulseStart = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                    if (intakeAtFirstPulse < 0) intakeAtFirstPulse = sim.intakeAngleDeg();
                } else if (pulseStart >= 0 && firstPulseSeconds < 0) {
                    firstPulseSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - pulseStart;
                }
            }
            if (st == SuperStructureState.FEED_PULSE && Handlers_intake().getCurrentState() == IntakeState.REVSLOW) sawRollersReverse = true;
            if (!sawFloorRaised && sim.rc.fourBarHandler.getDesiredState() == FourBarState.RAISED) {
                sawFloorRaised = true;
                pulsesWhenFloorRaised = (int) seen.stream().filter(x -> x == SuperStructureState.FEED_PULSE).count();
            }
        }
        System.out.printf("[SIM] states over the next 30 s: %s; first pulse lasted %.1f s, intake at %.1f deg at the first pulse, floor raised after %d pulses%n",
            seen, firstPulseSeconds, intakeAtFirstPulse, pulsesWhenFloorRaised);
        long pulses = seen.stream().filter(x -> x == SuperStructureState.FEED_PULSE).count();
        assertTrue(pulses >= 5, "expected several pulses in 30 s, saw " + pulses);
        assertTrue(sawRollersReverse, "the rollers never reversed during a pulse");
        assertEquals(2.0, firstPulseSeconds, 0.2, "a pulse should last 2 s");
        assertTrue(intakeAtFirstPulse > 5 && intakeAtFirstPulse < 14, "the intake should be slightly raised (about 10 deg) when pulsing starts, was " + intakeAtFirstPulse);
        assertTrue(sawFloorRaised, "the floor never raised");
        assertEquals(4, pulsesWhenFloorRaised, "the floor should raise after exactly the 4th pulse");
        assertEquals(0.7, sim.rc.drivetrain.getState().Pose.getX(), 0.4, "it should stay parked at the outpost");

        sim.pressBothBumpers(); // second press cancels
        sim.run(0.1);
        sim.releaseBothBumpers();
        double stowed = sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.IDLE
            && sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 6);
        System.out.printf("[SIM] second press stowed everything after %.1f s%n", stowed);
        assertTrue(stowed > 0, "second press should cancel, stow the intake and lower the floor");
    }

    private static frc.robot.Handlers.IntakeHandler Handlers_intake() {
        return frc.robot.Handlers.IntakeHandler.getInstance();
    }

    @Test
    void releasingWithNoPoseGivesAClearMessageInsteadOfDoingNothing() {
        RobotSim sim = RobotSim.get();
        sim.enableTeleop();
        sim.rc.drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.kZero)); // the real robot's pose before anything sets it
        sim.run(0.5);
        sim.pressBothBumpers();
        sim.run(0.3);
        sim.releaseBothBumpers();
        sim.run(0.5);
        System.out.printf("[SIM] status with no pose: %s%n", ntString("Outpost/Status"));
        assertTrue(ntString("Outpost/Status").contains("not on the field"));
        assertFalse("Outpost feed".equals(sim.rc.drivetrain.getCurrentCommand() == null ? "" : sim.rc.drivetrain.getCurrentCommand().getName()),
            "the outpost command should have ended");
        assertEquals(SuperStructureState.IDLE, sim.rc.superStructure.getCurrentState());
    }

    @Test
    void setRobotPoseButtonFromElasticPlacesTheRobot() {
        RobotSim sim = RobotSim.get();
        sim.enableTeleop();
        sim.rc.drivetrain.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
        var nt = NetworkTableInstance.getDefault();
        nt.getEntry("/Tuning/Pose/SetXMeters").setDouble(5.0);
        nt.getEntry("/Tuning/Pose/SetYMeters").setDouble(2.0);
        nt.getEntry("/Tuning/Pose/SetHeadingDegrees").setDouble(90.0);
        sim.run(0.3);
        nt.getEntry("/Tuning/Pose/Actions/SetRobotPoseNow").setBoolean(true);
        sim.run(0.5);
        Pose2d p = sim.rc.drivetrain.getState().Pose;
        System.out.printf("[SIM] pose after SetRobotPoseNow: %s%n", p);
        assertEquals(5.0, p.getX(), 0.1);
        assertEquals(2.0, p.getY(), 0.1);
        assertEquals(90.0, p.getRotation().getDegrees(), 3.0);
    }

    @Test
    void theElasticOutpostFeedButtonRunsTheWholeSequenceWithoutTheController() {
        RobotSim sim = RobotSim.get();
        sim.run(0.5);
        var button = NetworkTableInstance.getDefault().getEntry("/SmartDashboard/Outpost Feed/running");
        button.setBoolean(true);
        double arrived = sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.FEED, 20);
        Pose2d at = sim.rc.drivetrain.getState().Pose;
        System.out.printf("[SIM] dashboard button: reached FEED after %.1f s at (%.2f, %.2f)%n", arrived, at.getX(), at.getY());
        assertTrue(arrived > 0, "the dashboard button never started the feed");
        assertEquals(0.7, at.getX(), 0.4);
        assertEquals(0.65, at.getY(), 0.4);
        button.setBoolean(false); // pressing it again in Elastic cancels
        double stowed = sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.IDLE, 6);
        assertTrue(stowed > 0, "turning the button off should cancel and go idle");
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
