package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.FourBarConstants;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import org.junit.jupiter.api.Test;

/** Re-home from the Back button, and what happens when the robot is disabled mid-sequence. */
class RehomeAndDisableSimTest {
    @Test
    void backButtonRehomesTheFloorAgainstTheDownStopAndEndsFlat() {
        RobotSim sim = RobotSim.get();
        sim.holdRightTrigger(true); // raise the floor first so the re-home has to drive down to the stop
        sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.RAISED, 6);
        sim.holdRightTrigger(false);
        sim.run(0.2);
        sim.pressButton(RobotSim.kBack);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kBack);
        sim.run(0.3);
        System.out.printf("[SIM] after Back: floor state %s%n", sim.rc.fourBarHandler.getCurrentState());
        assertEquals(FourBarState.HOMING, sim.rc.fourBarHandler.getCurrentState(), "should be homing");
        double done = sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 10);
        sim.run(1.0);
        System.out.printf("[SIM] re-home finished in %.1f s, floor at %.4f rot (flat setpoint %.4f)%n", done, sim.floorRot(), FourBarConstants.FlatRot);
        assertTrue(done > 0, "re-home never finished");
        assertEquals(FourBarConstants.FlatRot, sim.floorRot(), 0.004);
    }

    @Test
    void disablingMidOutpostFeedStowsAndReEnablingDoesNothingOnItsOwn() {
        RobotSim sim = RobotSim.get();
        sim.pressButton(RobotSim.kRightBumper);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kRightBumper);
        sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.FEED, 20);
        sim.run(1.0);
        sim.disable();
        sim.run(0.5);
        System.out.printf("[SIM] disabled during the feed: superstructure %s%n", sim.rc.superStructure.getCurrentState());
        assertEquals(SuperStructureState.STOWED, sim.rc.superStructure.getCurrentState());
        sim.enableTeleop();
        sim.run(3.0);
        assertEquals(SuperStructureState.STOWED, sim.rc.superStructure.getCurrentState(), "re-enabling must not restart the feed");
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
