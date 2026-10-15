package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.FourBarConstants;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import org.junit.jupiter.api.Test;

/** The D-pad down zero (disabled only, never drives anything), and what happens when the robot is disabled mid-sequence. */
class RehomeAndDisableSimTest {
    @Test
    void dpadDownZeroIsRefusedWhileEnabledAndNeverDrivesTheFloor() {
        RobotSim sim = RobotSim.get();
        sim.holdRightTrigger(true);
        sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.RAISED, 6);
        double raisedAt = sim.floorRot();
        sim.pressDpadDown();
        sim.run(0.1);
        sim.releaseDpad();
        sim.run(1.0);
        System.out.printf("[SIM] D-pad down pressed while enabled: floor %s at %.4f rot (was %.4f), status: %s%n",
            sim.rc.fourBarHandler.getCurrentState(), sim.floorRot(), raisedAt, RobotSim.ntString("/Tuning/FourBar/Status"));
        assertEquals(FourBarState.RAISED, sim.rc.fourBarHandler.getCurrentState());
        assertEquals(raisedAt, sim.floorRot(), 0.01);
        assertTrue(RobotSim.ntString("/Tuning/FourBar/Status").contains("refused"));
    }

    @Test
    void thereIsNoCommandAnywhereThatDrivesTheFloorIntoItsHardStop() {
        RobotSim sim = RobotSim.get();
        sim.run(0.5);
        double worstAmps = 0;
        for (int i = 0; i < 250; i++) { // 5 s of mashing every button
            sim.pressDpadDown();
            sim.pressButton(RobotSim.kA);
            sim.loop();
            sim.releaseDpad();
            worstAmps = Math.max(worstAmps, Math.abs(RobotSim.nt("/Tuning/FourBar/Live/StatorCurrentAmps")));
        }
        assertTrue(worstAmps < 15.0, "unexpected current at the floor with nothing commanded: " + worstAmps);
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
