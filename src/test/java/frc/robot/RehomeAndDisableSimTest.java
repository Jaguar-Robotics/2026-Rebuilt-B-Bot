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
    void dpadDownWhileEnabledJogsTheFloorDownAndNeverZeroesIt() {
        RobotSim sim = RobotSim.get();
        sim.raiseFloor();
        double raisedAt = sim.floorRot();
        sim.pressDpadDown();
        sim.run(0.5);
        sim.releaseDpad();
        sim.run(0.5);
        System.out.printf("[SIM] D-pad down held 0.5 s while enabled: floor %.4f rot (was %.4f), status: %s%n", sim.floorRot(), raisedAt,
            RobotSim.ntString("/Tuning/FourBar/Status"));
        assertTrue(sim.floorRot() < raisedAt - 0.01, "D-pad down should have moved the floor down");
        assertTrue(sim.floorRot() > FourBarConstants.FlatRot + 0.02, "and only a little way, not all at once: " + sim.floorRot());
        assertTrue(!RobotSim.ntString("/Tuning/FourBar/Status").contains("Zeroed"), "enabled D-pad down must never zero the floor");
    }

    @Test
    void thereIsNoCommandAnywhereThatDrivesTheFloorIntoItsHardStop() {
        RobotSim sim = RobotSim.get();
        sim.run(0.5);
        double worstAmps = 0;
        for (int i = 0; i < 250; i++) { // 5 s of mashing every button
            sim.pressDpadDown();
            sim.pressButton(RobotSim.kBack);
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
        sim.pressBothBumpers();
        sim.run(0.1);
        sim.releaseBothBumpers();
        sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.FEED, 20);
        sim.run(1.0);
        sim.disable();
        sim.run(0.5);
        System.out.printf("[SIM] disabled during the feed: superstructure %s%n", sim.rc.superStructure.getCurrentState());
        assertEquals(SuperStructureState.IDLE, sim.rc.superStructure.getCurrentState());
        sim.enableTeleop();
        sim.run(3.0);
        assertEquals(SuperStructureState.IDLE, sim.rc.superStructure.getCurrentState(), "re-enabling must not restart the feed");
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
