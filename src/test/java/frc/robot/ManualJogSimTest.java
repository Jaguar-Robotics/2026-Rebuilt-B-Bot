package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.FourBarConstants;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import org.junit.jupiter.api.Test;

/** The D-pad manual overrides (left/right = intake, up/down = floor) must never be able to damage anything. */
class ManualJogSimTest {
    private static void deployIntake(RobotSim sim) {
        sim.holdLeftTrigger(true);
        sim.runUntil(() -> sim.intakeAngleDeg() < 5.0, 5);
        sim.holdLeftTrigger(false);
        sim.run(0.5);
    }

    @Test
    void floorJogUpStopsWellShortOfTheUpStopAndHoldsWhereItIsLeftAndJogDownStopsAboveTheDownStop() {
        RobotSim sim = RobotSim.get();
        deployIntake(sim);
        sim.pressPov(0); // D-pad up, held for 10 s
        sim.run(10.0);
        double top = sim.floorRot();
        System.out.printf("[JOG] floor held up for 10 s ended at %.4f rot (up stop %.4f)%n", top, FourBarConstants.UpStopRot);
        assertTrue(top > 0.05, "the floor should have moved up, at " + top);
        assertTrue(top < FourBarConstants.UpStopRot - 0.006, "the floor must stop short of the up stop, at " + top);
        sim.releaseDpad();
        sim.run(1.5);
        assertEquals(top, sim.floorRot(), 0.01, "released: the floor must hold where it is");
        sim.pressPov(180); // D-pad down, held 12 s
        sim.run(12.0);
        System.out.printf("[JOG] floor held down for 12 s ended at %.4f rot (down stop %.4f)%n", sim.floorRot(), FourBarConstants.DownStopRot);
        assertTrue(sim.floorRot() > FourBarConstants.DownStopRot + 0.001, "the floor must stop above the down stop, at " + sim.floorRot());
        assertTrue(sim.floorRot() < FourBarConstants.FlatRot + 0.01, "and come all the way down");
        sim.releaseDpad();
    }

    @Test
    void floorJogUpDoesNothingWhileTheIntakeIsStowedBecauseTheyWouldCollide() {
        RobotSim sim = RobotSim.get();
        sim.pressPov(0);
        sim.run(5.0);
        sim.releaseDpad();
        assertTrue(sim.floorRot() < FourBarConstants.FlatRot + 0.01, "the floor must not lift into a stowed intake, at " + sim.floorRot());
        sim.assertNoInterlockViolations();
    }

    @Test
    void intakeJogStaysInsideItsLimitsAndTheFloorGetsOutOfTheWayBeforeItStowsLikeEverythingElse() {
        RobotSim sim = RobotSim.get();
        deployIntake(sim);
        sim.pressPov(270); // D-pad left (deploy) held: it is already deployed, so it must not go past the deployed stop
        sim.run(4.0);
        assertTrue(sim.intakeAngleDeg() > -3.0, "the intake went past its deployed limit: " + sim.intakeAngleDeg());
        sim.releaseDpad();

        sim.raiseFloor(); // floor up, intake down
        assertEquals(FourBarState.RAISED, sim.rc.fourBarHandler.getCurrentState());
        sim.pressPov(90); // D-pad right (toward stowed) held for 8 s with the floor up
        double maxIntakeWhileFloorUp = 0;
        for (int i = 0; i < 8 * 50; i++) {
            sim.loop();
            if (sim.floorRot() > Constants.InterlockConstants.FloorCountsAsFlatBelowRot + 0.006) maxIntakeWhileFloorUp = Math.max(maxIntakeWhileFloorUp, sim.intakeAngleDeg());
        }
        System.out.printf("[JOG] intake jogged toward stow with the floor raised: highest while the floor was still up %.1f deg, ended at %.1f deg%n",
            maxIntakeWhileFloorUp, sim.intakeAngleDeg());
        assertTrue(maxIntakeWhileFloorUp <= Constants.InterlockConstants.IntakeClearBelowDeg + 5.0, "the intake jogged into the raised floor: " + maxIntakeWhileFloorUp);
        sim.releaseDpad();
        sim.assertNoInterlockViolations();
    }

    @Test
    void intakeJogReachesButNotPastTheStowedLimitWhenTheFloorIsFlat() {
        RobotSim sim = RobotSim.get();
        deployIntake(sim);
        sim.pressPov(90);
        sim.run(12.0);
        sim.releaseDpad();
        System.out.printf("[JOG] intake jogged toward stow for 12 s with the floor flat: %.1f deg%n", sim.intakeAngleDeg());
        assertTrue(sim.intakeAngleDeg() > 100.0, "it should have moved toward stowed: " + sim.intakeAngleDeg());
        assertTrue(sim.intakeAngleDeg() < 123.0, "it must not go past the stowed limit: " + sim.intakeAngleDeg());
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
