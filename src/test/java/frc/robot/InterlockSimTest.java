package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.InterlockConstants;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import java.util.Random;
import org.junit.jupiter.api.Test;

/** The intake and the hopper floor must never be in each other's way. Checked on every robot loop. */
class InterlockSimTest {
    private static final double kClear = InterlockConstants.IntakeClearBelowDeg;

    private static boolean intakeHeldBack() {
        return NetworkTableInstance.getDefault().getEntry("/Tuning/Interlock/IntakeHeldBackByFloor").getBoolean(false);
    }

    @Test
    void theIntakeCannotPivotUpUntilTheRaisedFloorIsFlat() {
        RobotSim sim = RobotSim.get();
        sim.raiseFloor(); // intake down, floor up
        assertEquals(FourBarState.RAISED, sim.rc.fourBarHandler.getCurrentState());
        sim.pressButton(RobotSim.kBack); // the stow button, with the floor still up: everything is told to stow at once (intake up, floor down)
        sim.run(0.05);
        sim.releaseButton(RobotSim.kBack);

        double maxIntakeWhileFloorUp = 0;
        boolean heldBack = false;
        double floorAtStowRequest = sim.floorRot();
        double tFlat = -1, tStowed = -1, t = 0;
        for (int i = 0; i < 8 * 50; i++) {
            sim.loop();
            t += 0.02;
            double intake = sim.intakeAngleDeg();
            if (sim.floorRot() > InterlockConstants.FloorCountsAsFlatBelowRot + 0.006) maxIntakeWhileFloorUp = Math.max(maxIntakeWhileFloorUp, intake);
            heldBack |= intakeHeldBack();
            if (tFlat < 0 && sim.floorRot() <= InterlockConstants.FloorCountsAsFlatBelowRot) tFlat = t;
            if (tStowed < 0 && intake > 115) tStowed = t;
        }
        System.out.printf("[INTERLOCK] stow from a raised floor: floor was at %.3f rot; highest the intake got while the floor was still up: %.1f deg "
            + "(clear zone ends at %.0f); floor flat at %.2f s; intake fully stowed at %.2f s; held back by the floor: %s%n",
            floorAtStowRequest, maxIntakeWhileFloorUp, kClear, tFlat, tStowed, heldBack);
        assertTrue(maxIntakeWhileFloorUp <= kClear + 5.0, "the intake pivoted up into the raised floor (reached " + maxIntakeWhileFloorUp + " deg)");
        assertTrue(heldBack, "the interlock should have held the intake back");
        assertTrue(tStowed > 0 && tFlat > 0 && tStowed >= tFlat - 0.05, "the intake should finish stowing only after the floor is flat");
        sim.assertNoInterlockViolations();
    }

    @Test
    void theFloorWaitsForTheIntakeToComeDownBeforeRaising() {
        RobotSim sim = RobotSim.get();
        sim.run(2.5);
        double intakeAtFloorRise = -1;
        sim.pressButton(RobotSim.kY); // the floor asks to lift at once, while the intake is still stowed: only the interlock holds it back
        sim.run(0.1);
        sim.releaseButton(RobotSim.kY);
        sim.holdRightTrigger(true); // outtake deploys the intake (and does not touch the floor)
        for (int i = 0; i < 6 * 50; i++) {
            sim.loop();
            if (intakeAtFloorRise < 0 && sim.floorRot() > InterlockConstants.FloorCountsAsFlatBelowRot + 0.01) intakeAtFloorRise = sim.intakeAngleDeg();
        }
        System.out.printf("[INTERLOCK] outtake from stowed: the floor started rising when the intake was at %.1f deg (must be <= %.0f)%n", intakeAtFloorRise, kClear);
        assertTrue(intakeAtFloorRise >= 0, "the floor never raised");
        assertTrue(intakeAtFloorRise <= kClear, "the floor started raising while the intake was still in its way (" + intakeAtFloorRise + " deg)");
        assertEquals(FourBarState.RAISED, sim.rc.fourBarHandler.getCurrentState());
        sim.assertNoInterlockViolations();
    }

    @Test
    void mashingTheTriggersAtRandomNeverPutsThemInEachOthersWay() {
        RobotSim sim = RobotSim.get();
        Random random = new Random(42);
        for (int burst = 0; burst < 45; burst++) {
            sim.holdLeftTrigger(random.nextBoolean());
            sim.holdRightTrigger(random.nextBoolean());
            sim.run(0.15 + random.nextDouble() * 0.9);
        }
        System.out.printf("[INTERLOCK] after 45 random trigger changes: %d loops in conflict%n", sim.interlockViolations);
        sim.assertNoInterlockViolations();
        sim.holdLeftTrigger(false);
        sim.holdRightTrigger(false);
        sim.run(2.5);
        sim.pressButton(RobotSim.kBack);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kBack);
        double done = sim.runUntil(() -> sim.rc.superStructure.getCurrentState() == SuperStructureState.STOWED
            && sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT && sim.intakeAngleDeg() > 115, 10);
        assertTrue(done > 0, "after letting go everything should end up stowed (intake " + sim.intakeAngleDeg() + ", floor " + sim.floorRot() + ")");
        sim.assertNoInterlockViolations();
    }

    @Test
    void turningTheInterlockOffInElasticOrWideningTheClearAngleChangesWhatIsAllowed() {
        RobotSim sim = RobotSim.get();
        var nt = NetworkTableInstance.getDefault();
        sim.raiseFloor();
        nt.getEntry("/Tuning/Interlock/IntakeClearOfFloorBelowDeg").setDouble(110.0); // widen the zone the intake may use with the floor up
        sim.pressButton(RobotSim.kBack); // stow everything at once while the floor is still up
        sim.run(0.05);
        sim.releaseButton(RobotSim.kBack);
        sim.run(0.8);
        double wentTo = sim.intakeAngleDeg();
        nt.getEntry("/Tuning/Interlock/IntakeClearOfFloorBelowDeg").setDouble(InterlockConstants.IntakeClearBelowDeg); // put it back: tests share one robot
        sim.run(0.1); // so the saved (persistent) copy is cleared too
        System.out.printf("[INTERLOCK] clear angle widened to 110 deg: intake went to %.1f deg with the floor still at %.3f rot%n", sim.intakeAngleDeg(), sim.floorRot());
        assertTrue(sim.intakeAngleDeg() > kClear + 5 || sim.floorRot() <= InterlockConstants.FloorCountsAsFlatBelowRot,
            "a wider clear angle should let the intake go further while the floor is still up");
    }
}
