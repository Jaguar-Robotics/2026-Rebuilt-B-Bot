package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.FourBarConstants;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import frc.robot.Handlers.IntakeHandler;
import frc.robot.Handlers.IntakeHandler.IntakeState;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import org.junit.jupiter.api.Test;

/** The triggers, intake, rollers and hopper floor, with the whole robot simulated. */
class IntakeAndFloorSimTest {
    @Test
    void bootsWithTheFloorFlatAndTheIntakeStowed() {
        RobotSim sim = RobotSim.get();
        System.out.printf("[SIM] boot: floor %.4f rot (down stop %.4f), intake %.1f deg, state %s/%s%n",
            sim.floorRot(), FourBarConstants.DownStopRot, sim.intakeAngleDeg(), sim.rc.superStructure.getCurrentState(),
            sim.rc.fourBarHandler.getCurrentState());
        assertEquals(FourBarConstants.FlatRot, sim.floorRot(), 0.003, "floor should rest flat");
        assertEquals(FourBarState.FLAT, sim.rc.fourBarHandler.getCurrentState());
        assertEquals(120.0, sim.intakeAngleDeg(), 3.0, "intake should start stowed");
        assertEquals(1.0, RobotSim.nt("/Tuning/FourBar/Live/AtSetpoint") == 1.0 || "true".equals(
            String.valueOf(edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry("/Tuning/FourBar/Live/AtSetpoint").getBoolean(false))) ? 1.0 : 0.0,
            "the floor should be actively held at its flat setpoint from boot");
        assertTrue(edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry("/Tuning/Hopper/Live/LoadValid").getBoolean(false),
            "the hopper weighing should have a valid reading shortly after boot");
        assertEquals(IntakeState.OFF, IntakeHandler.getInstance().getCurrentState());
    }

    @Test
    void holdingLeftTriggerDeploysAndRunsTheIntakeWithTheFloorStayingFlatAndReleasingStowsIt() {
        RobotSim sim = RobotSim.get();
        sim.holdLeftTrigger(true);
        double deployed = sim.runUntil(() -> sim.intakeAngleDeg() < 5.0, 5);
        System.out.printf("[SIM] left trigger: intake reached %.1f deg after %.1f s, rollers %s, floor %.4f rot%n",
            sim.intakeAngleDeg(), deployed, IntakeHandler.getInstance().getCurrentState(), sim.floorRot());
        assertTrue(deployed > 0, "intake never deployed");
        assertEquals(SuperStructureState.INTAKE, sim.rc.superStructure.getCurrentState());
        assertEquals(IntakeState.FAST, IntakeHandler.getInstance().getCurrentState());

        // while intaking the floor must never tilt up
        boolean everRaised = false;
        for (int i = 0; i < 150; i++) {
            sim.loop();
            if (sim.rc.fourBarHandler.getDesiredState() == FourBarState.RAISED || sim.floorRot() > FourBarConstants.FlatRot + 0.02) everRaised = true;
        }
        assertFalse(everRaised, "the floor lifted while intaking");

        sim.holdLeftTrigger(false);
        sim.run(4.0);
        System.out.printf("[SIM] release: state %s, intake %.1f deg (stays put), rollers %s, floor %s%n", sim.rc.superStructure.getCurrentState(),
            sim.intakeAngleDeg(), IntakeHandler.getInstance().getCurrentState(), sim.rc.fourBarHandler.getCurrentState());
        assertEquals(SuperStructureState.IDLE, sim.rc.superStructure.getCurrentState(), "letting go of the trigger is idle, not stow");
        assertTrue(sim.intakeAngleDeg() < 10.0, "idle must leave the intake deployed; it was at " + sim.intakeAngleDeg());
        assertEquals(IntakeState.OFF, IntakeHandler.getInstance().getCurrentState());
        assertEquals(FourBarState.FLAT, sim.rc.fourBarHandler.getCurrentState());

        // the stow button (A): the hopper is empty, so it pulls the intake in
        sim.pressButton(RobotSim.kA);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kA);
        double stowed = sim.runUntil(() -> sim.intakeAngleDeg() > 115.0, 6);
        System.out.printf("[SIM] stow button with an empty hopper: intake back to %.1f deg after %.1f s%n", sim.intakeAngleDeg(), stowed);
        assertTrue(stowed > 0, "the stow button should stow with an empty hopper");
        assertEquals(SuperStructureState.STOWED, sim.rc.superStructure.getCurrentState());
    }

    @Test
    void leftBumperOnlyReseedsHeadingAndDoesNotTouchTheMechanisms() {
        RobotSim sim = RobotSim.get();
        sim.pressButton(RobotSim.kLeftBumper);
        sim.run(0.5);
        sim.releaseButton(RobotSim.kLeftBumper);
        sim.run(0.5);
        assertEquals(SuperStructureState.STOWED, sim.rc.superStructure.getCurrentState());
        assertEquals(120.0, sim.intakeAngleDeg(), 3.0);
        assertEquals(FourBarState.FLAT, sim.rc.fourBarHandler.getCurrentState());
    }

    @Test
    void outtakeWithAFewBallsLiftsTheFloorAlmostImmediately() {
        RobotSim sim = RobotSim.get();
        sim.rc.getFuelSimulation().setHopperCountForTesting(10);
        sim.run(2.0); // let the load estimate see 10 balls
        System.out.printf("[SIM] few balls: estimated %.1f balls (hopper really has %d)%n",
            RobotSim.nt("/Tuning/Hopper/Live/EstimatedBalls"), sim.rc.getFuelSimulation().getHopperCount());
        sim.holdRightTrigger(true);
        double raised = sim.runUntil(() -> sim.rc.fourBarHandler.getDesiredState() == FourBarState.RAISED, 4);
        System.out.printf("[SIM] floor was told to raise %.2f s after right trigger (state %s, rollers %s)%n", raised,
            sim.rc.superStructure.getCurrentState(), IntakeHandler.getInstance().getCurrentState());
        assertEquals(SuperStructureState.POOP, sim.rc.superStructure.getCurrentState());
        assertEquals(IntakeState.REVFAST, IntakeHandler.getInstance().getCurrentState());
        assertTrue(raised > 0 && raised < 0.6, "with few balls the floor should lift right away, took " + raised);
        double up = sim.runUntil(() -> sim.floorRot() > FourBarConstants.RaisedRot - 0.01, 4);
        assertTrue(up > 0, "the floor never reached the raised position");
    }

    @Test
    void outtakeFromAFullHopperWaitsUntilTheRollersHaveMadeRoomThenLiftsAndEmptiesIt() {
        RobotSim sim = RobotSim.get();
        var fuel = sim.rc.getFuelSimulation();
        fuel.setHopperCountForTesting(160);
        sim.run(3.0); // the sim mass goes to full load and the load estimate settles
        double estimated = RobotSim.nt("/Tuning/Hopper/Live/EstimatedBalls");
        System.out.printf("[SIM] full hopper: estimated %.0f balls (really %d), hold current %.2f A%n", estimated, fuel.getHopperCount(),
            RobotSim.nt("/Tuning/Hopper/Live/FilteredHoldCurrentAmps"));
        assertTrue(estimated > 140, "the floor weighing should see a nearly full hopper, saw " + estimated);

        int fieldBefore = fuel.getFuelOnFieldForTesting();
        sim.holdRightTrigger(true);
        int[] countAtRaise = {-1};
        double raised = sim.runUntil(() -> {
            if (sim.rc.fourBarHandler.getDesiredState() == FourBarState.RAISED && countAtRaise[0] < 0) countAtRaise[0] = fuel.getHopperCount();
            return countAtRaise[0] >= 0;
        }, 6);
        System.out.printf("[SIM] full hopper: floor told to raise after %.2f s, when the hopper held %d balls%n", raised, countAtRaise[0]);
        assertTrue(raised > 0.3, "the floor lifted straight away into a packed hopper (after " + raised + " s)");
        assertTrue(countAtRaise[0] < 160 && countAtRaise[0] > 100, "it should lift once about 24 balls are out, was " + countAtRaise[0]);

        sim.runUntil(() -> fuel.getHopperCount() == 0, 8);
        System.out.printf("[SIM] hopper emptied: %d left, %d fuel now on the field (was %d)%n", fuel.getHopperCount(),
            fuel.getFuelOnFieldForTesting(), fieldBefore);
        assertEquals(0, fuel.getHopperCount());
        assertEquals(160, fuel.getFuelOnFieldForTesting() - fieldBefore, 5, "the outtaken balls should appear on the field");

        sim.holdRightTrigger(false);
        double stowed = sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT
            && sim.rc.superStructure.getCurrentState() == SuperStructureState.IDLE, 6);
        assertTrue(stowed > 0, "releasing the trigger should lower the floor and go idle");
        assertTrue(sim.intakeAngleDeg() < 10.0, "and leave the intake where it is");
    }

    @Test
    void holdingBothTriggersThenReleasingOneGoesToTheOtherOne() {
        RobotSim sim = RobotSim.get();
        sim.holdLeftTrigger(true);
        sim.run(0.3);
        sim.holdRightTrigger(true);
        sim.run(0.5);
        sim.holdRightTrigger(false); // right released while left is still held: back to intaking, not stowed
        sim.run(0.5);
        assertEquals(SuperStructureState.INTAKE, sim.rc.superStructure.getCurrentState());
        sim.holdLeftTrigger(false);
        sim.run(0.5);
        assertEquals(SuperStructureState.IDLE, sim.rc.superStructure.getCurrentState());
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
