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

        // the stow button (A) pulls the intake in
        sim.pressButton(RobotSim.kBack);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kBack);
        double stowed = sim.runUntil(() -> sim.intakeAngleDeg() > 115.0, 6);
        System.out.printf("[SIM] stow button: intake back to %.1f deg after %.1f s%n", sim.intakeAngleDeg(), stowed);
        assertTrue(stowed > 0, "the stow button should stow the intake");
        assertEquals(SuperStructureState.STOWED, sim.rc.superStructure.getCurrentState());
    }

    @Test
    void startButtonOnlyReseedsHeadingAndDoesNotTouchTheMechanisms() {
        RobotSim sim = RobotSim.get();
        sim.pressButton(RobotSim.kStart);
        sim.run(0.5);
        sim.releaseButton(RobotSim.kStart);
        sim.run(0.5);
        assertEquals(SuperStructureState.STOWED, sim.rc.superStructure.getCurrentState());
        assertEquals(120.0, sim.intakeAngleDeg(), 3.0);
        assertEquals(FourBarState.FLAT, sim.rc.fourBarHandler.getCurrentState());
    }

    @Test
    void rightTriggerOuttakeRunsTheRollersEmptiesTheHopperAndNeverTouchesTheFloor() {
        RobotSim sim = RobotSim.get();
        var fuel = sim.rc.getFuelSimulation();
        sim.setHopperBalls(160);
        sim.run(1.0);
        int fieldBefore = fuel.getFuelOnFieldForTesting();
        sim.holdRightTrigger(true);
        boolean everRaised = false;
        for (int i = 0; i < 300; i++) { // 6 s
            sim.loop();
            if (sim.rc.fourBarHandler.getDesiredState() == FourBarState.RAISED || sim.floorRot() > FourBarConstants.FlatRot + 0.01) everRaised = true;
        }
        assertEquals(SuperStructureState.POOP, sim.rc.superStructure.getCurrentState());
        assertEquals(IntakeState.REVFAST, IntakeHandler.getInstance().getCurrentState());
        assertFalse(everRaised, "the right trigger outtake must not move the floor");
        assertEquals(0, fuel.getHopperCount(), "the hopper should have emptied");
        assertEquals(160, fuel.getFuelOnFieldForTesting() - fieldBefore, 5, "the outtaken balls should appear on the field");
        sim.holdRightTrigger(false);
        sim.run(1.0);
        assertEquals(SuperStructureState.IDLE, sim.rc.superStructure.getCurrentState());
        assertTrue(sim.intakeAngleDeg() < 10.0, "idle leaves the intake where it is");
    }

    @Test
    void yTogglesTheFloorUpAllTheWayAndBackDown() {
        RobotSim sim = RobotSim.get();
        sim.raiseFloor();
        assertEquals(FourBarState.RAISED, sim.rc.fourBarHandler.getCurrentState());
        assertTrue(sim.floorRot() > FourBarConstants.RaisedRot - 0.01, "the floor should be all the way up, at " + sim.floorRot());
        sim.run(1.0); // letting go of nothing: the floor stays up (idle does not touch it)
        assertTrue(sim.floorRot() > FourBarConstants.RaisedRot - 0.01, "the floor must stay up");
        sim.pressButton(RobotSim.kY);
        sim.run(0.1);
        sim.releaseButton(RobotSim.kY);
        assertTrue(sim.runUntil(() -> sim.floorRot() < FourBarConstants.FlatRot + 0.01, 6) > 0, "a second press should lower the floor");
        assertTrue(sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 4) >= 0, "and settle flat");
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
