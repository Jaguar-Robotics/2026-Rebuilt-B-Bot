package frc.robot.Handlers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.tuning.Tunables;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.api.Test;

/** State-machine behavior of the 4-bar handler, on top of the real TalonFX sim. */
class FourBarHandlerTest {
    private FourBarHandler handler;

    enum Load {
        EMPTY(FourBarConstants.MovingMassEmptyKg),
        LOADED(FourBarConstants.MovingMassLoadedKg);

        final double massKg;

        Load(double massKg) {
            this.massKg = massKg;
        }
    }

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        // These tests check the floor's state machine on its own; the intake interlock is tested separately (InterlockSimTest).
        NetworkTableInstance.getDefault().getEntry("/Tuning/Interlock/Enabled").setBoolean(false);
        frc.robot.util.MechanismInterlock.get();
        Tunables.pollAll();
        handler = FourBarHandler.getInstance();
        runUntil(() -> false, 0.5);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        Tunables.resetAllToDefaults();
        handler.closeForTesting();
    }

    private boolean runUntil(BooleanSupplier done, double timeoutSeconds) {
        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < timeoutSeconds) {
            Tunables.pollAll();
            CommandScheduler.getInstance().run();
            Timer.delay(0.02);
            if (done.getAsBoolean()) return true;
        }
        return false;
    }

    private void assertReaches(FourBarState state, double rot) {
        handler.setDesiredState(state);
        assertEquals(state, handler.getDesiredState());
        assertTrue(runUntil(() -> handler.getCurrentState() == state, 3.0), "never reached state " + state);
        assertEquals(rot, handler.getAngleRot(), FourBarConstants.PositionToleranceRot);
    }

    @Test
    void startsFlat() {
        assertEquals(FourBarState.FLAT, handler.getCurrentState());
        assertEquals(FourBarState.FLAT, handler.getDesiredState());
    }

    @ParameterizedTest @EnumSource(Load.class)
    void transitionsThroughEveryState(Load load) {
        handler.setSimMassKgForTesting(load.massKg);
        assertReaches(FourBarState.RAISED, FourBarConstants.RaisedRot);
        assertReaches(FourBarState.FLAT, FourBarConstants.FlatRot);
        assertReaches(FourBarState.RAISED, FourBarConstants.RaisedRot);
    }

    @ParameterizedTest @EnumSource(Load.class)
    void currentStateLagsDesiredUntilArrival(Load load) {
        handler.setSimMassKgForTesting(load.massKg);
        handler.setDesiredState(FourBarState.RAISED);
        assertNotEquals(FourBarState.RAISED, handler.getCurrentState(), "state flipped before the mechanism moved");
        assertTrue(runUntil(() -> handler.getCurrentState() == FourBarState.RAISED, 3.0));
    }

    @ParameterizedTest @EnumSource(Load.class)
    void zeroHereIsRefusedWhileEnabledAndNeverMovesTheFloor(Load load) {
        handler.setSimMassKgForTesting(load.massKg);
        assertReaches(FourBarState.RAISED, FourBarConstants.RaisedRot);
        Command c = handler.zeroHere();
        c.schedule();
        CommandScheduler.getInstance().run();
        double before = handler.getAngleRot();
        runUntil(() -> false, 1.0);
        assertEquals(FourBarState.RAISED, handler.getCurrentState(), "a refused zero must leave the floor raised");
        assertEquals(before, handler.getAngleRot(), 0.01);
    }

    @ParameterizedTest @EnumSource(Load.class)
    void raisedSetpointStaysWellClearOfTheUpStop(Load load) {
        handler.setSimMassKgForTesting(load.massKg);
        assertReaches(FourBarState.RAISED, FourBarConstants.RaisedRot);
        runUntil(() -> false, 1.0);
        assertTrue(FourBarConstants.UpStopRot - handler.getAngleRot() > 0.01,
            "raised floor is only " + (FourBarConstants.UpStopRot - handler.getAngleRot()) + " rot from the up stop");
    }

    @org.junit.jupiter.api.Test
    void changingASetpointInNetworkTablesMovesTheFloorToTheNewPosition() {
        assertReaches(FourBarState.RAISED, FourBarConstants.RaisedRot);
        double newRaised = FourBarConstants.RaisedRot - 0.04;
        NetworkTableInstance.getDefault().getEntry("/Tuning/FourBar/Setpoints/RaisedRot").setDouble(newRaised);
        assertTrue(runUntil(() -> Math.abs(handler.getAngleRot() - newRaised) < FourBarConstants.PositionToleranceRot, 3.0),
            "floor should follow the edited setpoint, at " + handler.getAngleRot());
        assertEquals(FourBarState.RAISED, handler.getCurrentState());
    }

    @org.junit.jupiter.api.Test
    void squareWaveAlternatesFlatAndRaisedAndEndsFlat() {
        Command c = handler.squareWave(() -> 1.2);
        c.schedule();
        assertTrue(runUntil(() -> handler.getDesiredState() == FourBarState.RAISED, 1.0));
        assertTrue(runUntil(() -> handler.getCurrentState() == FourBarState.RAISED, 3.0), "should reach raised");
        assertTrue(runUntil(() -> handler.getDesiredState() == FourBarState.FLAT, 3.0), "should come back down");
        CommandScheduler.getInstance().cancel(c);
        assertTrue(runUntil(() -> handler.getCurrentState() == FourBarState.FLAT, 3.0), "cancelling leaves it flat");
    }
}
