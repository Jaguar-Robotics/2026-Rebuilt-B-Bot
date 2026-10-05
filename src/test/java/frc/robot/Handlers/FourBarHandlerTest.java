package frc.robot.Handlers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
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
        handler = FourBarHandler.getInstance();
        runUntil(() -> false, 0.5);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        handler.closeForTesting();
    }

    private boolean runUntil(BooleanSupplier done, double timeoutSeconds) {
        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < timeoutSeconds) {
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
    void startsStowed() {
        assertEquals(FourBarState.STOWED, handler.getCurrentState());
        assertEquals(FourBarState.STOWED, handler.getDesiredState());
    }

    @ParameterizedTest @EnumSource(Load.class)
    void transitionsThroughEveryState(Load load) {
        handler.setSimMassKgForTesting(load.massKg);
        assertReaches(FourBarState.INTAKE, FourBarConstants.IntakeRot);
        assertReaches(FourBarState.STOWED, FourBarConstants.StowRot);
        assertReaches(FourBarState.INTAKE, FourBarConstants.IntakeRot);
    }

    @ParameterizedTest @EnumSource(Load.class)
    void currentStateLagsDesiredUntilArrival(Load load) {
        handler.setSimMassKgForTesting(load.massKg);
        handler.setDesiredState(FourBarState.INTAKE);
        assertNotEquals(FourBarState.INTAKE, handler.getCurrentState(), "state flipped before the mechanism moved");
        assertTrue(runUntil(() -> handler.getCurrentState() == FourBarState.INTAKE, 3.0));
    }

    @Test
    void homingIsNotARequestableState() {
        handler.setDesiredState(FourBarState.HOMING);
        assertEquals(FourBarState.STOWED, handler.getDesiredState());
    }

    @ParameterizedTest @EnumSource(Load.class)
    void rehomeEndsStowed(Load load) {
        handler.setSimMassKgForTesting(load.massKg);
        assertReaches(FourBarState.INTAKE, FourBarConstants.IntakeRot);
        Command c = handler.rehome();
        c.schedule();
        CommandScheduler.getInstance().run();
        assertEquals(FourBarState.HOMING, handler.getCurrentState());
        assertTrue(runUntil(() -> !c.isScheduled(), FourBarConstants.HomingTimeoutSeconds + 1));
        assertTrue(runUntil(() -> handler.getCurrentState() == FourBarState.STOWED, 3.0), "not STOWED after re-home");
        assertEquals(FourBarConstants.StowRot, handler.getAngleRot(), FourBarConstants.PositionToleranceRot);
    }
}
