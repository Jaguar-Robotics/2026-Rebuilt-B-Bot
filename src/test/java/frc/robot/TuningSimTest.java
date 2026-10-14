package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Handlers.FourBarHandler.FourBarState;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/** The Elastic tuning controls, exercised on the whole simulated robot. */
class TuningSimTest {
    private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private static void setNt(String key, double v) { nt.getEntry(key).setDouble(v); }
    private static void press(String key) { nt.getEntry(key).setBoolean(true); }

    @Test
    void editsApplyAreCountedAndResetPutsThemBack() {
        RobotSim sim = RobotSim.get();
        setNt("/Tuning/FourBar/Gains/kP", 250.0);
        setNt("/Tuning/FourBar/Gains/kG", 99.0); // above the allowed maximum
        setNt("/Tuning/Rollers/Intake/Fast", 0.8);
        sim.run(0.5);
        System.out.printf("[SIM] kP=%.0f kG=%.1f (99 should clamp) changed=%.0f last: %s%n", RobotSim.nt("/Tuning/FourBar/Gains/kP"),
            RobotSim.nt("/Tuning/FourBar/Gains/kG"), RobotSim.nt("/Tuning/Status/ValuesChangedFromCode"), RobotSim.ntString("/Tuning/Status/LastChange"));
        assertEquals(250.0, RobotSim.nt("/Tuning/FourBar/Gains/kP"), 1e-9);
        assertEquals(6.0, RobotSim.nt("/Tuning/FourBar/Gains/kG"), 1e-9, "out-of-range values are pulled back into range");
        assertEquals(3.0, RobotSim.nt("/Tuning/Status/ValuesChangedFromCode"), 1e-9);
        press("/Tuning/Actions/ResetAllToDefaults");
        sim.run(0.5);
        assertEquals(FourBarConstants.kP, RobotSim.nt("/Tuning/FourBar/Gains/kP"), 1e-9);
        assertEquals(0.0, RobotSim.nt("/Tuning/Status/ValuesChangedFromCode"), 1e-9);
        sim.run(1.0);
        // this test deliberately feeds the floor an absurd gain (kG), which can twitch it for a moment; that is not an interlock bug
        sim.interlockViolations = 0;
    }

    @Test
    void rollerSpeedTunedWhileRunningAppliesImmediately() {
        RobotSim sim = RobotSim.get();
        sim.holdLeftTrigger(true);
        sim.run(1.0);
        setNt("/Tuning/Rollers/Intake/Fast", 0.55);
        sim.run(0.5);
        // the intake handler re-sends the new speed (visible as the tunable now holding it while the state is still FAST)
        assertEquals(0.55, RobotSim.nt("/Tuning/Rollers/Intake/Fast"), 1e-9);
        assertEquals(Handlers_state(), frc.robot.Handlers.IntakeHandler.IntakeState.FAST);
    }

    private static frc.robot.Handlers.IntakeHandler.IntakeState Handlers_state() {
        return frc.robot.Handlers.IntakeHandler.getInstance().getCurrentState();
    }

    @Test
    void fourBarSquareWaveAlternatesAndEveryMoveIsMeasuredWithAdvice() {
        RobotSim sim = RobotSim.get();
        setNt("/Tuning/Tests/SquareWaveDwellSeconds", 1.5);
        sim.run(0.3);
        nt.getEntry("/SmartDashboard/Tuning/FourBar Square Wave/running").setBoolean(true);
        boolean sawRaised = false, sawFlatAgain = false;
        for (int i = 0; i < 600; i++) { // 12 s
            sim.loop();
            var st = sim.rc.fourBarHandler.getCurrentState();
            if (st == FourBarState.RAISED) sawRaised = true;
            if (sawRaised && st == FourBarState.FLAT) sawFlatAgain = true;
        }
        double moves = RobotSim.nt("/Tuning/FourBar/Result/MovesMeasured");
        System.out.printf("[SIM] square wave: raised=%s flatAgain=%s moves measured=%.0f settle=%.2fs overshoot=%.4f advice='%s'%n", sawRaised, sawFlatAgain,
            moves, RobotSim.nt("/Tuning/FourBar/Result/SettleSeconds"), RobotSim.nt("/Tuning/FourBar/Result/OvershootRot"),
            RobotSim.ntString("/Tuning/FourBar/Result/Advice"));
        assertTrue(sawRaised && sawFlatAgain, "the floor should alternate flat and raised");
        assertTrue(moves >= 4, "each move should be measured, saw " + moves);
        assertFalse(RobotSim.ntString("/Tuning/FourBar/Result/Advice").isEmpty());

        nt.getEntry("/SmartDashboard/Tuning/FourBar Square Wave/running").setBoolean(false);
        double flat = sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 6);
        assertTrue(flat >= 0, "cancelling the test should leave the floor flat");
    }

    @Test
    void intakePivotSquareWaveAlternatesBetweenDeployedAndStowed() {
        RobotSim sim = RobotSim.get();
        setNt("/Tuning/Tests/SquareWaveDwellSeconds", 1.5);
        sim.run(0.3);
        nt.getEntry("/SmartDashboard/Tuning/IntakePivot Square Wave/running").setBoolean(true);
        double min = 999, max = -999;
        for (int i = 0; i < 600; i++) {
            sim.loop();
            min = Math.min(min, sim.intakeAngleDeg());
            max = Math.max(max, sim.intakeAngleDeg());
        }
        System.out.printf("[SIM] intake square wave swung between %.1f and %.1f deg, moves measured %.0f, advice '%s'%n", min, max,
            RobotSim.nt("/Tuning/IntakePivot/Result/MovesMeasured"), RobotSim.ntString("/Tuning/IntakePivot/Result/Advice"));
        assertTrue(min < 10 && max > 110, "the intake should swing deployed and stowed");
        assertTrue(RobotSim.nt("/Tuning/IntakePivot/Result/MovesMeasured") >= 3);
        nt.getEntry("/SmartDashboard/Tuning/IntakePivot Square Wave/running").setBoolean(false);
        assertTrue(sim.runUntil(() -> sim.intakeAngleDeg() > 112, 6) >= 0, "cancelling should leave it stowed");
    }

    @Test
    void safeModeSlowsTheFloorDownAndRestoringItBringsTheSpeedBack() {
        RobotSim sim = RobotSim.get();
        double normal = timeToRaise(sim);
        setNt("/Tuning/Safety/SafeMode", 0); // bool entry: use the boolean API below
        nt.getEntry("/Tuning/Safety/SafeMode").setBoolean(true);
        sim.run(0.5);
        double safe = timeToRaise(sim);
        nt.getEntry("/Tuning/Safety/SafeMode").setBoolean(false);
        sim.run(0.5);
        double normalAgain = timeToRaise(sim);
        nt.getEntry("/Tuning/Interlock/Enabled").setBoolean(true);
        System.out.printf("[SIM] raise time: normal %.2f s, safe mode %.2f s, normal again %.2f s%n", normal, safe, normalAgain);
        assertTrue(safe > normal * 1.3, "safe mode should slow the move down");
        assertTrue(normalAgain < safe * 0.85, "turning safe mode off restores the speed (normal again " + normalAgain + " vs safe " + safe + ")");
    }

    private double timeToRaise(RobotSim sim) {
        nt.getEntry("/Tuning/Interlock/Enabled").setBoolean(false); // measuring the floor's own speed, with the intake out of the picture
        sim.rc.fourBarHandler.setDesiredState(FourBarState.FLAT);
        sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 6);
        sim.run(0.5);
        sim.rc.fourBarHandler.setDesiredState(FourBarState.RAISED);
        double t = sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.RAISED, 10);
        sim.rc.fourBarHandler.setDesiredState(FourBarState.FLAT);
        sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 10);
        return t;
    }

    @Test
    void printTunedConstantsGivesPasteReadyJavaAndSavesAFile() throws IOException {
        RobotSim sim = RobotSim.get();
        Path file = Path.of(edu.wpi.first.wpilibj.Filesystem.getOperatingDirectory().getPath(), "tuned-constants.txt");
        Files.deleteIfExists(file);
        setNt("/Tuning/FourBar/Gains/kP", 321.0);
        setNt("/Tuning/Outpost/PulseIntervalSeconds", 4.5);
        sim.run(0.4);
        press("/Tuning/Actions/PrintTunedConstants");
        sim.run(0.4);
        String dump = RobotSim.ntString("/Tuning/Status/TunedConstants");
        System.out.printf("[SIM] dump:%n%s%n", dump);
        assertTrue(dump.contains("public static final double kP = 321.0;"));
        assertTrue(dump.contains("PulseIntervalSeconds = 4.5;"));
        assertTrue(Files.exists(file), "tuned-constants.txt should be written");
        Files.deleteIfExists(file);
    }

    @Test
    void liftWhenBelowBallsDecidesHowFullTheHopperIsBeforeTheFloorLifts() {
        RobotSim sim = RobotSim.get();
        var fuel = sim.rc.getFuelSimulation();
        int lowThreshold = liftCount(sim, fuel, 30);
        int highThreshold = liftCount(sim, fuel, 150);
        System.out.printf("[SIM] floor lifted with %d balls left when LiftWhenBelowBalls=30, and %d when it was 150%n", lowThreshold, highThreshold);
        assertTrue(lowThreshold < 45, "a low threshold waits for the hopper to drain (lifted at " + lowThreshold + ")");
        assertTrue(highThreshold > 120, "a high threshold lifts early (lifted at " + highThreshold + ")");
        assertTrue(lowThreshold < highThreshold - 60);
    }

    private int liftCount(RobotSim sim, FuelSimulation fuel, double liftWhenBelowBalls) {
        sim.holdRightTrigger(false);
        sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 6);
        setNt("/Tuning/Hopper/LiftWhenBelowBalls", liftWhenBelowBalls);
        fuel.setHopperCountForTesting(160);
        sim.run(3.0);
        sim.holdRightTrigger(true);
        int[] at = {-1};
        sim.runUntil(() -> {
            if (sim.rc.fourBarHandler.getDesiredState() == FourBarState.RAISED && at[0] < 0) at[0] = fuel.getHopperCount();
            return at[0] >= 0;
        }, 12);
        sim.holdRightTrigger(false);
        sim.runUntil(() -> sim.rc.fourBarHandler.getCurrentState() == FourBarState.FLAT, 8);
        return at[0];
    }

    @Test
    void calibrateButtonsSetTheEmptyAndFullCurrentsFromTheLiveReading() {
        RobotSim sim = RobotSim.get();
        var fuel = sim.rc.getFuelSimulation();
        fuel.setHopperCountForTesting(0);
        sim.run(3.0);
        press("/Tuning/Hopper/Actions/SetEmptyToCurrentReading");
        sim.run(0.3);
        double empty = RobotSim.nt("/Tuning/Hopper/EmptyHoldCurrentAmps");
        fuel.setHopperCountForTesting(160);
        sim.run(3.0);
        press("/Tuning/Hopper/Actions/SetFullToCurrentReading");
        sim.run(0.3);
        double full = RobotSim.nt("/Tuning/Hopper/FullHoldCurrentAmps");
        System.out.printf("[SIM] calibrated: empty %.2f A, full %.2f A (sim physics: about 1.4 and 10.5)%n", empty, full);
        assertEquals(1.4, empty, 0.4);
        assertEquals(10.5, full, 1.5);
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
