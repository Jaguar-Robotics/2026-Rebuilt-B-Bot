package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FloorRaiseGateTest {
    private static final double dt = 0.02;

    @Test
    void fewBallsMeansTheFloorMayLiftImmediately() {
        var gate = new FloorRaiseGate(0.85, 2.0, 6.0);
        assertTrue(gate.update(true, false, true, 0.10, dt));
    }

    @Test
    void packedHopperWaitsUntilTheRollersHaveDrainedIt() {
        var gate = new FloorRaiseGate(0.85, 2.0, 6.0);
        assertFalse(gate.update(true, false, true, 1.0, dt));
        assertFalse(gate.update(true, false, true, 0.90, dt));
        assertTrue(gate.update(true, false, true, 0.84, dt)); // room has been made
    }

    @Test
    void withoutAUsableLoadSignalItFallsBackToATimer() {
        var gate = new FloorRaiseGate(0.85, 1.0, 6.0);
        for (int i = 0; i < 49; i++) assertFalse(gate.update(true, false, false, 0.0, dt));
        assertTrue(gate.update(true, false, false, 0.0, dt * 2));
    }

    @Test
    void fallbackTimeCarriesAcrossPulsesAndResetsWhenDone() {
        var gate = new FloorRaiseGate(0.85, 1.0, 6.0);
        for (int pulse = 0; pulse < 2; pulse++) {
            for (int i = 0; i < 25; i++) gate.update(true, false, false, 0.0, dt);
            for (int i = 0; i < 150; i++) gate.update(false, false, false, 0.0, dt);
        }
        assertTrue(gate.update(false, false, false, 0.0, dt));
        assertFalse(gate.update(false, true, false, 0.0, dt)); // stowed / intaking: starts over
    }

    @Test
    void aValidLoadSignalIsNotCutShortByTheNoSignalFallbackButHasASafetyTimeout() {
        var gate = new FloorRaiseGate(0.2, 2.0, 5.0);
        boolean early = false;
        for (int i = 0; i < 200; i++) early |= gate.update(true, false, true, 0.9, dt); // 4 s of outtake, hopper still 90% full
        assertFalse(early, "with a valid signal the floor should keep waiting past the no-signal fallback time");
        boolean late = false;
        for (int i = 0; i < 100; i++) late |= gate.update(true, false, true, 0.9, dt); // 6 s total: past the safety timeout
        assertTrue(late, "the safety timeout must eventually let the floor up");
    }
}
