package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HopperLoadEstimatorTest {
    @Test
    void mapsHoldingCurrentToAFillFractionAfterTheWindowFills() {
        var est = new HopperLoadEstimator(0.7, 5.1, 10);
        for (int i = 0; i < 9; i++) est.update(2.9);
        assertFalse(est.isValid());
        est.update(2.9);
        assertTrue(est.isValid());
        assertEquals(0.5, est.getFraction(), 1e-9); // halfway between empty and full
    }

    @Test
    void clampsOutsideTheCalibratedRange() {
        var est = new HopperLoadEstimator(0.7, 5.1, 5);
        for (int i = 0; i < 5; i++) est.update(0.0);
        assertEquals(0.0, est.getFraction(), 1e-9);
        for (int i = 0; i < 5; i++) est.update(9.0);
        assertEquals(1.0, est.getFraction(), 1e-9);
    }

    @Test
    void exposesTheAveragedHoldingCurrent() {
        var est = new HopperLoadEstimator(0.7, 5.1, 4);
        for (int i = 0; i < 4; i++) est.update(3.2);
        assertEquals(3.2, est.getFilteredAmps(), 1e-9);
    }

    @Test
    void calibrationCanBeChangedLive() {
        var est = new HopperLoadEstimator(0.0, 10.0, 3);
        for (int i = 0; i < 3; i++) est.update(5.0);
        assertEquals(0.5, est.getFraction(), 1e-9);
        est.setCalibration(1.0, 6.0); // re-calibrated: 5 A is now nearly full
        assertEquals(0.8, est.getFraction(), 1e-9);
    }
}
