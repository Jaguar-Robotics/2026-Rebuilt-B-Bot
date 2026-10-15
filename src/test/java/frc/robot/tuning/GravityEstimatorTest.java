package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class GravityEstimatorTest {
    private static final double dt = 0.02;

    @Test
    void recoversKgFromTheHoldingVoltage() {
        var est = new GravityEstimator(0.5, 0.5);
        double kg = 0.4, angle = 0.05; // arm slightly above horizontal
        for (int i = 0; i < 200; i++) est.update(kg * Math.cos(2 * Math.PI * angle), angle, true, dt);
        assertTrue(est.hasEstimate());
        assertEquals(kg, est.getKg(), 1e-6);
    }

    @Test
    void ignoresReadingsWhileMovingOrBeforeItHasBeenStillLongEnough() {
        var est = new GravityEstimator(0.5, 0.5);
        for (int i = 0; i < 20; i++) est.update(1.0, 0.0, true, dt); // only 0.4 s still: not trusted yet
        assertFalse(est.hasEstimate());
        for (int i = 0; i < 200; i++) est.update(9.0, 0.0, false, dt); // moving: ignored
        assertFalse(est.hasEstimate());
    }

    @Test
    void ignoresAnArmThatIsNearlyVertical() {
        var est = new GravityEstimator(0.1, 0.5);
        for (int i = 0; i < 100; i++) est.update(0.3, 0.24, true, dt); // cos(86 deg) is tiny
        assertFalse(est.hasEstimate());
    }

    @Test
    void followsAChangeInLoad() {
        var est = new GravityEstimator(0.1, 0.3);
        for (int i = 0; i < 200; i++) est.update(0.1, 0.0, true, dt); // empty: needs 0.1 V
        assertEquals(0.1, est.getKg(), 1e-3);
        for (int i = 0; i < 400; i++) est.update(0.7, 0.0, true, dt); // now full: needs 0.7 V
        assertEquals(0.7, est.getKg(), 5e-3);
    }
}
