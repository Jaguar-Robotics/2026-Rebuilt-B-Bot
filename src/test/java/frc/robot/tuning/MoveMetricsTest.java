package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.Test;

class MoveMetricsTest {
    private static final double dt = 0.02;
    private static final double tol = 0.002;

    /** Runs a synthetic move and returns the finished result. */
    private Optional<MoveMetrics.Result> run(MoveMetrics m, double target, java.util.function.DoubleUnaryOperator positionAt, double seconds) {
        Optional<MoveMetrics.Result> out = Optional.empty();
        for (double t = 0; t <= seconds; t += dt) {
            var r = m.update(t, positionAt.applyAsDouble(t), target, 3.0 + t, tol);
            if (r.isPresent()) out = r;
        }
        return out;
    }

    @Test
    void measuresSettleTimeAndOvershootOfAMove() {
        var m = new MoveMetrics();
        m.update(-1.0, 0.0, 0.0, 0.0, tol); // sitting at 0
        // Heads to 0.2, overshoots to 0.205 around t=0.5, then settles by t=1.0.
        var result = run(m, 0.2, t -> t < 0.5 ? 0.41 * t : 0.2 + 0.005 * Math.exp(-(t - 0.5) * 8) * Math.cos((t - 0.5) * 12), 2.5);
        assertTrue(result.isPresent());
        var r = result.get();
        assertTrue(r.settled());
        assertEquals(0.2, r.distance(), 1e-9);
        assertTrue(r.overshoot() > 0.003 && r.overshoot() < 0.006, "overshoot was " + r.overshoot());
        assertTrue(r.settleSeconds() > 0.5 && r.settleSeconds() < 1.5, "settle was " + r.settleSeconds());
        assertTrue(r.peakCurrentAmps() > 3.0);
    }

    @Test
    void reportsAMoveThatNeverSettles() {
        var m = new MoveMetrics();
        m.update(-1.0, 0.0, 0.0, 0.0, tol);
        var result = run(m, 0.2, t -> 0.1, 6.0); // stuck halfway
        assertTrue(result.isPresent());
        assertFalse(result.get().settled());
        assertTrue(MoveMetrics.advise(result.get(), tol, 1.5).contains("Never settled"));
    }

    @Test
    void ignoresTinyNudgesAndRepeatedSameTarget() {
        var m = new MoveMetrics();
        m.update(0.0, 0.0, 0.0, 0.0, tol);
        assertTrue(m.update(0.02, 0.0, 0.001, 1.0, tol).isEmpty()); // inside 1.5 * tolerance: not a move
        for (double t = 0.04; t < 3.0; t += dt) assertTrue(m.update(t, 0.0, 0.001, 1.0, tol).isEmpty());
    }

    @Test
    void adviceNamesTheRightKnob() {
        var overshoot = new MoveMetrics.Result(0.2, 0.8, 0.02, 0.0, 5.0, true);
        assertTrue(MoveMetrics.advise(overshoot, tol, 1.5).contains("lower kP or raise kD"));
        var offTarget = new MoveMetrics.Result(0.2, 0.8, 0.0, 0.0018, 5.0, true);
        assertTrue(MoveMetrics.advise(offTarget, tol, 1.5).contains("kG"));
        var slow = new MoveMetrics.Result(0.2, 2.5, 0.0, 0.0, 5.0, true);
        assertTrue(MoveMetrics.advise(slow, tol, 1.5).contains("Slow"));
        var good = new MoveMetrics.Result(0.2, 0.8, 0.0, 0.0002, 5.0, true);
        assertEquals("Looks good.", MoveMetrics.advise(good, tol, 1.5));
    }
}
