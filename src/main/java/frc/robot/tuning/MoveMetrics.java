// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import java.util.Optional;

/**
 * Measures every move a position mechanism makes: how long it took to settle, how far it overshot, where it ended up
 * relative to the target, and the peak motor current. Feed it one sample per loop. When a move finishes it returns the
 * result, and {@link #advise} turns that into plain-English tuning advice.
 */
public class MoveMetrics {
    public record Result(double distance, double settleSeconds, double overshoot, double steadyStateError,
                         double peakCurrentAmps, boolean settled) {}

    private static final double kHoldSeconds = 0.25;
    private static final double kTimeoutSeconds = 4.0;

    private double lastTarget = Double.NaN;
    private boolean active = false;
    private double startTime;
    private double direction;
    private double distance;
    private double peakOvershoot;
    private double peakCurrent;
    private double inBandSince = Double.NaN;

    /**
     * @param timeSeconds a steadily increasing time
     * @param tolerance how close counts as "at the target", in the same units as position
     * @return the finished move's result on the loop it finishes, otherwise empty
     */
    public Optional<Result> update(double timeSeconds, double position, double target, double currentAmps, double tolerance) {
        if (Double.isNaN(lastTarget)) lastTarget = target;

        if (Math.abs(target - lastTarget) > 1e-9) {
            lastTarget = target;
            double toGo = target - position;
            active = Math.abs(toGo) > 1.5 * tolerance; // ignore nudges that are already inside the tolerance
            if (active) {
                startTime = timeSeconds;
                direction = Math.signum(toGo);
                distance = Math.abs(toGo);
                peakOvershoot = 0.0;
                peakCurrent = 0.0;
                inBandSince = Double.NaN;
            }
        }
        if (!active) return Optional.empty();

        peakOvershoot = Math.max(peakOvershoot, (position - target) * direction);
        peakCurrent = Math.max(peakCurrent, Math.abs(currentAmps));

        if (Math.abs(position - target) < tolerance) {
            if (Double.isNaN(inBandSince)) inBandSince = timeSeconds;
            if (timeSeconds - inBandSince >= kHoldSeconds) {
                active = false;
                return Optional.of(new Result(distance, inBandSince - startTime, peakOvershoot, position - target, peakCurrent, true));
            }
        } else {
            inBandSince = Double.NaN;
        }

        if (timeSeconds - startTime > kTimeoutSeconds) {
            active = false;
            return Optional.of(new Result(distance, timeSeconds - startTime, peakOvershoot, position - target, peakCurrent, false));
        }
        return Optional.empty();
    }

    /** Plain-English advice for a finished move. */
    public static String advise(Result r, double tolerance, double goalSettleSeconds) {
        if (!r.settled()) {
            return "Never settled. Check kG (see Suggested/kG), raise kP, and make sure the current limit is not choking it.";
        }
        StringBuilder sb = new StringBuilder();
        if (r.overshoot() > 2.0 * tolerance) {
            sb.append("Overshoot is high: lower kP or raise kD. ");
        }
        if (Math.abs(r.steadyStateError()) > 0.8 * tolerance) {
            sb.append("It rests off target: set kG to the suggested value, or add a little kI. ");
        }
        if (r.settleSeconds() > goalSettleSeconds) {
            sb.append("Slow to settle: raise cruise velocity / acceleration, or kP. ");
        }
        return sb.length() == 0 ? "Looks good." : sb.toString().trim();
    }
}
