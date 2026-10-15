// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

/**
 * Works out the kG a mechanism actually needs, without moving it. When an arm sits still on target, the voltage the motor
 * is applying is exactly the voltage needed to hold it against gravity, which for an arm is kG * cos(angle). So
 * kG = voltage / cos(angle). It averages over time and only trusts readings taken while the mechanism is stationary and
 * not near vertical (where cos is tiny).
 */
public class GravityEstimator {
    private final double minStationarySeconds;
    private final double averagingSeconds;
    private double stationarySeconds = 0.0;
    private double averageKg = Double.NaN;

    public GravityEstimator(double minStationarySeconds, double averagingSeconds) {
        this.minStationarySeconds = minStationarySeconds;
        this.averagingSeconds = averagingSeconds;
    }

    /**
     * @param appliedVolts the motor voltage being applied
     * @param angleRot the arm angle in rotations, 0 = horizontal
     * @param stationary true while the arm is on target and not moving
     */
    public void update(double appliedVolts, double angleRot, boolean stationary, double dtSeconds) {
        if (!stationary) {
            stationarySeconds = 0.0;
            return;
        }
        stationarySeconds += dtSeconds;
        if (stationarySeconds < minStationarySeconds) return;
        double cos = Math.cos(2 * Math.PI * angleRot);
        if (Math.abs(cos) < 0.25) return;
        double sample = appliedVolts / cos;
        if (Double.isNaN(averageKg)) {
            averageKg = sample;
        } else {
            double alpha = dtSeconds / (averagingSeconds + dtSeconds);
            averageKg += alpha * (sample - averageKg);
        }
    }

    public boolean hasEstimate() {
        return !Double.isNaN(averageKg);
    }

    /** The estimated kG in volts, or NaN until there has been a stationary reading. */
    public double getKg() {
        return averageKg;
    }

    public void reset() {
        stationarySeconds = 0.0;
        averageKg = Double.NaN;
    }
}
