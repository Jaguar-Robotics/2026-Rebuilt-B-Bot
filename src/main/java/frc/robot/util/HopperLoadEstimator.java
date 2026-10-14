// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;

/**
 * Estimates how full the hopper is from the current the 4-bar motors need to hold the floor still at stow. The weight of
 * the fuel on the floor is the load, so holding current rises linearly from the empty value to the full value.
 * Only feed it samples taken while the floor is stationary at stow.
 */
public class HopperLoadEstimator {
    private double emptyAmps;
    private double fullAmps;
    private final int windowSize;
    private final LinearFilter filter;
    private int samples = 0;
    private double filteredAmps = 0.0;

    public HopperLoadEstimator(double emptyAmps, double fullAmps, int windowSize) {
        this.emptyAmps = emptyAmps;
        this.fullAmps = fullAmps;
        this.windowSize = windowSize;
        this.filter = LinearFilter.movingAverage(windowSize);
    }

    /** Change the calibration live (empty and full holding currents). */
    public void setCalibration(double emptyAmps, double fullAmps) {
        this.emptyAmps = emptyAmps;
        this.fullAmps = Math.max(fullAmps, emptyAmps + 1e-6);
    }

    /** @param holdCurrentAmps average stator current of the 4-bar motors while holding the floor still at stow */
    public void update(double holdCurrentAmps) {
        filteredAmps = filter.calculate(holdCurrentAmps);
        if (samples < windowSize) samples++;
    }

    /** The averaged holding current in amps (meaningless until isValid()). */
    public double getFilteredAmps() {
        return filteredAmps;
    }

    /** true once the filter window is full of real samples. */
    public boolean isValid() {
        return samples >= windowSize;
    }

    /** 0 = empty, 1 = full (clamped). Meaningless until isValid(). */
    public double getFraction() {
        return MathUtil.clamp((filteredAmps - emptyAmps) / (fullAmps - emptyAmps), 0.0, 1.0);
    }
}
