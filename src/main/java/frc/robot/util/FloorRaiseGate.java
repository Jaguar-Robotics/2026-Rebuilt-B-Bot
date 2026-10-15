// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Decides when the hopper floor may tilt up during an outtake. Tilting a packed hopper's floor jams, so:
 * - if the measured load is below raiseBelowFraction, the floor may go up right away (a few balls need no room);
 * - otherwise it waits until the rollers have drained the hopper below that level;
 * - if the load signal is unavailable, it falls back to a short fixed amount of total outtake time; with a valid signal
 * the wait is only capped by a long safety timeout.
 * The fallback time carries over between short pulses and resets when intaking or stowed.
 */
public class FloorRaiseGate {
    private double raiseBelowFraction;
    private double fallbackSeconds;
    private double maxWaitSeconds;
    private double outtakeSeconds = 0.0;

    /**
     * @param fallbackSeconds raise after this much outtake time when there is NO usable load signal
     * @param maxWaitSeconds raise after this much outtake time even with a valid load signal, so a bad calibration cannot
     *                       keep the floor down forever
     */
    public FloorRaiseGate(double raiseBelowFraction, double fallbackSeconds, double maxWaitSeconds) {
        this.raiseBelowFraction = raiseBelowFraction;
        this.fallbackSeconds = fallbackSeconds;
        this.maxWaitSeconds = maxWaitSeconds;
    }

    /** Change the thresholds live. */
    public void setThresholds(double raiseBelowFraction, double fallbackSeconds, double maxWaitSeconds) {
        this.raiseBelowFraction = raiseBelowFraction;
        this.fallbackSeconds = fallbackSeconds;
        this.maxWaitSeconds = maxWaitSeconds;
    }

    /**
     * @param outtaking true while the rollers are running in outtake
     * @param reset true when the sequence is over (intaking or stowed)
     * @param loadValid true if loadFraction is a real measurement
     * @param loadFraction 0 = empty, 1 = full
     * @param dtSeconds time since the last call
     * @return true when the floor may be raised
     */
    public boolean update(boolean outtaking, boolean reset, boolean loadValid, double loadFraction, double dtSeconds) {
        if (reset) {
            outtakeSeconds = 0.0;
        } else if (outtaking) {
            outtakeSeconds += dtSeconds;
        }
        boolean roomForTheFloor = loadValid && loadFraction < raiseBelowFraction;
        boolean noSignalTimeout = !loadValid && outtakeSeconds >= fallbackSeconds;
        boolean safetyTimeout = outtakeSeconds >= maxWaitSeconds;
        return roomForTheFloor || noSignalTimeout || safetyTimeout;
    }
}
