// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import java.util.ArrayList;
import java.util.List;

/**
 * Safe tuning mode. Turn on /Tuning/Safety/SafeMode in Elastic before the first test of a new gain: it caps the motor
 * current limits and scales down mechanism speeds, so a bad value cannot hit a hard stop at full power.
 */
public final class TuningMode {
    private static final List<Runnable> listeners = new ArrayList<>();

    public static final TunableBoolean safeMode = Tunables.bool("Safety/SafeMode", false, null, v -> notifyListeners());
    public static final TunableNumber safeCurrentLimitAmps =
        Tunables.number("Safety/SafeCurrentLimitAmps", 20.0, 5.0, 60.0, null, v -> notifyListeners());
    public static final TunableNumber safeSpeedScale =
        Tunables.number("Safety/SafeSpeedScale", 0.5, 0.1, 1.0, null, v -> notifyListeners());

    private TuningMode() {}

    public static boolean isSafe() {
        return safeMode.get();
    }

    /** A motor current limit, capped in safe mode. */
    public static double limitAmps(double amps) {
        return isSafe() ? Math.min(amps, safeCurrentLimitAmps.get()) : amps;
    }

    /** A speed, acceleration or voltage, scaled down in safe mode. */
    public static double scale(double value) {
        return isSafe() ? value * safeSpeedScale.get() : value;
    }

    /** Run this whenever safe mode (or its limits) change, so the hardware gets the new effective values. */
    public static void onChange(Runnable reapply) {
        listeners.add(reapply);
    }

    public static void removeListener(Runnable reapply) {
        listeners.remove(reapply);
    }

    private static void notifyListeners() {
        for (Runnable r : new ArrayList<>(listeners)) r.run();
    }
}
