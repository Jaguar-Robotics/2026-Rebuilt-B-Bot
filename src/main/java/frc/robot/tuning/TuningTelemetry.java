// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;

/** Read-only numbers and text published under /Tuning/... for Elastic graphs and displays. */
public final class TuningTelemetry {
    private static final Map<String, NetworkTableEntry> entries = new HashMap<>();

    private TuningTelemetry() {}

    private static NetworkTableEntry entry(String key) {
        return entries.computeIfAbsent(key, k -> NetworkTableInstance.getDefault().getEntry("/Tuning/" + k));
    }

    private static final Map<String, Object> last = new HashMap<>();
    private static int tick = 0;

    /** Test hook: forget what was last sent, so the next put() writes again even if the value is the same. */
    public static void forgetForTesting() {
        last.clear();
    }

    /** Call once per robot loop. */
    public static void tick() {
        tick++;
    }

    /** Only sends the value to NetworkTables if it changed since the last time. */
    public static void put(String key, double value) {
        Object old = last.put(key, value);
        if (old == null || (Double) old != value) entry(key).setDouble(value);
    }

    public static void put(String key, boolean value) {
        Object old = last.put(key, value);
        if (old == null || (Boolean) old != value) entry(key).setBoolean(value);
    }

    public static void put(String key, String value) {
        Object old = last.put(key, value);
        if (old == null || !old.equals(value)) entry(key).setString(value);
    }

    /** For values that change every loop and are only for graphs: sent every other loop (25 Hz) and only if changed. */
    public static void putLive(String key, double value) {
        if ((tick & 1) == 0) put(key, value);
    }

    public static void putLive(String key, boolean value) {
        if ((tick & 1) == 0) put(key, value);
    }
}
