// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.DoubleConsumer;

/**
 * A number you can change live from Elastic (or any NetworkTables editor) under /Tuning/... The value survives reboots
 * (it is a persistent NetworkTables entry), is clamped to [min, max], and optionally calls back when it changes.
 * Create these through {@link Tunables#number}.
 */
public class TunableNumber {
    private final String key;
    private final String javaConstant;
    private final double defaultValue;
    private final double min;
    private final double max;
    private final NetworkTableEntry entry;
    private final DoubleConsumer onChange;
    private double value;

    TunableNumber(String key, String javaConstant, double defaultValue, double min, double max, DoubleConsumer onChange) {
        this.key = key;
        this.javaConstant = javaConstant;
        this.defaultValue = defaultValue;
        this.min = min;
        this.max = max;
        this.onChange = onChange;
        this.entry = NetworkTableInstance.getDefault().getEntry("/Tuning/" + key);
        entry.setDefaultDouble(defaultValue);
        double raw = entry.getDouble(defaultValue);
        value = sanitize(raw, defaultValue);
        if (value != raw) entry.setDouble(value);
        updatePersistence();
    }

    /**
     * Only a value that differs from the code is saved to the roboRIO. A value that matches the code is never saved, so
     * changing a constant in Constants.java is not overridden by an old saved copy of the previous default.
     */
    private void updatePersistence() {
        if (isModified()) entry.setPersistent(); else entry.clearPersistent();
    }

    boolean isPersistent() {
        return entry.isPersistent();
    }

    private double sanitize(double raw, double fallback) {
        return Double.isNaN(raw) || Double.isInfinite(raw) ? fallback : MathUtil.clamp(raw, min, max);
    }

    /** The current (clamped) value. */
    public double get() {
        return value;
    }

    /** Sets the value from code (writes it to NetworkTables, like typing it in Elastic). */
    public void set(double newValue) {
        entry.setDouble(sanitize(newValue, value));
        poll();
    }

    /** Called every loop by {@link Tunables#periodic()}: notices edits, clamps them, and fires the callback. */
    void poll() {
        double raw = entry.getDouble(value);
        if (raw == value) return;
        double clamped = sanitize(raw, value);
        if (clamped != raw) entry.setDouble(clamped); // pull an out-of-range edit back inside the allowed range
        if (clamped == value) return;
        double old = value;
        value = clamped;
        updatePersistence();
        Tunables.noteChange(key, old, clamped);
        if (onChange != null) onChange.accept(clamped);
    }

    void reset() {
        entry.setDouble(defaultValue);
        poll();
    }

    boolean isModified() {
        return value != defaultValue;
    }

    String getKey() {
        return key;
    }

    String getJavaConstant() {
        return javaConstant;
    }

    double getDefault() {
        return defaultValue;
    }
}
