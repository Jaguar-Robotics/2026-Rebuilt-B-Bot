// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.Consumer;

/** A true/false switch you can flip live from Elastic under /Tuning/... that survives reboots. See {@link Tunables#bool}. */
public class TunableBoolean {
    private final String key;
    private final String javaConstant;
    private final boolean defaultValue;
    private final NetworkTableEntry entry;
    private final Consumer<Boolean> onChange;
    private boolean value;

    TunableBoolean(String key, String javaConstant, boolean defaultValue, Consumer<Boolean> onChange) {
        this.key = key;
        this.javaConstant = javaConstant;
        this.defaultValue = defaultValue;
        this.onChange = onChange;
        this.entry = NetworkTableInstance.getDefault().getEntry("/Tuning/" + key);
        entry.setDefaultBoolean(defaultValue);
        value = entry.getBoolean(defaultValue);
        updatePersistence();
    }

    /** Only a value that differs from the code is saved; see TunableNumber. */
    private void updatePersistence() {
        if (isModified()) entry.setPersistent(); else entry.clearPersistent();
    }

    boolean isPersistent() {
        return entry.isPersistent();
    }

    public boolean get() {
        return value;
    }

    public void set(boolean newValue) {
        entry.setBoolean(newValue);
        poll();
    }

    void poll() {
        boolean raw = entry.getBoolean(value);
        if (raw == value) return;
        boolean old = value;
        value = raw;
        updatePersistence();
        Tunables.noteChange(key, old ? 1 : 0, raw ? 1 : 0);
        if (onChange != null) onChange.accept(raw);
    }

    void reset() {
        entry.setBoolean(defaultValue);
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

    boolean getDefault() {
        return defaultValue;
    }
}
