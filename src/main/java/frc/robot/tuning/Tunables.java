// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

/**
 * Everything tunable lives under /Tuning in NetworkTables, so Elastic shows one tidy tree you can edit live.
 * Values persist across reboots, are clamped to safe ranges, and apply to the hardware the moment you change them.
 *
 * Buttons (in Elastic, flip them to true; they flip themselves back):
 * - /Tuning/Actions/PrintTunedConstants : prints every value that differs from the code as paste-ready Java and saves it to
 *                                         tuned-constants.txt next to the robot code.
 * - /Tuning/Actions/ResetAllToDefaults  : puts every tunable back to the value in Constants.
 *
 * Call {@link #periodic()} once per robot loop.
 */
public final class Tunables {
    private static final List<TunableNumber> numbers = new ArrayList<>();
    private static final List<TunableBoolean> booleans = new ArrayList<>();
    private static final List<Action> actions = new ArrayList<>();
    private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    private static final NetworkTableEntry printEntry = nt.getEntry("/Tuning/Actions/PrintTunedConstants");
    private static final NetworkTableEntry resetEntry = nt.getEntry("/Tuning/Actions/ResetAllToDefaults");
    private static final NetworkTableEntry dumpEntry = nt.getEntry("/Tuning/Status/TunedConstants");
    private static final NetworkTableEntry modifiedEntry = nt.getEntry("/Tuning/Status/ValuesChangedFromCode");
    private static final NetworkTableEntry lastChangeEntry = nt.getEntry("/Tuning/Status/LastChange");

    static {
        printEntry.setDefaultBoolean(false);
        resetEntry.setDefaultBoolean(false);
        printEntry.setBoolean(false);
        resetEntry.setBoolean(false);
    }

    private Tunables() {}

    private record Action(NetworkTableEntry entry, Runnable run) {}

    /**
     * A live-tunable number.
     * @param key where it appears: "FourBar/Gains/kP" becomes /Tuning/FourBar/Gains/kP
     * @param defaultValue the value in Constants (used when nothing has been tuned yet, and by Reset)
     * @param javaConstant "FourBarConstants.kP", used to print paste-ready code; may be null for values with no constant
     * @param onChange runs with the new value when it is edited (apply it to the hardware here); may be null
     */
    public static TunableNumber number(String key, double defaultValue, double min, double max, String javaConstant, DoubleConsumer onChange) {
        TunableNumber t = new TunableNumber(key, javaConstant, defaultValue, min, max, onChange);
        numbers.add(t);
        return t;
    }

    public static TunableNumber number(String key, double defaultValue, double min, double max, String javaConstant) {
        return number(key, defaultValue, min, max, javaConstant, null);
    }

    public static TunableBoolean bool(String key, boolean defaultValue, String javaConstant, Consumer<Boolean> onChange) {
        TunableBoolean t = new TunableBoolean(key, javaConstant, defaultValue, onChange);
        booleans.add(t);
        return t;
    }

    /** A button in Elastic: flip /Tuning/<key> to true and the action runs once (it flips itself back to false). */
    public static Object action(String key, Runnable run) {
        NetworkTableEntry entry = nt.getEntry("/Tuning/" + key);
        entry.setDefaultBoolean(false);
        entry.setBoolean(false);
        Action action = new Action(entry, run);
        actions.add(action);
        return action;
    }

    /** Stop managing these tunables (their NetworkTables entries stay). Used when a subsystem is closed, e.g. in tests. */
    public static void unregister(Iterable<?> tunables) {
        for (Object t : tunables) {
            numbers.remove(t);
            booleans.remove(t);
            actions.remove(t);
        }
    }

    private static final int kPollEveryLoops = 5; // 10 Hz is plenty for a human typing in Elastic and saves ~100 NetworkTables reads per loop
    private static int tick = 0;

    /** Call every robot loop. Does the real work every few loops. */
    public static void periodic() {
        TuningTelemetry.tick();
        if (++tick % kPollEveryLoops == 0) pollAll();
    }

    /** Checks every tunable and button right now (also what the tests call so they do not have to wait). */
    public static void pollAll() {
        for (TunableNumber t : numbers) t.poll();
        for (TunableBoolean t : booleans) t.poll();
        for (Action a : actions) {
            if (a.entry().getBoolean(false)) {
                a.entry().setBoolean(false);
                a.run().run();
            }
        }
        if (printEntry.getBoolean(false)) {
            printEntry.setBoolean(false);
            printTunedConstants();
        }
        if (resetEntry.getBoolean(false)) {
            resetEntry.setBoolean(false);
            resetAllToDefaults();
        }
        modifiedEntry.setDouble(countModified());
    }

    static void noteChange(String key, double oldValue, double newValue) {
        String message = String.format(Locale.US, "%s: %s -> %s", key, format(oldValue), format(newValue));
        lastChangeEntry.setString(message);
        DogLog.log("Tuning/LastChange", message);
    }

    public static int countModified() {
        int n = 0;
        for (TunableNumber t : numbers) if (t.isModified()) n++;
        for (TunableBoolean t : booleans) if (t.isModified()) n++;
        return n;
    }

    public static void resetAllToDefaults() {
        for (TunableNumber t : numbers) t.reset();
        for (TunableBoolean t : booleans) t.reset();
        lastChangeEntry.setString("All tunables reset to the values in Constants");
    }

    /**
     * Java lines for every tunable that differs from the code, grouped by constants class, ready to paste into
     * Constants.java. Tunables with no matching constant are listed in a comment.
     */
    public static String dumpTunedConstants() {
        Map<String, List<String>> byClass = new LinkedHashMap<>();
        List<String> noConstant = new ArrayList<>();
        for (TunableNumber t : numbers) {
            if (!t.isModified()) continue;
            addLine(byClass, noConstant, t.getJavaConstant(), t.getKey(), "double", format(t.get()), format(t.getDefault()));
        }
        for (TunableBoolean t : booleans) {
            if (!t.isModified()) continue;
            addLine(byClass, noConstant, t.getJavaConstant(), t.getKey(), "boolean", String.valueOf(t.get()), String.valueOf(t.getDefault()));
        }
        StringBuilder sb = new StringBuilder("// Tuned values that differ from Constants.java (paste over the matching lines):\n");
        if (byClass.isEmpty() && noConstant.isEmpty()) sb.append("// (none: everything matches the code)\n");
        for (var e : byClass.entrySet()) {
            sb.append("\n// ").append(e.getKey()).append('\n');
            for (String line : e.getValue()) sb.append(line).append('\n');
        }
        if (!noConstant.isEmpty()) {
            sb.append("\n// No matching constant (set these in code if you want to keep them):\n");
            for (String line : noConstant) sb.append("// ").append(line).append('\n');
        }
        return sb.toString();
    }

    private static void addLine(Map<String, List<String>> byClass, List<String> noConstant, String javaConstant,
                                String key, String type, String value, String oldValue) {
        if (javaConstant == null) {
            noConstant.add(key + " = " + value + "  (was " + oldValue + ")");
            return;
        }
        int dot = javaConstant.lastIndexOf('.');
        String cls = dot < 0 ? "Constants" : javaConstant.substring(0, dot);
        String name = dot < 0 ? javaConstant : javaConstant.substring(dot + 1);
        byClass.computeIfAbsent(cls, c -> new ArrayList<>())
            .add("public static final " + type + " " + name + " = " + value + ";  // was " + oldValue);
    }

    private static void printTunedConstants() {
        String dump = dumpTunedConstants();
        dumpEntry.setString(dump);
        System.out.println("\n===== TUNED CONSTANTS =====\n" + dump + "===========================\n");
        try {
            Files.writeString(Path.of(Filesystem.getOperatingDirectory().getPath(), "tuned-constants.txt"), dump);
        } catch (IOException | RuntimeException e) {
            System.out.println("Could not save tuned-constants.txt: " + e.getMessage());
        }
    }

    private static String format(double v) {
        if (v == Math.rint(v) && Math.abs(v) < 1e9) return String.format(Locale.US, "%.1f", v);
        return String.format(Locale.US, "%s", Double.toString(v));
    }

    /** Test hook: forget every registered tunable. */
    static void clearForTesting() {
        numbers.clear();
        booleans.clear();
        actions.clear();
    }

    /** Convenience for conditions that read a TunableBoolean. */
    public static BooleanSupplier supplier(TunableBoolean b) {
        return b::get;
    }
}
