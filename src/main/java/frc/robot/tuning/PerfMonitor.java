// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Cheap loop-time profiler. Wrap a piece of the robot loop with {@code long t = PerfMonitor.now(); ...; PerfMonitor.lap("name", t);}
 * and every half second the average and worst time (milliseconds) of each section, plus the whole loop, are published under
 * /Tuning/Perf in NetworkTables, so Elastic can graph where the 20 ms goes. Call {@link #endLoop} once per robot loop.
 */
public final class PerfMonitor {
    private static final double kLoopBudgetMs = 20.0;
    private static final int kPublishEveryLoops = 25;

    private static final class Section {
        double sumMs, maxMs;
        int count;
    }

    private static final Map<String, Section> sections = new LinkedHashMap<>();
    private static int loops = 0;
    private static int overruns = 0;
    private static double loopSumMs = 0, loopMaxMs = 0;

    private PerfMonitor() {}

    public static long now() {
        return System.nanoTime();
    }

    /** Records the time since {@code startNanos} under a name and returns a fresh timestamp for chaining. */
    public static long lap(String name, long startNanos) {
        long end = System.nanoTime();
        double ms = (end - startNanos) / 1e6;
        Section s = sections.computeIfAbsent(name, k -> new Section());
        s.sumMs += ms;
        s.maxMs = Math.max(s.maxMs, ms);
        s.count++;
        return end;
    }

    /** Call once at the end of every robot loop with the timestamp taken at its start. */
    public static void endLoop(long loopStartNanos) {
        double ms = (System.nanoTime() - loopStartNanos) / 1e6;
        loopSumMs += ms;
        loopMaxMs = Math.max(loopMaxMs, ms);
        if (ms > kLoopBudgetMs) overruns++;
        if (++loops < kPublishEveryLoops) return;

        TuningTelemetry.put("Perf/Loop/AvgMs", loopSumMs / loops);
        TuningTelemetry.put("Perf/Loop/MaxMs", loopMaxMs);
        TuningTelemetry.put("Perf/Loop/OverrunsOver20ms", overruns);
        for (var e : sections.entrySet()) {
            Section s = e.getValue();
            if (s.count == 0) continue;
            TuningTelemetry.put("Perf/" + e.getKey() + "/AvgMs", s.sumMs / loops); // per loop, so the sections add up to the loop
            TuningTelemetry.put("Perf/" + e.getKey() + "/MaxMs", s.maxMs);
            s.sumMs = 0;
            s.maxMs = 0;
            s.count = 0;
        }
        loops = 0;
        loopSumMs = 0;
        loopMaxMs = 0;
    }
}
