package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Profiles the real robot loop in simulation (relative numbers: this is a Mac, not a roboRIO). */
class PerfSimTest {
    private static void report(String label) {
        var nt = NetworkTableInstance.getDefault();
        List<String> names = new ArrayList<>();
        for (var t : nt.getTopics("/Tuning/Perf/")) {
            if (t.getName().endsWith("/AvgMs")) names.add(t.getName().substring("/Tuning/Perf/".length(), t.getName().length() - "/AvgMs".length()));
        }
        Collections.sort(names);
        System.out.printf("[PERF] ---- %s ----%n", label);
        System.out.printf("[PERF] whole loop: avg %.2f ms, worst %.2f ms, loops over 20 ms: %.0f%n", RobotSim.nt("/Tuning/Perf/Loop/AvgMs"),
            RobotSim.nt("/Tuning/Perf/Loop/MaxMs"), RobotSim.nt("/Tuning/Perf/Loop/OverrunsOver20ms"));
        for (String n : names) {
            if (n.equals("Loop")) continue;
            System.out.printf("[PERF]   %-34s avg %6.3f ms   worst %6.2f ms%n", n, RobotSim.nt("/Tuning/Perf/" + n + "/AvgMs"), RobotSim.nt("/Tuning/Perf/" + n + "/MaxMs"));
        }
    }

    @Test
    void profile() {
        RobotSim sim = RobotSim.get();
        sim.run(3.0);   // warm up the JIT
        sim.run(5.0);
        report("idle (enabled, nothing happening)");
        org.junit.jupiter.api.Assertions.assertTrue(RobotSim.nt("/Tuning/Perf/Loop/AvgMs") < 8.0,
            "the average robot loop should stay far below the 20 ms budget even on a busy test machine");

        sim.drive(0.5, 0, 0.2);
        sim.run(5.0);
        report("driving");
        sim.drive(0, 0, 0);

        sim.rc.getFuelSimulation().setHopperCountForTesting(160);
        sim.holdLeftTrigger(true);
        sim.run(2.0);
        sim.holdLeftTrigger(false);
        sim.holdRightTrigger(true);
        sim.run(3.0);
        report("outtaking 160 fuel (lots of fuel physics)");
        sim.holdRightTrigger(false);
        sim.run(2.0);

        for (int i = 0; i < 4; i++) sim.run(3.0);
        report("idle again, fuel scattered on the field");
    }
}
