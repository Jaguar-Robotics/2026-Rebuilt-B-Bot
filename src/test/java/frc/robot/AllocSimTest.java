package frc.robot;

import java.lang.management.ManagementFactory;
import org.junit.jupiter.api.Test;

/** How much memory one robot loop allocates (garbage the roboRIO has to collect). */
class AllocSimTest {
    private static long allocated() {
        var bean = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
        return bean.getThreadAllocatedBytes(Thread.currentThread().getId());
    }

    private static void measure(RobotSim sim, String label, int loops) {
        long before = allocated();
        for (int i = 0; i < loops; i++) {
            sim.robot.robotPeriodic();   // just the robot loop, no sleeping, no sim thread work
        }
        long bytes = allocated() - before;
        System.out.printf("[ALLOC] %-32s %,8d bytes per robotPeriodic()  (%.1f MB/s at 50 Hz)%n", label, bytes / loops, bytes / (double) loops * 50 / 1e6);
        org.junit.jupiter.api.Assertions.assertTrue(bytes / loops < 60_000, label + ": the robot loop is allocating too much garbage (" + bytes / loops + " bytes per loop)");
    }

    @Test
    void measure() {
        RobotSim sim = RobotSim.get();
        sim.run(8.0); // warm up
        measure(sim, "idle", 500);
        sim.drive(0.5, 0, 0.2);
        sim.run(1.0);
        measure(sim, "driving", 500);
        sim.drive(0, 0, 0);
        sim.holdLeftTrigger(true);
        sim.run(1.5);
        measure(sim, "intake deployed and running", 500);
    }
}
