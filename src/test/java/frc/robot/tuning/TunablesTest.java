package frc.robot.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TunablesTest {
    private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void clear() {
        Tunables.clearForTesting();
    }

    @Test
    void startsAtTheDefaultAndPublishesItUnderTuning() {
        var kP = Tunables.number("TestA/Gains/kP", 12.5, 0, 100, "TestConstants.kP");
        assertEquals(12.5, kP.get(), 1e-9);
        assertEquals(12.5, nt.getEntry("/Tuning/TestA/Gains/kP").getDouble(-1), 1e-9);
    }

    @Test
    void editingTheValueInNetworkTablesAppliesItAndFiresTheCallback() {
        AtomicInteger calls = new AtomicInteger();
        double[] seen = {Double.NaN};
        var kP = Tunables.number("TestB/kP", 10.0, 0, 100, "TestConstants.kP", v -> {
            calls.incrementAndGet();
            seen[0] = v;
        });
        nt.getEntry("/Tuning/TestB/kP").setDouble(42.0); // what typing it into Elastic does
        Tunables.pollAll();
        assertEquals(42.0, kP.get(), 1e-9);
        assertEquals(1, calls.get());
        assertEquals(42.0, seen[0], 1e-9);
        Tunables.pollAll();
        assertEquals(1, calls.get(), "no change, no callback");
    }

    @Test
    void outOfRangeEditsAreClampedAndWrittenBack() {
        var limit = Tunables.number("TestC/currentLimit", 40, 5, 80, null);
        nt.getEntry("/Tuning/TestC/currentLimit").setDouble(500);
        Tunables.pollAll();
        assertEquals(80, limit.get(), 1e-9);
        assertEquals(80, nt.getEntry("/Tuning/TestC/currentLimit").getDouble(-1), 1e-9);
        nt.getEntry("/Tuning/TestC/currentLimit").setDouble(-3);
        Tunables.pollAll();
        assertEquals(5, limit.get(), 1e-9);
    }

    @Test
    void nanIsRejected() {
        var v = Tunables.number("TestD/v", 3.0, 0, 10, null);
        nt.getEntry("/Tuning/TestD/v").setDouble(Double.NaN);
        Tunables.pollAll();
        assertEquals(3.0, v.get(), 1e-9);
    }

    @Test
    void resetPutsEverythingBackToTheCode() {
        var a = Tunables.number("TestE/a", 1.0, 0, 10, null);
        var flag = Tunables.bool("TestE/flag", false, null, null);
        a.set(7.0);
        flag.set(true);
        assertEquals(2, Tunables.countModified());
        Tunables.resetAllToDefaults();
        assertEquals(1.0, a.get(), 1e-9);
        assertFalse(flag.get());
        assertEquals(0, Tunables.countModified());
    }

    @Test
    void dumpListsOnlyChangedValuesAsPasteReadyJava() {
        var kP = Tunables.number("TestF/kP", 80.0, 0, 1000, "FourBarConstants.kP");
        Tunables.number("TestF/kD", 3.0, 0, 100, "FourBarConstants.kD"); // untouched
        var spd = Tunables.number("TestF/speed", 0.5, 0, 1, "IntakeConstants.Fast");
        var loose = Tunables.number("TestF/loose", 1.0, 0, 10, null);
        kP.set(400.0);
        spd.set(0.75);
        loose.set(2.0);
        String dump = Tunables.dumpTunedConstants();
        assertTrue(dump.contains("// FourBarConstants"));
        assertTrue(dump.contains("public static final double kP = 400.0;  // was 80.0"));
        assertTrue(dump.contains("// IntakeConstants"));
        assertTrue(dump.contains("public static final double Fast = 0.75;"));
        assertFalse(dump.contains("kD"), "unchanged values must not be listed");
        assertTrue(dump.contains("TestF/loose = 2.0"), "values with no constant are still reported");
    }

    @Test
    void actionsRunOnceWhenFlippedAndResetThemselves() {
        AtomicInteger runs = new AtomicInteger();
        Tunables.action("TestG/Actions/Go", runs::incrementAndGet);
        nt.getEntry("/Tuning/TestG/Actions/Go").setBoolean(true);
        Tunables.pollAll();
        assertEquals(1, runs.get());
        assertFalse(nt.getEntry("/Tuning/TestG/Actions/Go").getBoolean(true), "the button should flip back to false");
        Tunables.pollAll();
        assertEquals(1, runs.get());
    }

    @Test
    void onlyTunedValuesAreSavedSoNewCodeDefaultsAreNeverOverridden() {
        var a = Tunables.number("TestH/a", 5.0, 0, 10, null);
        var flag = Tunables.bool("TestH/flag", false, null, null);
        assertFalse(a.isPersistent(), "an untouched value must not be saved to the roboRIO");
        assertFalse(flag.isPersistent());
        a.set(6.0);
        flag.set(true);
        assertTrue(a.isPersistent(), "a tuned value is saved so it survives a reboot");
        assertTrue(flag.isPersistent());
        Tunables.resetAllToDefaults();
        assertFalse(a.isPersistent(), "resetting stops saving it");
        assertFalse(flag.isPersistent());
    }
}
