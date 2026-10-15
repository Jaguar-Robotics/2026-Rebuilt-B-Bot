package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;

/** A lost zero (motor reboot, or a jump in the reading) must be flagged loudly, and clear once the mechanism is zeroed again. */
class PositionLostSimTest {
    private static String warning() {
        return RobotSim.ntString("/Tuning/Warnings/PositionLostWhere");
    }

    private static boolean lost() {
        return edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry("/Tuning/Warnings/PositionLost").getBoolean(false);
    }

    @Test
    void normalDrivingAndMechanismMovesNeverFalseAlarm() {
        RobotSim sim = RobotSim.get();
        sim.holdLeftTrigger(true);
        sim.run(2.0);
        sim.holdLeftTrigger(false);
        sim.holdRightTrigger(true);
        sim.run(4.0);
        sim.holdRightTrigger(false);
        sim.run(2.0);
        assertFalse(lost(), "a false alarm during normal operation: " + warning());
        assertFalse(sim.rc.superStructure.isPositionLost());
    }

    @Test
    void aLostFloorZeroIsFlaggedRumblesTheDriverAndClearsWhenZeroed() {
        RobotSim sim = RobotSim.get();
        sim.run(0.5);
        sim.rc.fourBarHandler.markPositionLostForTesting();
        sim.run(0.3);
        assertTrue(lost());
        assertTrue(warning().contains("floor"), warning());
        assertTrue(DriverStationSim.getJoystickRumble(0, 0) > 0.5, "driver should feel a rumble");

        sim.disable();
        sim.run(0.3);
        sim.pressDpadDown();
        sim.run(0.1);
        sim.releaseDpad();
        sim.run(0.5);
        assertFalse(lost(), "zeroing the floor should clear the warning: " + warning());
        sim.enableTeleop();
    }

    @Test
    void aLostIntakeZeroIsFlaggedAndClearsWhenReZeroed() {
        RobotSim sim = RobotSim.get();
        sim.run(0.5);
        frc.robot.Handlers.IntakePivotHandler.getInstance().markPositionLostForTesting();
        sim.run(0.3);
        assertTrue(lost());
        assertTrue(warning().contains("intake"), warning());
        sim.disable();
        sim.run(0.3);
        sim.pressButton(SimAwareXboxController.SIM_BUTTON_X);
        sim.run(0.1);
        sim.releaseButton(SimAwareXboxController.SIM_BUTTON_X);
        sim.run(0.5);
        assertFalse(lost(), warning());
        sim.enableTeleop();
        assertEquals("none", warning());
    }

    @org.junit.jupiter.api.AfterEach
    void noCollisionsWhateverHappened() {
        RobotSim.assertNoViolationsNow();
    }
}
