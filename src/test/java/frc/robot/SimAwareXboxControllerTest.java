package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** The sim axis remap: raw sim indices in, standard Xbox meaning out. */
class SimAwareXboxControllerTest {
    private SimAwareXboxController controller;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.setJoystickAxisCount(0, 6);
        // Everything at rest: sticks 0, triggers at -1 (as the sim GUI reports them).
        for (int i = 0; i < 4; i++) DriverStationSim.setJoystickAxis(0, i, 0.0);
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_LEFT_TRIGGER, -1.0);
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_RIGHT_TRIGGER, -1.0);
        update();
        controller = new SimAwareXboxController(0);
    }

    private void update() {
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    @Test
    void sticksReadFromTheSimIndices() {
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_LEFT_X, 0.25);
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_LEFT_Y, -0.5);
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_RIGHT_X, 0.75);
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_RIGHT_Y, -1.0);
        update();
        assertEquals(0.25, controller.getLeftX(), 1e-6);
        assertEquals(-0.5, controller.getLeftY(), 1e-6);
        assertEquals(0.75, controller.getRightX(), 1e-6);
        assertEquals(-1.0, controller.getRightY(), 1e-6);
    }

    @Test
    void triggersAtRestDoNotLookLikeASpinningStickOrAPress() {
        // Right stick rest must be 0 even though standard axis 4 (RX) is the sim's left trigger, resting at -1.
        assertEquals(0.0, controller.getRightX(), 1e-6);
        assertEquals(0.0, controller.getLeftTriggerAxis(), 1e-6);
        assertEquals(0.0, controller.getRightTriggerAxis(), 1e-6);
        assertFalse(controller.leftTrigger().getAsBoolean());
        assertFalse(controller.rightTrigger().getAsBoolean());
    }

    @Test
    void triggersRescaleToZeroOne() {
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_LEFT_TRIGGER, 1.0);
        DriverStationSim.setJoystickAxis(0, SimAwareXboxController.SIM_RIGHT_TRIGGER, 0.0);
        update();
        assertEquals(1.0, controller.getLeftTriggerAxis(), 1e-6);
        assertEquals(0.5, controller.getRightTriggerAxis(), 1e-6);
        assertTrue(controller.leftTrigger().getAsBoolean());
        assertFalse(controller.rightTrigger().getAsBoolean()); // exactly at the 0.5 threshold is not a press
        assertEquals(1.0, controller.getRawAxis(2), 1e-6); // standard LT index maps to the sim trigger
    }
}
