package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.FourBarConstants;
import frc.robot.Constants.FuelConstants;
import org.junit.jupiter.api.Test;

class FuelHopperTest {
    private FuelHopper newHopper() {
        return new FuelHopper(FuelConstants.HopperCapacity, FourBarConstants.MovingMassEmptyKg, FourBarConstants.MovingMassLoadedKg);
    }

    @Test
    void intakesUntilFullThenRefuses() {
        var hopper = newHopper();
        for (int i = 0; i < FuelConstants.HopperCapacity; i++) {
            assertTrue(hopper.intake(), "should still have room at fuel " + i);
        }
        assertTrue(hopper.isFull());
        assertEquals(160, hopper.getCount());
        assertFalse(hopper.intake());
        assertEquals(160, hopper.getCount());
    }

    @Test
    void outtakesUntilEmptyThenRefuses() {
        var hopper = newHopper();
        hopper.intake();
        hopper.intake();
        assertTrue(hopper.outtake());
        assertTrue(hopper.outtake());
        assertTrue(hopper.isEmpty());
        assertFalse(hopper.outtake());
        assertEquals(0, hopper.getCount());
    }

    @Test
    void massGoesFromEmptyToLoaded() {
        var hopper = newHopper();
        assertEquals(FourBarConstants.MovingMassEmptyKg, hopper.getMassKg(), 1e-9);
        for (int i = 0; i < FuelConstants.HopperCapacity / 2; i++) hopper.intake();
        assertEquals((FourBarConstants.MovingMassEmptyKg + FourBarConstants.MovingMassLoadedKg) / 2, hopper.getMassKg(), 1e-9);
        while (hopper.intake()) { /* fill */ }
        assertEquals(FourBarConstants.MovingMassLoadedKg, hopper.getMassKg(), 1e-9);
    }
}
