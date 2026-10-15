package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MechanismInterlockTest {
    private static final double clear = 80.0, flatBelow = 0.002, slack = 3.0, stowed = 120.0;

    @Test
    void intakeMayStowFullyOnlyWhileTheFloorIsFlat() {
        assertEquals(stowed, MechanismInterlock.maxIntakeDeg(true, clear, flatBelow, -0.007, stowed), "flat floor: all the way up");
        assertEquals(clear, MechanismInterlock.maxIntakeDeg(true, clear, flatBelow, 0.05, stowed), "floor partly raised: stop at the clear angle");
        assertEquals(clear, MechanismInterlock.maxIntakeDeg(true, clear, flatBelow, 0.17, stowed), "floor fully raised: stop at the clear angle");
    }

    @Test
    void theFloorMayOnlyRaiseOnceTheIntakeIsClearWithSlack() {
        assertTrue(MechanismInterlock.floorMayRaise(true, clear, slack, 0.0));
        assertTrue(MechanismInterlock.floorMayRaise(true, clear, slack, 77.0));
        assertFalse(MechanismInterlock.floorMayRaise(true, clear, slack, 78.0), "waits until the intake is well inside the clear zone");
        assertFalse(MechanismInterlock.floorMayRaise(true, clear, slack, 120.0), "stowed intake: the floor must not raise");
    }

    @Test
    void disablingTheInterlockLetsEverythingMove() {
        assertEquals(stowed, MechanismInterlock.maxIntakeDeg(false, clear, flatBelow, 0.17, stowed));
        assertTrue(MechanismInterlock.floorMayRaise(false, clear, slack, 120.0));
    }

    @Test
    void theLimitNeverExceedsTheStowedAngle() {
        assertEquals(60.0, MechanismInterlock.maxIntakeDeg(true, 100.0, flatBelow, 0.17, 60.0), "a stowed setpoint inside the clear zone is not limited");
    }
}
