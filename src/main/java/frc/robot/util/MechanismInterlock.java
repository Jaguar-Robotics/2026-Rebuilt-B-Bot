// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.InterlockConstants;
import frc.robot.tuning.TunableBoolean;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;

/**
 * Keeps the intake pivot and the hopper floor from colliding.
 * - The intake may only pivot past "clear below" degrees (toward stowed) while the floor is flat.
 * - The floor may only raise while the intake is at or below "clear below" (minus a little slack).
 * The two limits live under /Tuning/Interlock in Elastic.
 */
public final class MechanismInterlock {
    private static MechanismInterlock instance;

    public static synchronized MechanismInterlock get() {
        if (instance == null) instance = new MechanismInterlock();
        return instance;
    }

    private final TunableBoolean enabled = Tunables.bool("Interlock/Enabled", InterlockConstants.Enabled, "InterlockConstants.Enabled", null);
    private final TunableNumber intakeClearBelowDeg = Tunables.number("Interlock/IntakeClearOfFloorBelowDeg",
        InterlockConstants.IntakeClearBelowDeg, 0, 120, "InterlockConstants.IntakeClearBelowDeg");
    private final TunableNumber floorFlatBelowRot = Tunables.number("Interlock/FloorCountsAsFlatBelowRot",
        InterlockConstants.FloorCountsAsFlatBelowRot, -0.0105, 0.174, "InterlockConstants.FloorCountsAsFlatBelowRot");
    private final TunableNumber intakeSlackDeg = Tunables.number("Interlock/IntakeSlackDeg",
        InterlockConstants.IntakeSlackDeg, 0, 20, "InterlockConstants.IntakeSlackDeg");

    private MechanismInterlock() {}

    /** The furthest the intake may pivot toward stowed (degrees) given where the floor is. */
    public double maxIntakeDeg(double floorRot, double stowedDeg) {
        return maxIntakeDeg(enabled.get(), intakeClearBelowDeg.get(), floorFlatBelowRot.get(), floorRot, stowedDeg);
    }

    /** Whether the floor may rise given where the intake is (degrees; 0 = deployed, larger = toward stowed). */
    public boolean floorMayRaise(double intakeDeg) {
        return floorMayRaise(enabled.get(), intakeClearBelowDeg.get(), intakeSlackDeg.get(), intakeDeg);
    }

    /** True if the floor is flat enough for the intake to go all the way up. */
    public boolean floorIsFlat(double floorRot) {
        return floorRot <= floorFlatBelowRot.get();
    }

    // ---- the decisions, as plain functions so they can be tested without any hardware

    public static double maxIntakeDeg(boolean enabled, double clearBelowDeg, double flatBelowRot, double floorRot, double stowedDeg) {
        if (!enabled || floorRot <= flatBelowRot) return stowedDeg;
        return Math.min(clearBelowDeg, stowedDeg);
    }

    public static boolean floorMayRaise(boolean enabled, double clearBelowDeg, double slackDeg, double intakeDeg) {
        return !enabled || intakeDeg <= clearBelowDeg - slackDeg;
    }
}
