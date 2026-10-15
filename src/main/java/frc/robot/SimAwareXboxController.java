// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * CommandXboxController that fixes the axis layout in the WPILib simulator.
 *
 * On the real robot (and in any non-simulation run) this behaves exactly like CommandXboxController. In simulation the
 * controller's raw axes come through in a different order on this Mac, so the six axes are read from the SIM_* indices
 * below instead, and the triggers are rescaled from -1..1 to 0..1 if they rest at -1.
 *
 * Edit the SIM_* constants if the layout in the sim GUI changes.
 */
public class SimAwareXboxController extends CommandXboxController {
    // ---- Sim-only raw axis indices (what the sim GUI shows for the controller on the Mac) ----
    public static final int SIM_LEFT_X = 0;
    public static final int SIM_LEFT_Y = 1;
    public static final int SIM_RIGHT_X = 2;
    public static final int SIM_RIGHT_Y = 3;
    public static final int SIM_LEFT_TRIGGER = 5;  // swapped: pressing the left trigger showed up on sim axis 5
    public static final int SIM_RIGHT_TRIGGER = 4; // swapped: pressing the right trigger showed up on sim axis 4

    // ---- Sim-only button numbers (1-based, as the sim GUI's Joysticks window numbers them) ----
    // Measured on the Xbox Wireless Controller over Bluetooth on macOS, which gives the sim 27 raw buttons in a different order
    // than a real Xbox controller on the robot. Each standard button is looked up here in simulation. To re-measure (or for a
    // different controller), watch /Tuning/Controller/PressedRawButtons or the terminal while pressing a physical button.
    public static final int SIM_BUTTON_A = 1;
    public static final int SIM_BUTTON_B = 2;
    public static final int SIM_BUTTON_X = 4;
    public static final int SIM_BUTTON_Y = 5;
    public static final int SIM_BUTTON_LEFT_BUMPER = 7;
    public static final int SIM_BUTTON_RIGHT_BUMPER = 8;
    public static final int SIM_BUTTON_BACK = 11;   // the small left button (View)
    public static final int SIM_BUTTON_START = 12;  // the small right button (Menu)
    public static final int SIM_BUTTON_LEFT_STICK = 14;
    public static final int SIM_BUTTON_RIGHT_STICK = 15;
    // (raw button 17 reads as permanently pressed on this controller, and 24..27 mirror the D-pad; neither is used.)

    /** Set false if the sim triggers already read 0..1 (rest at 0). */
    public static final boolean SIM_TRIGGERS_REST_AT_NEGATIVE_ONE = true;

    private static final double kDefaultTriggerThreshold = 0.5;

    private final boolean simulation = RobotBase.isSimulation();

    public SimAwareXboxController(int port) {
        super(port);
    }

    /** Standard Xbox axis index in, sim index out; triggers rescaled to 0..1. Real robot: unchanged. */
    @Override
    public double getRawAxis(int axis) {
        if (!simulation) {
            return super.getRawAxis(axis);
        }
        if (axis == Axis.kLeftX.value) return super.getRawAxis(SIM_LEFT_X);
        if (axis == Axis.kLeftY.value) return super.getRawAxis(SIM_LEFT_Y);
        if (axis == Axis.kRightX.value) return super.getRawAxis(SIM_RIGHT_X);
        if (axis == Axis.kRightY.value) return super.getRawAxis(SIM_RIGHT_Y);
        if (axis == Axis.kLeftTrigger.value) return simTrigger(SIM_LEFT_TRIGGER);
        if (axis == Axis.kRightTrigger.value) return simTrigger(SIM_RIGHT_TRIGGER);
        return super.getRawAxis(axis);
    }

    private double simTrigger(int simIndex) {
        if (!getHID().isConnected()) {
            return 0.0; // nothing plugged in reads 0; don't let the rescale turn that into 0.5
        }
        double raw = super.getRawAxis(simIndex);
        return SIM_TRIGGERS_REST_AT_NEGATIVE_ONE ? MathUtil.clamp((raw + 1.0) / 2.0, 0.0, 1.0) : raw;
    }

    @Override
    public double getLeftX() {
        return simulation ? getRawAxis(Axis.kLeftX.value) : super.getLeftX();
    }

    @Override
    public double getLeftY() {
        return simulation ? getRawAxis(Axis.kLeftY.value) : super.getLeftY();
    }

    @Override
    public double getRightX() {
        return simulation ? getRawAxis(Axis.kRightX.value) : super.getRightX();
    }

    @Override
    public double getRightY() {
        return simulation ? getRawAxis(Axis.kRightY.value) : super.getRightY();
    }

    @Override
    public double getLeftTriggerAxis() {
        return simulation ? getRawAxis(Axis.kLeftTrigger.value) : super.getLeftTriggerAxis();
    }

    @Override
    public double getRightTriggerAxis() {
        return simulation ? getRawAxis(Axis.kRightTrigger.value) : super.getRightTriggerAxis();
    }

    // Buttons: in simulation each one is looked up in the SIM_BUTTON_* table above (the base class would use the real numbers).

    @Override
    public Trigger a(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_A, loop) : super.a(loop);
    }

    @Override
    public Trigger b(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_B, loop) : super.b(loop);
    }

    @Override
    public Trigger x(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_X, loop) : super.x(loop);
    }

    @Override
    public Trigger y(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_Y, loop) : super.y(loop);
    }

    @Override
    public Trigger leftBumper(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_LEFT_BUMPER, loop) : super.leftBumper(loop);
    }

    @Override
    public Trigger rightBumper(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_RIGHT_BUMPER, loop) : super.rightBumper(loop);
    }

    @Override
    public Trigger back(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_BACK, loop) : super.back(loop);
    }

    @Override
    public Trigger start(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_START, loop) : super.start(loop);
    }

    @Override
    public Trigger leftStick(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_LEFT_STICK, loop) : super.leftStick(loop);
    }

    @Override
    public Trigger rightStick(EventLoop loop) {
        return simulation ? button(SIM_BUTTON_RIGHT_STICK, loop) : super.rightStick(loop);
    }

    // The base class builds these from the raw HID axis, bypassing the getters above, so they are overridden too.

    @Override
    public Trigger leftTrigger(double threshold, EventLoop loop) {
        return simulation ? new Trigger(loop, () -> getLeftTriggerAxis() > threshold) : super.leftTrigger(threshold, loop);
    }

    @Override
    public Trigger leftTrigger(double threshold) {
        return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger leftTrigger() {
        return leftTrigger(kDefaultTriggerThreshold);
    }

    @Override
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return simulation ? new Trigger(loop, () -> getRightTriggerAxis() > threshold) : super.rightTrigger(threshold, loop);
    }

    @Override
    public Trigger rightTrigger(double threshold) {
        return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    @Override
    public Trigger rightTrigger() {
        return rightTrigger(kDefaultTriggerThreshold);
    }
}
