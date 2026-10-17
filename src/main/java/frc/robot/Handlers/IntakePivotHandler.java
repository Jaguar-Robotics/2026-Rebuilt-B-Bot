// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Handlers;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningMode;
import frc.robot.tuning.TuningTelemetry;
import frc.robot.util.MechanismInterlock;
import java.util.Set;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class IntakePivotHandler extends SubsystemBase implements StateSubsystem {

    public enum IntakePivotState implements State {
        STOWED,
        FEED, // slightly deployed, for feeding the outpost
        ENGADGED
  }

  private static IntakePivotHandler instance;
  private final IntakePivotSubsystem IntakePivotSubsystem = new IntakePivotSubsystem();

  private IntakePivotState desiredState = IntakePivotState.STOWED;
  private IntakePivotState currentState = IntakePivotState.STOWED;
  /** Creates a new IntakePivotHandler. */
  private IntakePivotHandler() {}

  public static IntakePivotHandler getInstance(){
    if (instance == null){
      instance = new IntakePivotHandler();
    }
    return instance;
  }

  // ---- manual jog (D-pad left / right): moves the intake a little at a time while the button is held, in case of a jam
  /** Jog speed in degrees per second (scaled down in safe mode). */
  private final TunableNumber manualRate = Tunables.number("Manual/IntakeDegPerSec", 40.0, 5.0, 90.0, null);
  /** The jog target never runs further than this ahead of where the intake really is, so a jam cannot wind the target up. */
  private static final double kManualMaxLeadDeg = 6.0;
  private boolean manual = false;

  /**
   * Manual override: nudges the intake toward stowed (direction +1) or deployed (-1) for one 20 ms loop. It cannot hurt the robot: the target is
   * clamped to the soft limits, it only creeps a short way ahead of the real position, the intake cannot be jogged up into a raised floor
   * (the same interlock as everything else), and safe mode slows it. When the button is released the intake holds where it is.
   */
  public void manualJog(int direction, double dtSeconds) {
    double pos = getPositionDeg();
    double target = IntakePivotSubsystem.getTarget().in(Degrees);
    double next = target + Math.signum(direction) * TuningMode.scale(manualRate.get()) * dtSeconds;
    next = Math.max(pos - kManualMaxLeadDeg, Math.min(pos + kManualMaxLeadDeg, next));
    manual = true;
    applyTarget(Degrees.of(next)); // soft limits and the floor interlock are enforced in here
    TuningTelemetry.put("IntakePivot/Status", "Manual jog");
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof IntakePivotState IntakePivotState && (manual || desiredState != IntakePivotState)) {
        manual = false;
        desiredState = IntakePivotState;

        handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    update();
  }

      @Override
    public void update() {
      if((currentState != desiredState)){
        switch (desiredState) {
            case STOWED:
                applyTarget(IntakePivotSubsystem.getStowedSetpoint());
            break;
            case FEED:
                applyTarget(IntakePivotSubsystem.getFeedSetpoint());
            break;
            case ENGADGED:
                applyTarget(IntakePivotSubsystem.getDeployedSetpoint());
            break;
        }   
        currentState = desiredState;
    }
  }

    public IntakePivotState getCurrentState() {
      return currentState;
  }

  /**
   * Square-wave test for tuning: alternates deployed and stowed, waiting dwellSeconds at each. Run it from Elastic and
   * watch /Tuning/IntakePivot/Live and /Tuning/IntakePivot/Result. Cancel it to stop (it ends stowed).
   */
  public Command squareWave(DoubleSupplier dwellSeconds) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> setDesiredState(IntakePivotState.ENGADGED)),
            Commands.defer(() -> Commands.waitSeconds(dwellSeconds.getAsDouble()), Set.of()),
            Commands.runOnce(() -> setDesiredState(IntakePivotState.STOWED)),
            Commands.defer(() -> Commands.waitSeconds(dwellSeconds.getAsDouble()), Set.of()))
        .finallyDo(() -> setDesiredState(IntakePivotState.STOWED))
        .withName("IntakePivot square wave");
  }

  /** Declares the intake to be physically stowed right now (re-seeds the motor position) and holds it there. */
  public void rezeroToStowed() {
    IntakePivotSubsystem.rezeroToStowed();
    currentState = IntakePivotState.STOWED;
    desiredState = IntakePivotState.STOWED;
  }

  /** The slightly-raised outpost feeding angle (degrees). */
  public double getFeedDeg() {
    return IntakePivotSubsystem.getFeedSetpoint().in(Degrees);
  }

  public boolean isPositionLost() {
    return IntakePivotSubsystem.isPositionLost();
  }

  public void markPositionLostForTesting() {
    IntakePivotSubsystem.markPositionLostForTesting();
  }

  /** The intake's actual angle in degrees (0 = deployed). */
  public double getPositionDeg() {
    return IntakePivotSubsystem.getPosition().in(Degrees);
  }

  /**
   * Sends a target to the intake, but never further toward stowed than the floor allows: the intake cannot pivot up into the hopper
   * floor, so while the floor is raised the intake waits at the clear angle, and goes the rest of the way once the floor is flat.
   */
  private void applyTarget(Angle wanted) {
    Angle clamped = IntakePivotSubsystem.clampToLimits(wanted);
    double limitDeg = MechanismInterlock.get().maxIntakeDeg(FourBarHandler.getInstance().getAngleRot(), IntakePivotSubsystem.getStowedSetpoint().in(Degrees));
    Angle allowed = Degrees.of(Math.min(clamped.in(Degrees), limitDeg));
    TuningTelemetry.put("Interlock/IntakeHeldBackByFloor", allowed.in(Degrees) < clamped.in(Degrees) - 1e-9);
    if (Math.abs(allowed.in(Degrees) - IntakePivotSubsystem.getTarget().in(Degrees)) > 1e-9) {
      IntakePivotSubsystem.goToSetpoint(allowed);
    }
  }

  /** If a setpoint is edited in Elastic while the intake is in that state, move to the new angle right away. */
  private void followTunedSetpoints() {
    if (manual) return; // the driver is holding the intake where they put it
    Angle want;
    switch (desiredState) {
        case FEED: want = IntakePivotSubsystem.getFeedSetpoint(); break;
        case ENGADGED: want = IntakePivotSubsystem.getDeployedSetpoint(); break;
        default: want = IntakePivotSubsystem.getStowedSetpoint(); break;
    }
    if (currentState == desiredState) applyTarget(want); // also re-checks the floor every loop, releasing the intake once it is flat
  }

  @Override
  public void periodic() {
    followTunedSetpoints();
  }
}
