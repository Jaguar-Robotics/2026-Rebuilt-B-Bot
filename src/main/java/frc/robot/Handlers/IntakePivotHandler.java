// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Handlers;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  @Override
  public void setDesiredState(State state){
        if (state instanceof IntakePivotState IntakePivotState && desiredState != IntakePivotState) {
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
