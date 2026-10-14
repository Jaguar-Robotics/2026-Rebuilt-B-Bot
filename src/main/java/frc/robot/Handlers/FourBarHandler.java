// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Handlers;

import static edu.wpi.first.units.Units.Rotations;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Set;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tuning.PerfMonitor;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.tuning.TuningTelemetry;
import frc.robot.util.MechanismInterlock;

public class FourBarHandler extends SubsystemBase implements StateSubsystem {

    public enum FourBarState implements State {
        FLAT,   // floor flat: the default, boot, intaking and stowed
        RAISED, // floor tilted up so balls roll to the intake: only while outtaking
        HOMING // re-home in progress; also left set if a re-home fails to find the hard stop
  }

  private static FourBarHandler instance;
  private final FourBarSubsystem fourBarSubsystem = new FourBarSubsystem();

  private FourBarState desiredState = FourBarState.FLAT;
  private FourBarState currentState = FourBarState.FLAT;
  /** Creates a new FourBarHandler. */
  private FourBarHandler() {
    // FLAT is the default state, so setDesiredState(FLAT) would be ignored as "already there". Command it once now so the motors
    // actively hold the floor just above the down stop from boot (also what makes the hopper weighing work).
    fourBarSubsystem.setTarget(Rotations.of(fourBarSubsystem.getFlatRot()));
  }

  public static FourBarHandler getInstance(){
    if (instance == null){
      instance = new FourBarHandler();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof FourBarState fourBarState && desiredState != fourBarState && fourBarState != FourBarState.HOMING) {
        desiredState = fourBarState;

        handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    switch (desiredState) {
        case FLAT:
            fourBarSubsystem.setTarget(Rotations.of(fourBarSubsystem.getFlatRot()));
        break;
        case RAISED:
            fourBarSubsystem.setTarget(Rotations.of(effectiveTarget(FourBarState.RAISED)));
        break;
        case HOMING:
            // only entered through rehome()
        break;
    }
  }

  /**
   * Where the floor should actually go for a state. A raised floor would hit an intake that is not clear of it, so RAISED waits (the
   * floor stays flat) until the intake has come down out of the way.
   */
  private double effectiveTarget(FourBarState state) {
    double wanted = fourBarSubsystem.clampToLimits(angleOf(state));
    if (state == FourBarState.RAISED) {
        boolean clear = MechanismInterlock.get().floorMayRaise(IntakePivotHandler.getInstance().getPositionDeg());
        TuningTelemetry.put("Interlock/FloorHeldBackByIntake", !clear);
        if (!clear) return fourBarSubsystem.clampToLimits(fourBarSubsystem.getFlatRot());
    }
    return wanted;
  }

  /** Marks the current state once the mechanism has actually arrived at the desired one. */
  @Override
  public void update() {
    if (currentState != desiredState && desiredState != FourBarState.HOMING
        && fourBarSubsystem.atSetpoint()
        && Math.abs(fourBarSubsystem.getTarget().in(Rotations) - fourBarSubsystem.clampToLimits(angleOf(desiredState))) < 1e-6) {
        currentState = desiredState;
    }
  }

  private double angleOf(FourBarState state) {
    switch (state) {
        case RAISED: return fourBarSubsystem.getRaisedRot();
        default: return fourBarSubsystem.getFlatRot();
    }
  }

  /**
   * Drives into the stow hard stop and re-zeroes. State is HOMING while it runs, then FLAT if the stop
   * was found. If it was not (timeout/interrupt) the state stays HOMING until a new state is requested.
   */
  public Command rehome() {
    return Commands.runOnce(() -> {
            desiredState = FourBarState.HOMING;
            currentState = FourBarState.HOMING;
        })
        .andThen(fourBarSubsystem.rehome())
        .finallyDo(() -> {
            if (Math.abs(fourBarSubsystem.getTarget().in(Rotations) - fourBarSubsystem.getFlatRot()) < 1e-6) {
                desiredState = FourBarState.FLAT; // update() flips currentState once it is flat
            }
        })
        .withName("FourBarHandler rehome");
  }

  public FourBarState getCurrentState() {
      return currentState;
  }

  public FourBarState getDesiredState() {
      return desiredState;
  }

  /** true when the floor is flat and not moving, so its holding current reflects only the weight on it. */
  public boolean isFlatAndStill() {
      return currentState == FourBarState.FLAT && fourBarSubsystem.atSetpoint();
  }

  public double getHoldCurrentAmps() {
      return fourBarSubsystem.getHoldCurrentAmps();
  }

  public double getAngleRot() {
      return fourBarSubsystem.getPosition().in(Rotations);
  }

  /** Simulation only: sets the simulated moving mass (more fuel in the hopper = heavier load). */
  public void setSimMassKg(double massKg) {
    fourBarSubsystem.setSimMassKg(massKg);
  }

  /** Test hook: simulated moving mass (empty vs loaded). */
  void setSimMassKgForTesting(double massKg) {
    fourBarSubsystem.setSimMassKg(massKg);
  }

  /** Test hook: releases the singleton and its devices so a fresh one can be built. */
  void closeForTesting() {
    fourBarSubsystem.stop();
    edu.wpi.first.wpilibj.Timer.delay(0.1);
    fourBarSubsystem.close();
    CommandScheduler.getInstance().unregisterSubsystem(this);
    instance = null;
  }

  /**
   * Square-wave test for tuning: alternates flat and raised, waiting for dwellSeconds at each. Run it from Elastic, watch
   * /Tuning/FourBar/Live and /Tuning/FourBar/Result, change a gain, and watch the result change. Cancel it to stop.
   */
  public Command squareWave(DoubleSupplier dwellSeconds) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> setDesiredState(FourBarState.RAISED)),
            Commands.defer(() -> Commands.waitSeconds(dwellSeconds.getAsDouble()), Set.of()),
            Commands.runOnce(() -> setDesiredState(FourBarState.FLAT)),
            Commands.defer(() -> Commands.waitSeconds(dwellSeconds.getAsDouble()), Set.of()))
        .finallyDo(() -> setDesiredState(FourBarState.FLAT))
        .withName("FourBar square wave");
  }

  /** If a setpoint is edited in Elastic while the floor is in that state, move to the new position right away. */
  private void followTunedSetpoints() {
    if (desiredState == FourBarState.HOMING) return;
    double want = effectiveTarget(desiredState);
    if (Math.abs(fourBarSubsystem.getTarget().in(Rotations) - want) > 1e-9) {
        fourBarSubsystem.setTarget(Rotations.of(want));
    }
  }

  @Override
  public void periodic() {
    long perfStart = PerfMonitor.now();
    periodicInner();
    PerfMonitor.lap("Handler.FourBar", perfStart);
  }

  private void periodicInner() {
    followTunedSetpoints();
    update();
    DogLog.log("FourBar/DesiredState", desiredState);
    DogLog.log("FourBar/CurrentState", currentState);
  }
}
