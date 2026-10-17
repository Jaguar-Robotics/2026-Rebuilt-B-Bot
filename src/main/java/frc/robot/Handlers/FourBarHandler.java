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
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningMode;
import frc.robot.tuning.TuningTelemetry;
import frc.robot.util.MechanismInterlock;

public class FourBarHandler extends SubsystemBase implements StateSubsystem {

    public enum FourBarState implements State {
        FLAT,   // floor flat: the default, boot, intaking and stowed
        RAISED // floor tilted up so balls roll to the intake: only while outtaking
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

  // ---- manual jog (D-pad up / down): moves the floor a little at a time while the button is held, in case of a jam
  /** Jog speed in rotations per second (scaled down in safe mode). */
  private final TunableNumber manualRate = Tunables.number("Manual/FloorRotPerSec", 0.08, 0.01, 0.25, null);
  /** The jog target never runs further than this ahead of where the floor really is, so a jam cannot wind the target up. */
  private static final double kManualMaxLeadRot = 0.012;
  private boolean manual = false;

  /**
   * Manual override: nudges the floor up (direction +1) or down (-1) for one 20 ms loop. It cannot hurt the robot: the target is clamped to the
   * soft limits (which sit inside both hard stops), it only creeps a short way ahead of the real position, the floor cannot be jogged up
   * unless the intake is down and clear of it, and safe mode slows it. When the button is released the floor holds where it is; the next
   * state request takes over again.
   */
  public void manualJog(int direction, double dtSeconds) {
    double pos = getAngleRot();
    double target = fourBarSubsystem.getTarget().in(Rotations);
    double next = target + Math.signum(direction) * TuningMode.scale(manualRate.get()) * dtSeconds;
    next = Math.max(pos - kManualMaxLeadRot, Math.min(pos + kManualMaxLeadRot, next));
    if (!MechanismInterlock.get().floorMayRaise(IntakePivotHandler.getInstance().getPositionDeg())) {
        next = Math.min(next, fourBarSubsystem.getFlatRot()); // the intake is in the way: no lifting
    }
    manual = true;
    fourBarSubsystem.setTarget(Rotations.of(next)); // clamps to the soft limits
    TuningTelemetry.put("FourBar/Status", "Manual jog");
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof FourBarState fourBarState && (manual || desiredState != fourBarState)) {
        manual = false;
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
    if (currentState != desiredState
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
   * Zeroes the floor position at the down stop (see {@link FourBarSubsystem#zeroHere()}). No motion. Only works while the robot is disabled,
   * with the floor resting flat.
   */
  public Command zeroHere() {
    return Commands.runOnce(() -> {
            if (fourBarSubsystem.zeroHere()) {
                desiredState = FourBarState.FLAT;
                currentState = FourBarState.FLAT;
            }
        })
        .ignoringDisable(true)
        .withName("FourBarHandler zeroHere");
  }

  public FourBarState getCurrentState() {
      return currentState;
  }

  public FourBarState getDesiredState() {
      return desiredState;
  }

  public boolean isPositionLost() {
      return fourBarSubsystem.isPositionLost();
  }

  public void markPositionLostForTesting() {
      fourBarSubsystem.markPositionLostForTesting();
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
    if (manual) return; // the driver is holding the floor where they put it
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
