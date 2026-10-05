// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Handlers;

import static edu.wpi.first.units.Units.Rotations;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.FourBarSubsystem;

public class FourBarHandler extends SubsystemBase implements StateSubsystem {

    public enum FourBarState implements State {
        STOWED,
        INTAKE,
        HOMING // re-home in progress; also left set if a re-home fails to find the hard stop
  }

  private static FourBarHandler instance;
  private final FourBarSubsystem fourBarSubsystem = new FourBarSubsystem();

  private FourBarState desiredState = FourBarState.STOWED;
  private FourBarState currentState = FourBarState.STOWED;
  /** Creates a new FourBarHandler. */
  private FourBarHandler() {}

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
        case STOWED:
            fourBarSubsystem.setTarget(Rotations.of(Constants.FourBarConstants.StowRot));
        break;
        case INTAKE:
            fourBarSubsystem.setTarget(Rotations.of(Constants.FourBarConstants.IntakeRot));
        break;
        case HOMING:
            // only entered through rehome()
        break;
    }
  }

  /** Marks the current state once the mechanism has actually arrived at the desired one. */
  @Override
  public void update() {
    if (currentState != desiredState && desiredState != FourBarState.HOMING
        && fourBarSubsystem.atSetpoint()
        && Math.abs(fourBarSubsystem.getTarget().in(Rotations) - angleOf(desiredState)) < 1e-6) {
        currentState = desiredState;
    }
  }

  private static double angleOf(FourBarState state) {
    switch (state) {
        case INTAKE: return Constants.FourBarConstants.IntakeRot;
        default: return Constants.FourBarConstants.StowRot;
    }
  }

  /**
   * Drives into the stow hard stop and re-zeroes. State is HOMING while it runs, then STOWED if the stop
   * was found. If it was not (timeout/interrupt) the state stays HOMING until a new state is requested.
   */
  public Command rehome() {
    return Commands.runOnce(() -> {
            desiredState = FourBarState.HOMING;
            currentState = FourBarState.HOMING;
        })
        .andThen(fourBarSubsystem.rehome())
        .finallyDo(() -> {
            if (Math.abs(fourBarSubsystem.getTarget().in(Rotations) - Constants.FourBarConstants.StowRot) < 1e-6) {
                desiredState = FourBarState.STOWED; // update() flips currentState once it is at stow
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

  public double getAngleRot() {
      return fourBarSubsystem.getPosition().in(Rotations);
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

  @Override
  public void periodic() {
    update();
    DogLog.log("FourBar/DesiredState", desiredState);
    DogLog.log("FourBar/CurrentState", currentState);
  }
}
