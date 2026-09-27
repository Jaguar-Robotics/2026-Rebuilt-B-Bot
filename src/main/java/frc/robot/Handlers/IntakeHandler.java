// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class IntakeHandler extends SubsystemBase implements StateSubsystem {

    public enum IntakeState implements State {
        FAST,
        REVFAST,
        SLOW,
        REVSLOW,
        OFF
  }

  private static IntakeHandler instance;
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final KickerSubsystem kickerSubsystem = new KickerSubsystem();

  private IntakeState desiredState = IntakeState.OFF;
  private IntakeState currentState = IntakeState.OFF;
  /** Creates a new IntakeHandler. */
  private IntakeHandler() {}

  public static IntakeHandler getInstance(){
    if (instance == null){
      instance = new IntakeHandler();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof IntakeState IntakeState && desiredState != IntakeState) {
        desiredState = IntakeState;
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
            case FAST:
                intakeSubsystem.set(Constants.IntakeConstants.Fast);
                kickerSubsystem.set(Constants.KickerConstants.Fast);
            break;
            case REVFAST:
                intakeSubsystem.set(Constants.IntakeConstants.RevFast);
                kickerSubsystem.set(Constants.KickerConstants.RevFast);
            break;
            case SLOW:
                intakeSubsystem.set(Constants.IntakeConstants.SlowSpeed);
                kickerSubsystem.set(Constants.KickerConstants.SlowSpeed);
            break;
            case REVSLOW:
                intakeSubsystem.set(Constants.IntakeConstants.RevSlowSpeed);
                kickerSubsystem.set(Constants.KickerConstants.RevSlowSpeed);
            case OFF:
                intakeSubsystem.stop();
                kickerSubsystem.stop();
            break;
        }
        currentState = desiredState;
    }
  }

    public IntakeState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
