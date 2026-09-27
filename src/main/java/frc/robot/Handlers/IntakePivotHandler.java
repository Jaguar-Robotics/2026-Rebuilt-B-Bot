// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class IntakePivotHandler extends SubsystemBase implements StateSubsystem {

    public enum IntakePivotState implements State {
        STOWED,
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
                IntakePivotSubsystem.goToSetpoint(Constants.IntakeConstants.PivotHighSetpoint);
            break;
            case ENGADGED:
                IntakePivotSubsystem.goToSetpoint(Constants.IntakeConstants.PivotLowSetpoint);
            break;
        }   
        currentState = desiredState;
    }
  }

    public IntakePivotState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
