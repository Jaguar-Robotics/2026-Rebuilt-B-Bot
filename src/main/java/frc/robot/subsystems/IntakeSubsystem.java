// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  double targetSpeed = 0.0;
  private final TalonFX intakeLeaderMotor = new TalonFX(Constants.IntakeConstants.IntakeLeaderID, Constants.UpperCanBus);
  private final TalonFX intakeFollowerMotor = new TalonFX(Constants.IntakeConstants.IntakefollowerID, Constants.UpperCanBus);
  private final TalonFX intakeReversedMotor = new TalonFX(Constants.IntakeConstants.ReversedIntakeID, Constants.UpperCanBus);
  private final TalonFX intakeReversedMotorTwo = new TalonFX(Constants.IntakeConstants.ReversedIntakeID2, Constants.UpperCanBus);

  //intakeFollowerMotor.setControl(new Follower(Constants.IntakeConstants.IntakeMotorID, MotorAlignmentValue.Opposed));

  public IntakeSubsystem() {
    intakeFollowerMotor.setControl(new Follower(Constants.IntakeConstants.IntakeLeaderID, MotorAlignmentValue.Aligned));
  }

  public void set(double speed){
    exampleMotor.set(speed);
    targetSpeed = speed;
  }

  public void stop (){
    exampleMotor.set(0.0);
    DogLog.log("ExampleMotor/Speed", 0.0);
    targetSpeed = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("ExampleMotor/Speed", targetSpeed); 
    //this will log whatever targetSpeed is under robot/ExampleMotor/speed (each "/"" is a new folder in logs)
    //Check these in AdvnatageScope or Elastic (Elastic for on the field feedback, AdvantageScope for debugging) (Wpi buttion -> start tool -> whatever tool u want)
  }
}
