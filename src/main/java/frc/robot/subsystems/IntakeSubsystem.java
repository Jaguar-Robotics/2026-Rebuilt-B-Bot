// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  double targetSpeed = 0.0;
  private final TalonFX intakeLeaderMotor = new TalonFX(Constants.IntakeConstants.IntakeLeaderID, Constants.DriveBus);
  private final TalonFX intakeFollowerMotor = new TalonFX(Constants.IntakeConstants.IntakefollowerID, Constants.DriveBus);
  private final TalonFX intakeReversedMotor = new TalonFX(Constants.IntakeConstants.ReversedIntakeID, Constants.DriveBus);
  private final TalonFX intakeReversedMotorTwo = new TalonFX(Constants.IntakeConstants.ReversedIntakeID2, Constants.DriveBus);

  private final TalonFXConfigurator intakeLeaderConfigurator = intakeLeaderMotor.getConfigurator();
  private final TalonFXConfigurator intakeFollowerMotorConfigurator = intakeFollowerMotor.getConfigurator();
  private final TalonFXConfigurator intakeReversedMotorConfigurator = intakeReversedMotor.getConfigurator();
  private final TalonFXConfigurator intakeReversedMotorTwoConfigurator = intakeReversedMotorTwo.getConfigurator();

  CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();


  public IntakeSubsystem() {
    intakeFollowerMotor.setControl(new Follower(Constants.IntakeConstants.IntakeLeaderID, MotorAlignmentValue.Aligned));
    intakeReversedMotor.setControl(new Follower(Constants.IntakeConstants.IntakeLeaderID, MotorAlignmentValue.Opposed));
    intakeReversedMotorTwo.setControl(new Follower(Constants.IntakeConstants.IntakeLeaderID, MotorAlignmentValue.Opposed));

    limitConfigs.SupplyCurrentLimit = 60;
    limitConfigs.SupplyCurrentLimitEnable = true;

    intakeLeaderConfigurator.apply(limitConfigs);
    intakeFollowerMotorConfigurator.apply(limitConfigs);
    intakeReversedMotorConfigurator.apply(limitConfigs);
    intakeReversedMotorTwoConfigurator.apply(limitConfigs);
  }

  public void set(double speed){
    intakeLeaderMotor.set(speed);
    targetSpeed = speed;
  }

  public void stop (){
    intakeLeaderMotor.set(0.0);
    DogLog.log("ExampleMotor/Speed", 0.0);
    targetSpeed = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("ExampleMotor/Speed", targetSpeed); 
  }
}
