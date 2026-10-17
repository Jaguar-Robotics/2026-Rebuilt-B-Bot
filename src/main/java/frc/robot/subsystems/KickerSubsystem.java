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

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  double targetSpeed = 0.0;
  private final TalonFX kickerLeaderMotor = new TalonFX(Constants.KickerConstants.KickerLeaderID, Constants.DriveBus);
  private final TalonFX KickerFollowerMotor = new TalonFX(Constants.KickerConstants.KickerFollowerID, Constants.DriveBus);

  private final TalonFXConfigurator kickerLeadeConfigurator = kickerLeaderMotor.getConfigurator();
  private final TalonFXConfigurator kickerfollowerConfigurator = KickerFollowerMotor.getConfigurator();

  CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();


  public KickerSubsystem() {
    KickerFollowerMotor.setControl(new Follower(Constants.KickerConstants.KickerLeaderID, MotorAlignmentValue.Opposed));

    // flip KickerConstants.LeaderInverted if the kicker spins the wrong way for a positive (feed) speed
    kickerLeadeConfigurator.apply(new com.ctre.phoenix6.configs.MotorOutputConfigs().withInverted(
        Constants.KickerConstants.LeaderInverted
            ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
            : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive));

    limitConfigs.SupplyCurrentLimit = 20;
    limitConfigs.SupplyCurrentLimitEnable = true;

    kickerLeadeConfigurator.apply(limitConfigs);
    kickerfollowerConfigurator.apply(limitConfigs);
  }

  public void set(double speed){
    kickerLeaderMotor.set(speed);
    targetSpeed = speed;
  }

  public void stop (){
    kickerLeaderMotor.set(0.0);
    DogLog.log("ExampleMotor/Speed", 0.0);
    targetSpeed = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DogLog.log("ExampleMotor/Speed", targetSpeed); 
  }
}
