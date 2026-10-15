// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningMode;

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

  // live-tunable roller speeds, duty cycle -1..1 (/Tuning/Rollers/...)
  private final TunableNumber intakeFast = speed("Rollers/Intake/Fast", Constants.IntakeConstants.Fast, "IntakeConstants.Fast");
  private final TunableNumber intakeSlow = speed("Rollers/Intake/Slow", Constants.IntakeConstants.SlowSpeed, "IntakeConstants.SlowSpeed");
  private final TunableNumber intakeRevFast = speed("Rollers/Intake/RevFast", Constants.IntakeConstants.RevFast, "IntakeConstants.RevFast");
  private final TunableNumber intakeRevSlow = speed("Rollers/Intake/RevSlow", Constants.IntakeConstants.RevSlowSpeed, "IntakeConstants.RevSlowSpeed");
  private final TunableNumber kickerFast = speed("Rollers/Kicker/Fast", Constants.KickerConstants.Fast, "KickerConstants.Fast");
  private final TunableNumber kickerSlow = speed("Rollers/Kicker/Slow", Constants.KickerConstants.SlowSpeed, "KickerConstants.SlowSpeed");
  private final TunableNumber kickerRevFast = speed("Rollers/Kicker/RevFast", Constants.KickerConstants.RevFast, "KickerConstants.RevFast");
  private final TunableNumber kickerRevSlow = speed("Rollers/Kicker/RevSlow", Constants.KickerConstants.RevSlowSpeed, "KickerConstants.RevSlowSpeed");

  private TunableNumber speed(String key, double defaultValue, String javaConstant) {
    return Tunables.number(key, defaultValue, -1.0, 1.0, javaConstant, v -> applyState(currentState));
  }

  private IntakeState desiredState = IntakeState.OFF;
  private IntakeState currentState = IntakeState.OFF;
  /** Creates a new IntakeHandler. */
  private IntakeHandler() {
    TuningMode.onChange(() -> applyState(currentState)); // safe mode scales the roller speeds
  }

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
        applyState(desiredState);
        currentState = desiredState;
    }
  }

  /** Sends the roller speeds for a state to the motors (also re-sent when a speed is tuned while in that state). */
  private void applyState(IntakeState state) {
    switch (state) {
        case FAST:
            intakeSubsystem.set(TuningMode.scale(intakeFast.get()));
            kickerSubsystem.set(TuningMode.scale(kickerFast.get()));
        break;
        case REVFAST:
            intakeSubsystem.set(TuningMode.scale(intakeRevFast.get()));
            kickerSubsystem.set(TuningMode.scale(kickerRevFast.get()));
        break;
        case SLOW:
            intakeSubsystem.set(TuningMode.scale(intakeSlow.get()));
            kickerSubsystem.set(TuningMode.scale(kickerSlow.get()));
        break;
        case REVSLOW:
            intakeSubsystem.set(TuningMode.scale(intakeRevSlow.get()));
            kickerSubsystem.set(TuningMode.scale(kickerRevSlow.get()));
        break;
        case OFF:
            intakeSubsystem.stop();
            kickerSubsystem.stop();
        break;
    }
  }

    public IntakeState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
