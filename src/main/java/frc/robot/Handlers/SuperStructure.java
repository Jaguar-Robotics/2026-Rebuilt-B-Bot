package frc.robot.Handlers;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Handlers.*;

public class SuperStructure extends SubsystemBase {

  // Define your states in all caps
  public enum SuperStructureState {
    INTAKE,
    JERK,
    POOP,
    STOWED
  }

  private static SuperStructure instance;
  
  //private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final IntakeHandler intakeHandler = IntakeHandler.getInstance();
  private final IntakePivotHandler intakePivotHandler = IntakePivotHandler.getInstance();

  private SuperStructureState desiredState = SuperStructureState.STOWED;
  private SuperStructureState currentState = SuperStructureState.STOWED;

  

  public static SuperStructure getInstance(){
    if (instance == null) {
      instance = new SuperStructure();
    }
    return instance;
  }


  private SuperStructure() {}

  /** Call this from commands or joystick logic to set the next goal. */
  public void setDesiredState(SuperStructureState newState) {
    System.out.print("SetDesiredState:" + newState);
    if (desiredState != newState) {
      desiredState = newState;
      handleStateTransition();
    }
  }

  /** Handles transitions between states (2910-style logic). */
  private void handleStateTransition() {
    switch (desiredState) {
        case INTAKE:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.FAST);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.ENGADGED);
        break;
        case JERK:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.REVFAST);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.ENGADGED);
        break;
        case POOP:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.REVFAST);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.ENGADGED);
        break;
        case STOWED:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.STOWED);
        break;
    }
    currentState = desiredState;
  }

  @Override
  public void periodic() {    
  }

  public SuperStructureState getCurrentState() {
    return currentState;
  }

  public SuperStructureState getDesiredState() {
    return desiredState;
  }
}