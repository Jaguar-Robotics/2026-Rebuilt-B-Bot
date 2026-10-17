package frc.robot.Handlers;

import static edu.wpi.first.units.Units.Degrees;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningTelemetry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.tuning.PerfMonitor;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Handlers.*;

public class SuperStructure extends SubsystemBase {

  // Define your states in all caps
  public enum SuperStructureState {
    INTAKE,
    JERK,
    POOP,        // right trigger: outtake with the rollers only. The floor is NOT touched (Y toggles it)
    FEED,        // outpost feeding: intake slightly deployed, rollers off, floor flat until the pulses have run for a while
    FEED_PULSE,  // outpost feeding: same, with a short slow outtake of the rollers
    IDLE,        // rollers off. The intake pivot STAYS WHERE IT IS (never pulled in against a full hopper) and so does the floor (Y toggles it)
    STOWED       // rollers off, floor flat, intake pulled in (A button). Pulling it in against a full hopper can stall the pivot motor, so only do it when the hopper is empty
  }

  private static SuperStructure instance;
  
  //private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final IntakeHandler intakeHandler = IntakeHandler.getInstance();
  private final IntakePivotHandler intakePivotHandler = IntakePivotHandler.getInstance();
  private final FourBarHandler fourBarHandler = FourBarHandler.getInstance();

  private double lastPeriodicSeconds = Timer.getFPGATimestamp();

  // The floor must not tilt up into a packed hopper or it jams, so it lifts only after the rollers have been running for a while.

  private final edu.wpi.first.networktables.NetworkTableEntry stowStatusEntry =
      edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry("/Tuning/Intake/StowStatus");

  /**
   * Driver button X: tells the intake pivot that it is physically stowed right now (use it if the intake has lost its position),
   * then holds it there. Only call this with the intake really stowed against its stop.
   */
  public void rezeroIntakeToStowed() {
    if (!edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
      // a wrong zero would later drive the intake past its stops, so this is a pit action only
      stowStatusEntry.setString("Re-zero refused: only works while the robot is DISABLED, with the intake stowed");
      return;
    }
    intakePivotHandler.rezeroToStowed();
    transitionTo(SuperStructureState.STOWED);
    stowStatusEntry.setString("Intake position re-zeroed: it is now treated as stowed");
  }

  /** Raises the floor (all the way). Used by the outpost feeding once enough pulses have been done. */
  public void raiseFloor() {
    fourBarHandler.setDesiredState(FourBarHandler.FourBarState.RAISED);
  }

  /** Y button: if the floor is down, raise it all the way; if it is up, lower it. (The floor still waits for the intake to be down before it lifts.) */
  public void toggleFloor() {
    boolean raised = fourBarHandler.getDesiredState() == FourBarHandler.FourBarState.RAISED;
    fourBarHandler.setDesiredState(raised ? FourBarHandler.FourBarState.FLAT : FourBarHandler.FourBarState.RAISED);
  }

  /** Test hook: stows (to get a test robot back to its boot state). */
  public void forceStowIgnoringHopper() {
    transitionTo(SuperStructureState.STOWED);
  }

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
    if (newState == SuperStructureState.STOWED) {
      stowStatusEntry.setString("Stowing the intake");
    }
    transitionTo(newState);
  }

  private void transitionTo(SuperStructureState newState) {
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
            fourBarHandler.setDesiredState(FourBarHandler.FourBarState.FLAT); // floor stays flat while intaking
        break;
        case JERK:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.REVFAST);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.ENGADGED);
        break;
        case POOP:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.REVFAST);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.ENGADGED);
            // Rollers only: the floor is left alone.
        break;
        case FEED:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.FEED);
            // Floor is left as it is: the outpost command raises it after the first few pulses.
        break;
        case FEED_PULSE:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.REVSLOW);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.FEED);
        break;
        case IDLE:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
            // The intake pivot is deliberately NOT commanded: it stays where it is. Pulling it in against a full hopper could stall the motor.
            // The floor is not commanded either: it stays where the driver (Y) left it.
        break;
        case STOWED:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.STOWED);
            fourBarHandler.setDesiredState(FourBarHandler.FourBarState.FLAT);
        break;
    }
    currentState = desiredState;
  }

  @Override
  public void periodic() {
    long perfStart = PerfMonitor.now();
    periodicInner();
    PerfMonitor.lap("Handler.SuperStructure", perfStart);
  }

  /** true if the floor or the intake pivot has lost its zero (motor reboot or a jump in the reading) and needs to be re-zeroed. */
  public boolean isPositionLost() {
    return fourBarHandler.isPositionLost() || intakePivotHandler.isPositionLost();
  }

  private void periodicInner() {
    DogLog.log("SuperStructure/DesiredState", desiredState);
    boolean lost = isPositionLost();
    TuningTelemetry.put("Warnings/PositionLost", lost);
    TuningTelemetry.put("Warnings/PositionLostWhere", !lost ? "none"
        : fourBarHandler.isPositionLost() && intakePivotHandler.isPositionLost() ? "floor AND intake"
        : fourBarHandler.isPositionLost() ? "floor (zero: robot disabled, floor flat, D-pad down)"
        : "intake (zero: robot disabled, intake stowed, X)");

    double now = Timer.getFPGATimestamp();
    double dt = now - lastPeriodicSeconds;
    lastPeriodicSeconds = now;
  }

  public SuperStructureState getCurrentState() {
    return currentState;
  }

  public SuperStructureState getDesiredState() {
    return desiredState;
  }
}