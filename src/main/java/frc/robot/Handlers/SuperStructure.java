package frc.robot.Handlers;

import static edu.wpi.first.units.Units.Degrees;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningTelemetry;
import frc.robot.util.FloorRaiseGate;
import frc.robot.util.HopperLoadEstimator;
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
    POOP,
    FEED,        // outpost feeding: intake slightly deployed, rollers off, floor flat until the outtake has made room
    FEED_PULSE,  // outpost feeding: same, with a short slow outtake of the rollers (floor raised once there is room)
    IDLE,        // rollers off, floor flat, and the intake pivot STAYS WHERE IT IS (it is never pulled in against a full hopper)
    STOWED       // rollers off, floor flat, intake pulled in. Only accepted when the hopper is empty or nearly empty (see requestStow)
  }

  private static SuperStructure instance;
  
  //private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final IntakeHandler intakeHandler = IntakeHandler.getInstance();
  private final IntakePivotHandler intakePivotHandler = IntakePivotHandler.getInstance();
  private final FourBarHandler fourBarHandler = FourBarHandler.getInstance();

  private final HopperLoadEstimator loadEstimator = new HopperLoadEstimator(
      Constants.FourBarConstants.EmptyHoldCurrentAmps,
      Constants.FourBarConstants.FullHoldCurrentAmps,
      Constants.FourBarConstants.LoadFilterSamples);
  private final FloorRaiseGate floorRaiseGate = new FloorRaiseGate(
      Constants.FourBarConstants.RaiseBelowBalls / Constants.FuelConstants.HopperCapacity,
      Constants.FourBarConstants.RaiseFallbackAfterOuttakeSeconds,
      Constants.FourBarConstants.RaiseMaxWaitSeconds);
  private double lastPeriodicSeconds = Timer.getFPGATimestamp();

  // live-tunable (/Tuning/Hopper/...): the floor-weighing calibration and when the floor is allowed to raise
  private final TunableNumber emptyHoldCurrent = Tunables.number("Hopper/EmptyHoldCurrentAmps",
      Constants.FourBarConstants.EmptyHoldCurrentAmps, 0, 40, "FourBarConstants.EmptyHoldCurrentAmps", v -> applyHopperTuning());
  private final TunableNumber fullHoldCurrent = Tunables.number("Hopper/FullHoldCurrentAmps",
      Constants.FourBarConstants.FullHoldCurrentAmps, 0.5, 60, "FourBarConstants.FullHoldCurrentAmps", v -> applyHopperTuning());
  /** How many balls the hopper holds when full (turns the 0..1 load estimate into a ball count). */
  private final TunableNumber capacityBalls = Tunables.number("Hopper/CapacityBalls",
      Constants.FuelConstants.HopperCapacity, 1, 500, "FuelConstants.HopperCapacity", v -> applyHopperTuning());
  /** The floor may lift once the estimated ball count is below this. THE knob for "how full before it lifts". */
  private final TunableNumber liftWhenBelowBalls = Tunables.number("Hopper/LiftWhenBelowBalls",
      Constants.FourBarConstants.RaiseBelowBalls, 0, 500, "FourBarConstants.RaiseBelowBalls", v -> applyHopperTuning());
  private final TunableNumber raiseFallbackSeconds = Tunables.number("Hopper/RaiseFallbackAfterOuttakeSeconds",
      Constants.FourBarConstants.RaiseFallbackAfterOuttakeSeconds, 0, 10, "FourBarConstants.RaiseFallbackAfterOuttakeSeconds", v -> applyHopperTuning());

  private final TunableNumber raiseMaxWaitSeconds = Tunables.number("Hopper/RaiseMaxWaitSeconds",
      Constants.FourBarConstants.RaiseMaxWaitSeconds, 0.5, 30, "FourBarConstants.RaiseMaxWaitSeconds", v -> applyHopperTuning());

  // Stowing the intake is only allowed with the hopper (nearly) empty
  private final edu.wpi.first.networktables.NetworkTableEntry stowStatusEntry =
      edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry("/Tuning/Intake/StowStatus");
  private final frc.robot.tuning.TunableBoolean stowGateEnabled = Tunables.bool("Hopper/StowGateEnabled",
      Constants.IntakeConstants.StowGateEnabled, "IntakeConstants.StowGateEnabled", null);
  private final TunableNumber stowOnlyBelowBalls = Tunables.number("Hopper/StowIntakeOnlyBelowBalls",
      Constants.IntakeConstants.StowOnlyBelowBalls, 0, 500, "IntakeConstants.StowOnlyBelowBalls");
  private int stowRefusals = 0;

  private double estimatedBalls() {
    return loadEstimator.getFraction() * capacityBalls.get();
  }

  /** The intake may be pulled in only if the hopper is (nearly) empty, judged from the floor's weight. With no reading, it is refused. */
  private boolean stowAllowed() {
    if (!stowGateEnabled.get()) return true;
    return loadEstimator.isValid() && estimatedBalls() <= stowOnlyBelowBalls.get();
  }

  /** How many times a stow request was refused because the hopper was not (nearly) empty. */
  public int getStowRefusals() {
    return stowRefusals;
  }

  /** Stows if the hopper is empty enough, otherwise just goes idle (intake stays where it is). No refusal noise. */
  public void stowIfAllowedElseIdle() {
    setDesiredState(stowAllowed() ? SuperStructureState.STOWED : SuperStructureState.IDLE);
  }

  /** Test hook: stows regardless of what is in the hopper (to get a test robot back to its boot state). */
  public void forceStowIgnoringHopper() {
    transitionTo(SuperStructureState.STOWED);
  }

  /** The lift threshold as a 0..1 fraction of a full hopper. */
  private double liftFraction() {
    return Math.min(1.0, liftWhenBelowBalls.get() / capacityBalls.get());
  }

  /** Buttons in Elastic: press with an EMPTY (or FULL) hopper and the floor stationary to calibrate from the live reading. */
  private void calibrateEmptyNow() {
    if (loadEstimator.isValid()) emptyHoldCurrent.set(Math.min(loadEstimator.getFilteredAmps(), fullHoldCurrent.get() - 0.1));
  }

  private void calibrateFullNow() {
    if (loadEstimator.isValid()) fullHoldCurrent.set(Math.max(loadEstimator.getFilteredAmps(), emptyHoldCurrent.get() + 0.1));
  }

  private void applyHopperTuning() {
    loadEstimator.setCalibration(emptyHoldCurrent.get(), fullHoldCurrent.get());
    floorRaiseGate.setThresholds(liftFraction(), raiseFallbackSeconds.get(), raiseMaxWaitSeconds.get());
  }

  private SuperStructureState desiredState = SuperStructureState.STOWED;
  private SuperStructureState currentState = SuperStructureState.STOWED;

  

  public static SuperStructure getInstance(){
    if (instance == null) {
      instance = new SuperStructure();
    }
    return instance;
  }


  private SuperStructure() {
    applyHopperTuning(); // pick up any values that were tuned and saved earlier
    Tunables.action("Hopper/Actions/SetEmptyToCurrentReading", this::calibrateEmptyNow);
    Tunables.action("Hopper/Actions/SetFullToCurrentReading", this::calibrateFullNow);
  }

  /** Call this from commands or joystick logic to set the next goal. */
  public void setDesiredState(SuperStructureState newState) {
    if (newState == SuperStructureState.STOWED && desiredState != SuperStructureState.STOWED && !stowAllowed()) {
      stowRefusals++;
      String message = String.format("Stow refused: the hopper holds about %.0f balls (it must be under %.0f). Intake stays where it is.",
          estimatedBalls(), stowOnlyBelowBalls.get());
      stowStatusEntry.setString(message);
      DogLog.log("SuperStructure/StowRefused", message);
      newState = SuperStructureState.IDLE;
    } else if (newState == SuperStructureState.STOWED) {
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
            // The floor is raised later, in periodic(), once the rollers have made room (see FloorRaiseGate).
        break;
        case FEED:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.FEED);
            // Floor is left as it is: flat until the first pulses have made room, then raised by periodic().
        break;
        case FEED_PULSE:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.REVSLOW);
            intakePivotHandler.setDesiredState(IntakePivotHandler.IntakePivotState.FEED);
        break;
        case IDLE:
            intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
            // The intake pivot is deliberately NOT commanded: it stays where it is. Pulling it in against a full hopper could stall the motor.
            fourBarHandler.setDesiredState(FourBarHandler.FourBarState.FLAT);
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

  private void periodicInner() {
    DogLog.log("SuperStructure/DesiredState", desiredState);

    // Weigh the hopper whenever the floor is flat and still (this also tracks the fuel coming in during intake).
    if (fourBarHandler.isFlatAndStill()) {
      loadEstimator.update(fourBarHandler.getHoldCurrentAmps());
    }

    // Outtake: tilt the floor up only when there is room for it. A few balls = right away; a packed hopper = after the
    // rollers have drained it below RaiseBelowLoadFraction (or after the fallback time if the signal is not usable).
    double now = Timer.getFPGATimestamp();
    double dt = now - lastPeriodicSeconds;
    lastPeriodicSeconds = now;
    boolean outtaking = desiredState == SuperStructureState.POOP || desiredState == SuperStructureState.FEED_PULSE;
    boolean sequenceOver = desiredState == SuperStructureState.STOWED || desiredState == SuperStructureState.IDLE || desiredState == SuperStructureState.INTAKE;
    boolean raiseAllowed = floorRaiseGate.update(
        outtaking, sequenceOver, loadEstimator.isValid(), loadEstimator.getFraction(), dt);
    TuningTelemetry.putLive("Hopper/Live/LoadFraction", loadEstimator.getFraction());
    TuningTelemetry.putLive("Hopper/Live/EstimatedBalls", loadEstimator.getFraction() * capacityBalls.get());
    TuningTelemetry.putLive("Hopper/Live/LiftThresholdBalls", liftWhenBelowBalls.get());
    TuningTelemetry.putLive("Hopper/Live/FilteredHoldCurrentAmps", loadEstimator.getFilteredAmps());
    TuningTelemetry.putLive("Hopper/Live/LiftThresholdHoldCurrentAmps",
        emptyHoldCurrent.get() + liftFraction() * (fullHoldCurrent.get() - emptyHoldCurrent.get()));
    TuningTelemetry.putLive("Hopper/Live/LoadValid", loadEstimator.isValid());
    TuningTelemetry.putLive("Hopper/Live/FloorRaiseAllowed", raiseAllowed);
    TuningTelemetry.putLive("Hopper/Live/IntakeMayStow", stowAllowed());
    TuningTelemetry.putLive("Hopper/Live/HoldCurrentAmps", fourBarHandler.getHoldCurrentAmps());
    DogLog.log("SuperStructure/HopperLoadFraction", loadEstimator.getFraction());
    DogLog.log("SuperStructure/HopperLoadValid", loadEstimator.isValid());
    DogLog.log("SuperStructure/FloorRaiseAllowed", raiseAllowed);
    if (raiseAllowed && (outtaking || desiredState == SuperStructureState.FEED)) {
      fourBarHandler.setDesiredState(FourBarHandler.FourBarState.RAISED);
    }
  }

  public SuperStructureState getCurrentState() {
    return currentState;
  }

  public SuperStructureState getDesiredState() {
    return desiredState;
  }
}