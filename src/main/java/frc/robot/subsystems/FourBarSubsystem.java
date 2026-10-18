package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Visualizer;
import frc.robot.tuning.PerfMonitor;
import frc.robot.tuning.GravityEstimator;
import frc.robot.tuning.MoveMetrics;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningMode;
import frc.robot.tuning.TuningTelemetry;
import java.util.ArrayList;
import java.util.List;

/**
 * 4-bar linkage (the hopper floor is its coupler) driven by two Krakens (leader + opposed follower) using the built-in
 * encoder. Angles are mechanism rotations of the driven link, 0 = horizontal. The starting values live in
 * {@link FourBarConstants}; everything tunable is also live under /Tuning/FourBar in NetworkTables (edit it in Elastic).
 */
public class FourBarSubsystem extends SubsystemBase implements AutoCloseable {
    private static final int kNumConfigAttempts = 5;

    // ---------------------------------------------------------------- live-tunable values (/Tuning/FourBar/...)
    private final List<Object> tunables = new ArrayList<>();
    private <T> T track(T t) {
        tunables.add(t);
        return t;
    }

    //Current Limit in tuner : 55

    private static final double kUp = FourBarConstants.UpStopRot;
    private static final double kDown = FourBarConstants.DownStopRot;

    // gains
    private final TunableNumber kP = track(Tunables.number("FourBar/Gains/kP", FourBarConstants.kP, 0, 5000, "FourBarConstants.kP", v -> applyGains()));
    private final TunableNumber kI = track(Tunables.number("FourBar/Gains/kI", FourBarConstants.kI, 0, 500, "FourBarConstants.kI", v -> applyGains()));
    private final TunableNumber kD = track(Tunables.number("FourBar/Gains/kD", FourBarConstants.kD, 0, 500, "FourBarConstants.kD", v -> applyGains()));
    private final TunableNumber kS = track(Tunables.number("FourBar/Gains/kS", FourBarConstants.kS, 0, 3, "FourBarConstants.kS", v -> applyGains()));
    private final TunableNumber kV = track(Tunables.number("FourBar/Gains/kV", FourBarConstants.kV, 0, 60, "FourBarConstants.kV", v -> applyGains()));
    private final TunableNumber kA = track(Tunables.number("FourBar/Gains/kA", FourBarConstants.kA, 0, 20, "FourBarConstants.kA", v -> applyGains()));
    private final TunableNumber kG = track(Tunables.number("FourBar/Gains/kG", FourBarConstants.kG, 0, 6, "FourBarConstants.kG", v -> applyGains()));
    // Motion Magic profile
    private final TunableNumber cruiseVelocity = track(Tunables.number("FourBar/Motion/CruiseVelocityRotPerSec",
        FourBarConstants.CruiseVelocityRotPerSec, 0.02, 0.6, "FourBarConstants.CruiseVelocityRotPerSec", v -> applyMotionMagic()));
    private final TunableNumber acceleration = track(Tunables.number("FourBar/Motion/AccelerationRotPerSecSq",
        FourBarConstants.AccelerationRotPerSecSq, 0.1, 20, "FourBarConstants.AccelerationRotPerSecSq", v -> applyMotionMagic()));
    private final TunableNumber jerk = track(Tunables.number("FourBar/Motion/JerkRotPerSecCubed",
        FourBarConstants.JerkRotPerSecCubed, 0, 500, "FourBarConstants.JerkRotPerSecCubed", v -> applyMotionMagic()));
    // setpoints (the floor is flat except while outtaking)
    private final TunableNumber flatRot = track(Tunables.number("FourBar/Setpoints/FlatRot",
        FourBarConstants.FlatRot, kDown + 0.0005, kUp - FourBarConstants.UpSoftLimitMarginRot, "FourBarConstants.FlatRot"));
    private final TunableNumber raisedRot = track(Tunables.number("FourBar/Setpoints/RaisedRot",
        FourBarConstants.RaisedRot, kDown + 0.0005, kUp - FourBarConstants.UpSoftLimitMarginRot, "FourBarConstants.RaisedRot"));
    // "at setpoint" tolerances
    private final TunableNumber positionTolerance = track(Tunables.number("FourBar/Tolerance/PositionRot",
        FourBarConstants.PositionToleranceRot, 0.0003, 0.02, "FourBarConstants.PositionToleranceRot"));
    private final TunableNumber velocityTolerance = track(Tunables.number("FourBar/Tolerance/VelocityRotPerSec",
        FourBarConstants.VelocityToleranceRotPerSec, 0.002, 0.2, "FourBarConstants.VelocityToleranceRotPerSec"));
    // limits
    private final TunableNumber statorCurrentLimit = track(Tunables.number("FourBar/Limits/StatorCurrentAmps",
        FourBarConstants.StatorCurrentLimitAmps, 5, 80, "FourBarConstants.StatorCurrentLimitAmps", v -> applyLimits()));
    private final TunableNumber softLimitMargin = track(Tunables.number("FourBar/Limits/SoftLimitMarginRot",
        FourBarConstants.SoftLimitMarginRot, 0.0005, 0.03, "FourBarConstants.SoftLimitMarginRot", v -> applyLimits()));
    private final TunableNumber upSoftLimitMargin = track(Tunables.number("FourBar/Limits/UpSoftLimitMarginRot",
        FourBarConstants.UpSoftLimitMarginRot, 0.004, 0.05, "FourBarConstants.UpSoftLimitMarginRot", v -> applyLimits()));
    // 3D linkage geometry (visualization only)
    private final TunableNumber floorBackX = track(Tunables.number("FourBar/3D/FloorBackPivotX", FourBarConstants.FloorBackPivotX, -1, 1, "FourBarConstants.FloorBackPivotX"));
    private final TunableNumber floorBackZ = track(Tunables.number("FourBar/3D/FloorBackPivotZ", FourBarConstants.FloorBackPivotZ, 0, 1, "FourBarConstants.FloorBackPivotZ"));
    private final TunableNumber floorFrontX = track(Tunables.number("FourBar/3D/FloorFrontPivotX", FourBarConstants.FloorFrontPivotX, -1, 1, "FourBarConstants.FloorFrontPivotX"));
    private final TunableNumber floorFrontZ = track(Tunables.number("FourBar/3D/FloorFrontPivotZ", FourBarConstants.FloorFrontPivotZ, 0, 1, "FourBarConstants.FloorFrontPivotZ"));
    private final TunableNumber shortLink = track(Tunables.number("FourBar/3D/ShortLinkLengthMeters", FourBarConstants.ShortLinkLengthMeters, 0.01, 0.3, "FourBarConstants.ShortLinkLengthMeters"));
    private final TunableNumber groundDX = track(Tunables.number("FourBar/3D/ShortLinkGroundDX", FourBarConstants.ShortLinkGroundDX, -0.5, 0.5, "FourBarConstants.ShortLinkGroundDX"));
    private final TunableNumber groundDZ = track(Tunables.number("FourBar/3D/ShortLinkGroundDZ", FourBarConstants.ShortLinkGroundDZ, -0.5, 0.5, "FourBarConstants.ShortLinkGroundDZ"));

    private final Runnable reapplyForSafeMode = () -> {
        applyMotionMagic();
        applyLimits();
    };

    private double softLimitForwardRot() {
        return kUp - upSoftLimitMargin.get();
    }

    private double softLimitReverseRot() {
        return kDown + softLimitMargin.get();
    }

    private final TalonFX leader = new TalonFX(FourBarConstants.LeaderID, FourBarConstants.CanBus);
    private final TalonFX follower = new TalonFX(FourBarConstants.FollowerID, FourBarConstants.CanBus);

    /* status signals */
    private final StatusSignal<Angle> position = leader.getPosition(false);
    private final StatusSignal<AngularVelocity> velocity = leader.getVelocity(false);
    private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent(false);
    private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent(false);
    private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent(false);
    private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent(false);
    private final StatusSignal<Voltage> leaderMotorVoltage = leader.getMotorVoltage(false);
    // diagnostics: what the motors themselves report (published under /Tuning/FourBar/Diagnostics)
    private final StatusSignal<com.ctre.phoenix6.signals.ControlModeValue> diagControlMode = leader.getControlMode(false);
    private final StatusSignal<Boolean> diagForwardSoftLimit = leader.getFault_ForwardSoftLimit(false);
    private final StatusSignal<Boolean> diagReverseSoftLimit = leader.getFault_ReverseSoftLimit(false);
    private final StatusSignal<Boolean> diagUnlicensed = leader.getFault_UnlicensedFeatureInUse(false);
    private com.ctre.phoenix6.StatusCode lastControlStatus = com.ctre.phoenix6.StatusCode.OK;

    /* control requests */
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withEnableFOC(false);
    private final NeutralOut neutral = new NeutralOut();

    private double targetRot = FourBarConstants.FlatRot;
    private double lastPosedTarget = Double.NaN;

    // tuning aids: per-move measurements and the gravity feedforward the mechanism actually needs
    private final MoveMetrics moveMetrics = new MoveMetrics();
    private final GravityEstimator gravityEstimator = new GravityEstimator(0.5, 2.0);
    private double lastPeriodicSeconds = Timer.getFPGATimestamp();

    // position-lost detection: a motor controller reboot resets its position to 0, and a sudden jump means the reading cannot be trusted
    private static final double kJumpRotPerLoop = 0.03; // the floor cannot move this far in one 20 ms loop (cruise is 0.3 rot/s)
    private double lastAngleRot = Double.NaN;
    private boolean positionLost = false;

    /** true if the motor rebooted or the reading jumped: the floor position can no longer be trusted until it is zeroed again. */
    public boolean isPositionLost() {
        return positionLost;
    }

    /** Test hook: pretends the position was lost. */
    public void markPositionLostForTesting() {
        positionLost = true;
    }
    private int movesMeasured = 0;

    /* simulation */
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private static final double kMaxSimStepSeconds = 0.01;
    private SingleJointedArmSim armSim = buildArmSim(FourBarConstants.MovingMassEmptyKg,
        Rotations.of(FourBarConstants.DownStopRot).in(Radians), 0.0);
    private static final double kSimBootAngleRad = Rotations.of(FourBarConstants.DownStopRot).in(Radians);
    private Notifier simNotifier = null;
    private double lastSimTime = 0.0;

    public FourBarSubsystem() {
        TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
        leaderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfigs.MotorOutput.Inverted = FourBarConstants.LeaderInverted
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        leaderConfigs.Feedback.SensorToMechanismRatio = FourBarConstants.GearReduction;
        leaderConfigs.ClosedLoopGeneral.ContinuousWrap = false;
        leaderConfigs.CurrentLimits = currentLimits();
        leaderConfigs.SoftwareLimitSwitch = softLimits();
        leaderConfigs.Slot0 = slot0();
        leaderConfigs.MotionMagic = motionMagicConfigs();

        // The follower only needs the output-side configs; its direction comes from the Opposed alignment.
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfigs.CurrentLimits = currentLimits();

        applyConfigs(leader, leaderConfigs);
        applyConfigs(follower, followerConfigs);
        follower.setControl(new Follower(FourBarConstants.LeaderID, MotorAlignmentValue.Opposed));

        // The control loop and the gravity estimate need these fast; the rest are only logged, so they use less CAN bandwidth.
        BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, leaderStatorCurrent, leaderMotorVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(50, followerStatorCurrent);
        BaseStatusSignal.setUpdateFrequencyForAll(20, leaderSupplyCurrent, followerSupplyCurrent);

        // Start the sim first so the raw rotor value is in a known state before it is offset by the seed.
        if (Utils.isSimulation()) {
            startSimThread();
            Timer.delay(0.1); // give the sim loop (and the sim'd device) time to apply the zeroed raw rotor
        }

        // The robot always boots against the hard stop.
        seedPosition();
        refreshSignals();

        TuningMode.onChange(reapplyForSafeMode);
        track(Tunables.action("FourBar/Actions/ApplySuggestedKG", this::applySuggestedKg));
        track(Tunables.action("FourBar/Actions/ResetGravityEstimate", gravityEstimator::reset));
    }

    // ---------------------------------------------------------------- applying tuned values to the motors

    private Slot0Configs slot0() {
        Slot0Configs s = new Slot0Configs();
        s.GravityType = GravityTypeValue.Arm_Cosine;
        s.kP = kP.get();
        s.kI = kI.get();
        s.kD = kD.get();
        s.kS = kS.get();
        s.kV = kV.get();
        s.kA = kA.get();
        s.kG = kG.get();
        return s;
    }

    private MotionMagicConfigs motionMagicConfigs() {
        MotionMagicConfigs m = new MotionMagicConfigs();
        m.MotionMagicCruiseVelocity = TuningMode.scale(cruiseVelocity.get());
        m.MotionMagicAcceleration = TuningMode.scale(acceleration.get());
        m.MotionMagicJerk = TuningMode.scale(jerk.get());
        return m;
    }

    private CurrentLimitsConfigs currentLimits() {
        CurrentLimitsConfigs c = new CurrentLimitsConfigs();
        c.StatorCurrentLimit = TuningMode.limitAmps(statorCurrentLimit.get());
        c.StatorCurrentLimitEnable = true;
        return c;
    }

    private SoftwareLimitSwitchConfigs softLimits() {
        SoftwareLimitSwitchConfigs c = new SoftwareLimitSwitchConfigs();
        c.ForwardSoftLimitEnable = true;
        c.ForwardSoftLimitThreshold = softLimitForwardRot();
        c.ReverseSoftLimitEnable = true;
        c.ReverseSoftLimitThreshold = softLimitReverseRot();
        return c;
    }

    private void applyGains() {
        leader.getConfigurator().apply(slot0(), 0.05);
    }

    private void applyMotionMagic() {
        leader.getConfigurator().apply(motionMagicConfigs(), 0.05);
    }

    private void applyLimits() {
        leader.getConfigurator().apply(currentLimits(), 0.05);
        follower.getConfigurator().apply(currentLimits(), 0.05);
        leader.getConfigurator().apply(softLimits(), 0.05);
    }

    /** Sets the kG tunable to the value the gravity estimator measured (the "ApplySuggestedKG" button in Elastic). */
    private void applySuggestedKg() {
        if (gravityEstimator.hasEstimate()) {
            kG.set(Math.max(0.0, gravityEstimator.getKg()));
        }
    }

    private static void applyConfigs(TalonFX motor, TalonFXConfiguration configs) {
        for (int i = 0; i < kNumConfigAttempts; ++i) {
            if (motor.getConfigurator().apply(configs).isOK()) return;
        }
        DogLog.logFault("FourBar config failed for TalonFX " + motor.getDeviceID());
    }

    private void seedPosition() {
        leader.setPosition(Rotations.of(FourBarConstants.DownStopRot), 0.25); // boots against the down stop (floor flat)
    }

    private void refreshSignals() {
        BaseStatusSignal.refreshAll(
            position, velocity, leaderStatorCurrent, followerStatorCurrent,
            leaderSupplyCurrent, followerSupplyCurrent, leaderMotorVoltage,
            diagControlMode, diagForwardSoftLimit, diagReverseSoftLimit, diagUnlicensed);
    }

    // ---------------------------------------------------------------- getters

    public Angle getPosition() {
        return position.getValue();
    }

    public AngularVelocity getVelocity() {
        return velocity.getValue();
    }

    public Current getLeaderStatorCurrent() {
        return leaderStatorCurrent.getValue();
    }

    public Current getFollowerStatorCurrent() {
        return followerStatorCurrent.getValue();
    }

    public Angle getTarget() {
        return Rotations.of(targetRot);
    }

    /** Where the floor is flat (tunable). */
    public double getFlatRot() {
        return flatRot.get();
    }

    /** Where the floor is raised for outtake (tunable). */
    public double getRaisedRot() {
        return raisedRot.get();
    }

    /** A target clamped inside the soft limits. */
    public double clampToLimits(double rot) {
        return MathUtil.clamp(rot, softLimitReverseRot(), softLimitForwardRot());
    }

    /** @return true when the link is within tolerance of the commanded target and basically stopped. */
    public boolean atSetpoint() {
        return Math.abs(position.getValue().in(Rotations) - targetRot) < positionTolerance.get()
            && Math.abs(velocity.getValue().in(RotationsPerSecond)) < velocityTolerance.get();
    }

    // ---------------------------------------------------------------- control

    /** Sets the Motion Magic target (clamped to the soft limits). Call once; the TalonFX holds it. */
    public void setTarget(Angle target) {
        targetRot = clampToLimits(target.in(Rotations));
        lastControlStatus = leader.setControl(motionMagic.withPosition(targetRot));
    }

    /** Drives to the angle and finishes when {@link #atSetpoint()}. The target is held after the command ends. */
    public Command goTo(Angle angle) {
        return run(() -> setTarget(angle)).until(this::atSetpoint).withName("FourBar goTo");
    }

    /**
     * Tells the motor "the floor is resting flat on its down stop right now" (re-seeds the position there), then holds flat. It does NOT drive
     * anywhere: an earlier version drove into the hard stop to find it, and with a 106:1 reduction that sheared the pivot's bolts and gears.
     * Only valid with the floor really resting flat; a wrong zero would later drive the floor past its up stop. So it only works while the
     * robot is disabled, and refuses if the floor is moving. Returns true if it zeroed.
     */
    public boolean zeroHere() {
        if (!edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
            TuningTelemetry.put("FourBar/Status", "Zero refused: only works while the robot is DISABLED, with the floor resting flat");
            DogLog.logFault("FourBar zero refused: robot is enabled");
            return false;
        }
        if (Math.abs(velocity.getValue().in(RotationsPerSecond)) > 0.02) {
            TuningTelemetry.put("FourBar/Status", "Zero refused: the floor is moving");
            return false;
        }
        seedPosition(); // the down stop is the zero reference
        positionLost = false;
        lastAngleRot = Double.NaN; // the reading is about to jump to the seed value
        setTarget(Rotations.of(flatRot.get()));
        TuningTelemetry.put("FourBar/Status", "Zeroed: the floor is now treated as flat on its down stop");
        return true;
    }

    /** Test hook: sends a Motion Magic target WITHOUT clamping, to exercise the firmware soft limits. */
    void setUnclampedTargetForTesting(double rot) {
        targetRot = rot;
        leader.setControl(motionMagic.withPosition(rot));
    }

    /** Test hooks: read back what the motor itself is configured with, to prove tuned values really reach it. */
    double readKpFromMotorForTesting() {
        Slot0Configs c = new Slot0Configs();
        leader.getConfigurator().refresh(c, 0.5);
        return c.kP;
    }

    double readStatorLimitFromMotorForTesting() {
        CurrentLimitsConfigs c = new CurrentLimitsConfigs();
        leader.getConfigurator().refresh(c, 0.5);
        return c.StatorCurrentLimit;
    }

    double readCruiseVelocityFromMotorForTesting() {
        MotionMagicConfigs c = new MotionMagicConfigs();
        leader.getConfigurator().refresh(c, 0.5);
        return c.MotionMagicCruiseVelocity;
    }

    /** Test hook: overwrites the encoder position without moving the sim link (simulates a lost/bad zero). */
    void corruptPositionForTesting(double rot) {
        leader.setPosition(Rotations.of(rot), 0.25);
    }

    /** Neutral output (brake mode). Useful when disabling the mechanism. */
    public void stop() {
        leader.setControl(neutral);
    }

    // ---------------------------------------------------------------- periodic / logging

    @Override
    public void periodic() {
        long perfStart = PerfMonitor.now();
        periodicInner();
        PerfMonitor.lap("Sub.FourBar", perfStart);
    }

    private void periodicInner() {
        refreshSignals();

        double now = Timer.getFPGATimestamp();
        double dt = now - lastPeriodicSeconds;
        lastPeriodicSeconds = now;

        double angleRot = position.getValue().in(Rotations);
        boolean rebooted = leader.hasResetOccurred() || follower.hasResetOccurred();
        boolean jumped = !Double.isNaN(lastAngleRot) && Math.abs(angleRot - lastAngleRot) > kJumpRotPerLoop;
        lastAngleRot = angleRot;
        if ((rebooted || jumped) && !positionLost) {
            positionLost = true;
            DogLog.logFault("FourBar position lost (" + (rebooted ? "motor rebooted" : "reading jumped") + "): re-zero the floor");
        }
        double velRotPerSec = velocity.getValue().in(RotationsPerSecond);
        double amps = leaderStatorCurrent.getValue().in(Amps);
        double volts = leaderMotorVoltage.getValue().in(edu.wpi.first.units.Units.Volts);

        DogLog.log("FourBar/AngleRot", angleRot);
        DogLog.log("FourBar/TargetRot", targetRot);
        DogLog.log("FourBar/AtSetpoint", atSetpoint());
        DogLog.log("FourBar/VelocityRotPerSec", velRotPerSec);
        DogLog.log("FourBar/LeaderStatorCurrentAmps", amps);
        DogLog.log("FourBar/FollowerStatorCurrentAmps", followerStatorCurrent.getValue().in(Amps));
        DogLog.log("FourBar/LeaderSupplyCurrentAmps", leaderSupplyCurrent.getValue().in(Amps));
        DogLog.log("FourBar/FollowerSupplyCurrentAmps", followerSupplyCurrent.getValue().in(Amps));

        // ---- tuning aids (graph these in Elastic: Live/PositionRot against Live/TargetRot)
        TuningTelemetry.putLive("FourBar/Live/PositionRot", angleRot);
        TuningTelemetry.putLive("FourBar/Live/TargetRot", targetRot);
        TuningTelemetry.putLive("FourBar/Live/ErrorRot", targetRot - angleRot);
        TuningTelemetry.putLive("FourBar/Live/VelocityRotPerSec", velRotPerSec);
        TuningTelemetry.putLive("FourBar/Live/MotorVolts", volts);
        TuningTelemetry.putLive("FourBar/Live/StatorCurrentAmps", amps);
        TuningTelemetry.putLive("FourBar/Live/AtSetpoint", atSetpoint());
        TuningTelemetry.putLive("FourBar/Live/SafeMode", TuningMode.isSafe());
        TuningTelemetry.put("FourBar/Diagnostics/LastSetControlStatus", lastControlStatus.toString());
        TuningTelemetry.put("FourBar/Diagnostics/LeaderControlMode", String.valueOf(diagControlMode.getValue()));
        TuningTelemetry.putLive("FourBar/Diagnostics/LeaderFound", position.getStatus().isOK());
        TuningTelemetry.putLive("FourBar/Diagnostics/FollowerFound", followerStatorCurrent.getStatus().isOK());
        TuningTelemetry.putLive("FourBar/Diagnostics/ForwardSoftLimitFault", Boolean.TRUE.equals(diagForwardSoftLimit.getValue()));
        TuningTelemetry.putLive("FourBar/Diagnostics/ReverseSoftLimitFault", Boolean.TRUE.equals(diagReverseSoftLimit.getValue()));
        TuningTelemetry.putLive("FourBar/Diagnostics/UnlicensedFeatureInUse", Boolean.TRUE.equals(diagUnlicensed.getValue()));
        TuningTelemetry.putLive("FourBar/Diagnostics/FollowerStatorCurrentAmps", followerStatorCurrent.getValue().in(Amps));

        {
            moveMetrics.update(now, angleRot, targetRot, amps, positionTolerance.get()).ifPresent(this::publishMove);
            gravityEstimator.update(volts, angleRot, atSetpoint(), dt);
        }
        if (gravityEstimator.hasEstimate()) {
            TuningTelemetry.putLive("FourBar/Suggested/kG", gravityEstimator.getKg());
            TuningTelemetry.putLive("FourBar/Suggested/kGMinusCurrent", gravityEstimator.getKg() - kG.get());
        }
        TuningTelemetry.putLive("FourBar/Suggested/CurrentKG", kG.get());

        // AdvantageScope: pose of the hopper floor relative to the robot (the model is zeroed flat; see
        // advantagescope/Robot_BBot/config.json).
        Visualizer.setFloor(getFloorPose(angleRot, geometry()));
        if (targetRot != lastPosedTarget) { // the target pose only changes when the target does
            lastPosedTarget = targetRot;
            DogLog.log("FourBar/TargetLinkPose3d", getFloorPose(targetRot, geometry()));
        }
    }

    private void publishMove(MoveMetrics.Result r) {
        movesMeasured++;
        TuningTelemetry.put("FourBar/Result/MovesMeasured", movesMeasured);
        TuningTelemetry.put("FourBar/Result/DistanceRot", r.distance());
        TuningTelemetry.put("FourBar/Result/SettleSeconds", r.settleSeconds());
        TuningTelemetry.put("FourBar/Result/OvershootRot", r.overshoot());
        TuningTelemetry.put("FourBar/Result/SteadyStateErrorRot", r.steadyStateError());
        TuningTelemetry.put("FourBar/Result/PeakStatorCurrentAmps", r.peakCurrentAmps());
        TuningTelemetry.put("FourBar/Result/Settled", r.settled());
        TuningTelemetry.put("FourBar/Result/Advice", MoveMetrics.advise(r, positionTolerance.get(), 1.5));
    }

    /** The linkage dimensions used to draw the floor, as currently tuned. */
    record FloorGeometry(double linkLength, double shortLink, double backX, double backZ, double frontX, double frontZ,
                         double groundDX, double groundDZ, double flatAngleRot) {}

    private FloorGeometry geometry() {
        return new FloorGeometry(FourBarConstants.LinkLengthMeters, shortLink.get(), floorBackX.get(), floorBackZ.get(),
            floorFrontX.get(), floorFrontZ.get(), groundDX.get(), groundDZ.get(), FourBarConstants.DownStopRot);
    }

    /**
     * Floor pose for a link angle, from the 4-bar's kinematics. The floor is the coupler: its back pivot rides the end of the
     * long (driven) link and its front pivot rides the end of the short link, so it both tilts and moves along an arc.
     * Everything is in the robot's X (forward) / Z (up) plane. The component model is drawn in the FLAT pose, so the pose
     * returned is the rigid motion from the flat coupler to the coupler at this link angle.
     */
    static Pose3d getFloorPose(double linkAngleRot) {
        return getFloorPose(linkAngleRot, new FloorGeometry(FourBarConstants.LinkLengthMeters,
            FourBarConstants.ShortLinkLengthMeters, FourBarConstants.FloorBackPivotX, FourBarConstants.FloorBackPivotZ,
            FourBarConstants.FloorFrontPivotX, FourBarConstants.FloorFrontPivotZ, FourBarConstants.ShortLinkGroundDX,
            FourBarConstants.ShortLinkGroundDZ, FourBarConstants.DownStopRot));
    }

    static Pose3d getFloorPose(double linkAngleRot, FloorGeometry g) {
        double l1 = g.linkLength();
        double l3 = g.shortLink();
        double b0x = g.backX(), b0z = g.backZ();
        double f0x = g.frontX(), f0z = g.frontZ();
        double coupler = Math.hypot(f0x - b0x, f0z - b0z);
        // Link angle is measured above the backward horizontal: back pivot = hub + l1 * (-cos a, sin a).
        double aFlat = Rotations.of(g.flatAngleRot()).in(Radians);
        double hubX = b0x + l1 * Math.cos(aFlat);
        double hubZ = b0z - l1 * Math.sin(aFlat);
        double sx = hubX + g.groundDX();
        double sz = hubZ + g.groundDZ();

        double a = Rotations.of(linkAngleRot).in(Radians);
        double bx = hubX - l1 * Math.cos(a);
        double bz = hubZ + l1 * Math.sin(a);

        // Front pivot: intersection of the circle of radius l3 about the short link's ground pivot with the circle of radius
        // "coupler" about the back pivot, on the same side as in the flat pose.
        double dx = bx - sx, dz = bz - sz;
        double d = Math.max(1e-6, Math.hypot(dx, dz));
        double along = (l3 * l3 - coupler * coupler + d * d) / (2 * d);
        double h = Math.sqrt(Math.max(0.0, l3 * l3 - along * along));
        double px = sx + along * dx / d, pz = sz + along * dz / d;
        double fx1 = px + h * (-dz / d), fz1 = pz + h * (dx / d);
        double fx2 = px - h * (-dz / d), fz2 = pz - h * (dx / d);
        double flatSide = Math.signum((b0x - sx) * (f0z - sz) - (b0z - sz) * (f0x - sx));
        double side1 = Math.signum(dx * (fz1 - sz) - dz * (fx1 - sx));
        double fx = side1 == flatSide ? fx1 : fx2;
        double fz = side1 == flatSide ? fz1 : fz2;

        // Rigid motion of the coupler: tilt change (counter-clockwise, front up = positive) and the translation that puts
        // the back pivot where it now is.
        double tilt = Math.atan2(fz - bz, fx - bx);
        double tilt0 = Math.atan2(f0z - b0z, f0x - b0x);
        double delta = tilt - tilt0;
        double rx = b0x * Math.cos(delta) - b0z * Math.sin(delta);
        double rz = b0x * Math.sin(delta) + b0z * Math.cos(delta);
        // Pitch about +Y is positive nose-down, so a counter-clockwise (front-up) tilt is a negative pitch.
        return new Pose3d(new Translation3d(bx - rx, 0.0, bz - rz), new Rotation3d(0, -delta, 0));
    }

    // ---------------------------------------------------------------- simulation

    /** @return the sim's true link angle in rotations (simulation only). */
    public double getSimAngleRot() {
        return Radians.of(armSim.getAngleRads()).in(Rotations);
    }

    /** Changes the simulated moving mass (empty vs loaded with balls), keeping the link where it is. Simulation only. */
    public void setSimMassKg(double massKg) {
        armSim = buildArmSim(massKg, armSim.getAngleRads(), armSim.getVelocityRadPerSec());
    }

    /**
     * Point mass at the CG distance: I = m r^2, gravity torque = m g r cos(theta). SingleJointedArmSim bakes in a
     * uniform-rod gravity term (alpha = 1.5 g cos / armLength), so the length passed in is 1.5 r to get g cos / r.
     * Hard stops are the real up/down stops.
     */
    private static SingleJointedArmSim buildArmSim(double massKg, double angleRad, double velocityRadPerSec) {
        double r = FourBarConstants.CgDistanceMeters;
        var sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(2),
            FourBarConstants.GearReduction,
            massKg * r * r,
            1.5 * r,
            Rotations.of(FourBarConstants.DownStopRot).in(Radians),
            Rotations.of(FourBarConstants.UpStopRot).in(Radians),
            true,
            angleRad);
        sim.setState(angleRad, velocityRadPerSec);
        return sim;
    }

    /** Teleports the physical sim link (does not touch the TalonFX position). Simulation only. */
    public void setSimAngleRot(double rot) {
        armSim.setState(Rotations.of(rot).in(Radians), 0.0);
    }

    private void startSimThread() {
        ChassisReference leaderRef = FourBarConstants.LeaderInverted
            ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        ChassisReference followerRef = FourBarConstants.LeaderInverted
            ? ChassisReference.CounterClockwise_Positive : ChassisReference.Clockwise_Positive;
        leader.getSimState().Orientation = leaderRef;
        leader.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        follower.getSimState().Orientation = followerRef;
        follower.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            // Cap the step: if this thread is starved (a busy machine) one huge step could throw the mechanism sim off.
            final double deltaTime = Math.min(currentTime - lastSimTime, kMaxSimStepSeconds);
            lastSimTime = currentTime;

            final var leaderSim = leader.getSimState();
            final var followerSim = follower.getSimState();
            leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            // Both motors are on the same output; the follower's opposed orientation makes it push the same way.
            armSim.setInputVoltage(leaderSim.getMotorVoltage());
            armSim.update(deltaTime);

            // Like the real encoder, the raw rotor reads 0 at power-up (link on the stow stop); the seeded
            // position offset is what turns that into a mechanism angle.
            double rotorPos = (armSim.getAngleRads() - kSimBootAngleRad) * FourBarConstants.GearReduction;
            double rotorVel = armSim.getVelocityRadPerSec() * FourBarConstants.GearReduction;
            leaderSim.setRawRotorPosition(Radians.of(rotorPos));
            leaderSim.setRotorVelocity(RadiansPerSecond.of(rotorVel));
            followerSim.setRawRotorPosition(Radians.of(rotorPos));
            followerSim.setRotorVelocity(RadiansPerSecond.of(rotorVel));
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /** Stops the sim thread, releases the CAN devices and unregisters the subsystem (used by tests). */
    @Override
    public void close() {
        if (simNotifier != null) {
            simNotifier.stop();
            simNotifier.close();
            simNotifier = null;
        }
        TuningMode.removeListener(reapplyForSafeMode);
        Tunables.unregister(tunables);
        leader.close();
        follower.close();
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(this);
    }
}
