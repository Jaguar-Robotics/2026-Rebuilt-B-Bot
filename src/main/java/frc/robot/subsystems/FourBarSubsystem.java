package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Visualizer;

/**
 * Parallel 4-bar linkage driven by two Krakens (leader + opposed follower) using the built-in encoder.
 * Angles are mechanism rotations of the driven link, 0 = horizontal. All tunables live in
 * {@link FourBarConstants}.
 */
public class FourBarSubsystem extends SubsystemBase implements AutoCloseable {
    private static final int kNumConfigAttempts = 5;

    private static final double kSoftLimitForwardRot = FourBarConstants.UpStopRot - FourBarConstants.SoftLimitMarginRot;
    private static final double kSoftLimitReverseRot = FourBarConstants.DownStopRot + FourBarConstants.SoftLimitMarginRot;

    private final TalonFX leader = new TalonFX(FourBarConstants.LeaderID, FourBarConstants.CanBus);
    private final TalonFX follower = new TalonFX(FourBarConstants.FollowerID, FourBarConstants.CanBus);

    /* status signals */
    private final StatusSignal<Angle> position = leader.getPosition(false);
    private final StatusSignal<AngularVelocity> velocity = leader.getVelocity(false);
    private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent(false);
    private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent(false);
    private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent(false);
    private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent(false);

    /* control requests */
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withEnableFOC(false);
    private final VoltageOut homingRequest = new VoltageOut(0).withEnableFOC(false).withIgnoreSoftwareLimits(true);
    private final NeutralOut neutral = new NeutralOut();

    private double targetRot = FourBarConstants.StowRot;
    private Debouncer stallDebouncer = new Debouncer(FourBarConstants.HomingStallDebounceSeconds);

    /* simulation */
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private SingleJointedArmSim armSim = buildArmSim(FourBarConstants.MovingMassEmptyKg,
        Rotations.of(FourBarConstants.UpStopRot).in(Radians), 0.0);
    private static final double kSimBootAngleRad = Rotations.of(FourBarConstants.UpStopRot).in(Radians);
    private Notifier simNotifier = null;
    private double lastSimTime = 0.0;

    public FourBarSubsystem() {
        TalonFXConfiguration leaderConfigs = new TalonFXConfiguration();
        leaderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfigs.MotorOutput.Inverted = FourBarConstants.LeaderInverted
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        leaderConfigs.Feedback.SensorToMechanismRatio = FourBarConstants.GearReduction;
        leaderConfigs.CurrentLimits.StatorCurrentLimit = FourBarConstants.StatorCurrentLimitAmps;
        leaderConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kSoftLimitForwardRot;
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kSoftLimitReverseRot;
        leaderConfigs.ClosedLoopGeneral.ContinuousWrap = false;

        leaderConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        leaderConfigs.Slot0.kP = FourBarConstants.kP;
        leaderConfigs.Slot0.kI = FourBarConstants.kI;
        leaderConfigs.Slot0.kD = FourBarConstants.kD;
        leaderConfigs.Slot0.kS = FourBarConstants.kS;
        leaderConfigs.Slot0.kV = FourBarConstants.kV;
        leaderConfigs.Slot0.kA = FourBarConstants.kA;
        leaderConfigs.Slot0.kG = FourBarConstants.kG;
        leaderConfigs.MotionMagic.MotionMagicCruiseVelocity = FourBarConstants.CruiseVelocityRotPerSec;
        leaderConfigs.MotionMagic.MotionMagicAcceleration = FourBarConstants.AccelerationRotPerSecSq;
        leaderConfigs.MotionMagic.MotionMagicJerk = FourBarConstants.JerkRotPerSecCubed;

        // The follower only needs the output-side configs; its direction comes from the Opposed alignment.
        TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
        followerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfigs.CurrentLimits.StatorCurrentLimit = FourBarConstants.StatorCurrentLimitAmps;
        followerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        applyConfigs(leader, leaderConfigs);
        applyConfigs(follower, followerConfigs);
        follower.setControl(new Follower(FourBarConstants.LeaderID, MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(100,
            position, velocity, leaderStatorCurrent, followerStatorCurrent,
            leaderSupplyCurrent, followerSupplyCurrent);

        // Start the sim first so the raw rotor value is in a known state before it is offset by the seed.
        if (Utils.isSimulation()) {
            startSimThread();
            Timer.delay(0.1); // give the sim loop (and the sim'd device) time to apply the zeroed raw rotor
        }

        // The robot always boots against the hard stop.
        seedPosition();
        refreshSignals();
    }

    private static void applyConfigs(TalonFX motor, TalonFXConfiguration configs) {
        for (int i = 0; i < kNumConfigAttempts; ++i) {
            if (motor.getConfigurator().apply(configs).isOK()) return;
        }
        DogLog.logFault("FourBar config failed for TalonFX " + motor.getDeviceID());
    }

    private void seedPosition() {
        leader.setPosition(Rotations.of(FourBarConstants.UpStopRot), 0.25);
    }

    private void refreshSignals() {
        BaseStatusSignal.refreshAll(
            position, velocity, leaderStatorCurrent, followerStatorCurrent,
            leaderSupplyCurrent, followerSupplyCurrent);
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

    /** @return true when the link is within tolerance of the commanded target and basically stopped. */
    public boolean atSetpoint() {
        return Math.abs(position.getValue().in(Rotations) - targetRot) < FourBarConstants.PositionToleranceRot
            && Math.abs(velocity.getValue().in(RotationsPerSecond)) < FourBarConstants.VelocityToleranceRotPerSec;
    }

    // ---------------------------------------------------------------- control

    /** Sets the Motion Magic target (clamped to the soft limits). Call once; the TalonFX holds it. */
    public void setTarget(Angle target) {
        targetRot = MathUtil.clamp(target.in(Rotations), kSoftLimitReverseRot, kSoftLimitForwardRot);
        leader.setControl(motionMagic.withPosition(targetRot));
    }

    /** Drives to the angle and finishes when {@link #atSetpoint()}. The target is held after the command ends. */
    public Command goTo(Angle angle) {
        return run(() -> setTarget(angle)).until(this::atSetpoint).withName("FourBar goTo");
    }

    /**
     * Drives slowly into the stow hard stop, detects the stall (stator current spike + near-zero velocity),
     * then resets the position to the stowed angle. If no stall is seen before the timeout, or the command
     * is interrupted, the position is NOT reset and the mechanism holds where it is.
     */
    public Command rehome() {
        final double volts = Math.copySign(FourBarConstants.HomingVolts,
            FourBarConstants.UpStopRot - FourBarConstants.DownStopRot);
        final boolean[] stalled = {false};
        return startRun(
                () -> {
                    stalled[0] = false;
                    stallDebouncer = new Debouncer(FourBarConstants.HomingStallDebounceSeconds);
                },
                () -> {
                    leader.setControl(homingRequest.withOutput(volts));
                    stalled[0] = stallDebouncer.calculate(isStallSample());
                })
            .until(() -> stalled[0])
            .withTimeout(FourBarConstants.HomingTimeoutSeconds)
            .finallyDo(interrupted -> {
                if (stalled[0]) {
                    seedPosition(); // the up stop is the zero reference
                    setTarget(Rotations.of(FourBarConstants.StowRot));
                } else {
                    DogLog.logFault("FourBar re-home did not detect the hard stop");
                    setTarget(getPosition()); // hold where we are
                }
            })
            .withName("FourBar rehome");
    }

    private boolean isStallSample() {
        return Math.abs(leaderStatorCurrent.getValue().in(Amps)) > FourBarConstants.HomingStallCurrentAmps
            && Math.abs(velocity.getValue().in(RotationsPerSecond)) < FourBarConstants.HomingStallVelocityRotPerSec;
    }

    /** Test hook: sends a Motion Magic target WITHOUT clamping, to exercise the firmware soft limits. */
    void setUnclampedTargetForTesting(double rot) {
        targetRot = rot;
        leader.setControl(motionMagic.withPosition(rot));
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
        refreshSignals();

        double angleRot = position.getValue().in(Rotations);
        DogLog.log("FourBar/AngleRot", angleRot);
        DogLog.log("FourBar/TargetRot", targetRot);
        DogLog.log("FourBar/AtSetpoint", atSetpoint());
        DogLog.log("FourBar/VelocityRotPerSec", velocity.getValue().in(RotationsPerSecond));
        DogLog.log("FourBar/LeaderStatorCurrentAmps", leaderStatorCurrent.getValue().in(Amps));
        DogLog.log("FourBar/FollowerStatorCurrentAmps", followerStatorCurrent.getValue().in(Amps));
        DogLog.log("FourBar/LeaderSupplyCurrentAmps", leaderSupplyCurrent.getValue().in(Amps));
        DogLog.log("FourBar/FollowerSupplyCurrentAmps", followerSupplyCurrent.getValue().in(Amps));

        // AdvantageScope: pose of the pivoting floor relative to the robot. Rotation is about the pivot, which is
        // fixed (see advantagescope/Robot_BBot/config.json: components are zeroed at their CAD placement).
        Visualizer.setFloor(getFloorPose(angleRot));
        DogLog.log("FourBar/TargetLinkPose3d", getFloorPose(targetRot));
    }

    /**
     * Floor pose for a 4-bar angle: rotated about the pivot by how far the link has moved from the CAD pose angle.
     * The component model is zeroed at its CAD placement, so the pose translation is P - R*P to keep the pivot fixed.
     */
    static Pose3d getFloorPose(double angleRot) {
        double rotatedRad = Rotations.of(angleRot - FourBarConstants.CadPoseAngleRot).in(Radians);
        Translation3d pivot = new Translation3d(
            FourBarConstants.PivotXMeters, FourBarConstants.PivotYMeters, FourBarConstants.PivotZMeters);
        Rotation3d rotation = new Rotation3d(0, -rotatedRad, FourBarConstants.MountYawRadians);
        return new Pose3d(pivot.minus(pivot.rotateBy(rotation)), rotation);
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
            final double deltaTime = currentTime - lastSimTime;
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
        leader.close();
        follower.close();
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(this);
    }
}
