package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FourBarConstants;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Runs the real TalonFX sim (TalonFXSimState) against a WPILib SingleJointedArmSim, in real time.
 * Everything is derived from {@link FourBarConstants}, so these should keep passing after the real values go in.
 */
class FourBarSubsystemTest {
    private static final double kLoopSeconds = 0.02;
    private static final double kSettleTimeoutSeconds = 3.0;
    /** Max allowed overshoot past a target, in mechanism rotations (0.001 rot = 0.36 deg). */
    private static final double kMaxOvershootRot = 0.001;
    /**
     * Soft limits sit SoftLimitMarginRot inside the hard stops. That is only ~6 ms of travel at cruise speed, so when the
     * firmware is pushed past a limit (clamp bypassed) the link can coast across the margin, more so on a loaded machine.
     * The backstop guarantee is therefore "never past the hard stop"; normal moves are clamped and never reach the limits.
     */
    private static final double kLimitSlackRot = FourBarConstants.SoftLimitMarginRot + 0.0005; // + encoder quantization
    /** Holding the loaded link at horizontal must stay well clear of the stator current limit. */
    private static final double kMaxHoldCurrentFraction = 0.5;

    private static final double kLow = FourBarConstants.DownStopRot + FourBarConstants.SoftLimitMarginRot;
    private static final double kHigh = FourBarConstants.UpStopRot - FourBarConstants.SoftLimitMarginRot;

    enum Load {
        EMPTY(FourBarConstants.MovingMassEmptyKg),
        LOADED(FourBarConstants.MovingMassLoadedKg);

        final double massKg;

        Load(double massKg) {
            this.massKg = massKg;
        }
    }

    private void load(Load load) {
        fourBar.setSimMassKg(load.massKg);
    }

    private FourBarSubsystem fourBar;
    private double minSeen;
    private double maxSeen;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        fourBar = new FourBarSubsystem();
        minSeen = Double.POSITIVE_INFINITY;
        maxSeen = Double.NEGATIVE_INFINITY;
        step(0.5); // let the sim and signals come up
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        fourBar.stop(); // sim'd TalonFXs outlive the Java object and keep their last control request
        Timer.delay(0.1);
        fourBar.close();
    }

    private double pos() {
        return fourBar.getPosition().in(Rotations);
    }

    /** One scheduler loop at real time, tracking the extremes of the measured angle. */
    private void step(double seconds) {
        double end = Timer.getFPGATimestamp() + seconds;
        while (Timer.getFPGATimestamp() < end) {
            tick();
        }
    }

    private void tick() {
        CommandScheduler.getInstance().run();
        minSeen = Math.min(minSeen, pos());
        maxSeen = Math.max(maxSeen, pos());
        Timer.delay(kLoopSeconds);
    }

    /** Runs until the condition holds or the timeout elapses. @return elapsed seconds, or -1 on timeout. */
    private double runUntil(BooleanSupplier done, double timeoutSeconds) {
        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < timeoutSeconds) {
            tick();
            if (done.getAsBoolean()) return Timer.getFPGATimestamp() - start;
        }
        return -1;
    }

    private boolean isStopped() {
        return Math.abs(fourBar.getVelocity().in(Rotations.per(edu.wpi.first.units.Units.Second))) < 0.005;
    }

    private void goToAndSettle(double rot) {
        Command c = fourBar.goTo(Rotations.of(rot));
        c.schedule();
        double t = runUntil(() -> !c.isScheduled(), kSettleTimeoutSeconds);
        System.out.printf("[FourBar] move to %.3f settled in %.2f s%n", rot, t);
        assertTrue(t >= 0, "did not reach " + rot + " within " + kSettleTimeoutSeconds + " s, at " + pos());
        assertEquals(rot, pos(), FourBarConstants.PositionToleranceRot);
        assertTrue(fourBar.atSetpoint());
    }

    @ParameterizedTest @EnumSource(Load.class)
    void bootSeedsUpStop(Load load) {
        load(load);
        assertEquals(FourBarConstants.UpStopRot, pos(), 1e-3);
    }

    @ParameterizedTest @EnumSource(Load.class)
    void reachesEverySetpointInReasonableTime(Load load) {
        load(load);
        double[] setpoints = {
            FourBarConstants.IntakeRot, FourBarConstants.StowRot,
            FourBarConstants.IntakeRot, FourBarConstants.StowRot, 0.0, FourBarConstants.StowRot};
        for (double sp : setpoints) {
            goToAndSettle(sp);
        }
    }

    @ParameterizedTest @EnumSource(Load.class)
    void neverGoesPastSoftLimits(Load load) {
        load(load);
        goToAndSettle(FourBarConstants.IntakeRot);
        goToAndSettle(FourBarConstants.StowRot);

        // Ask for far past each end, bypassing the subsystem's own clamp so the TalonFX soft limits are what stops it.
        // The limit is checked once the link has come to rest against it (not after a fixed time, so a slow or loaded CPU
        // does not matter). After that the sim (no friction) lets a loaded link creep down under gravity with the output
        // cut, so later drift is only reported, not asserted.
        minSeen = Double.POSITIVE_INFINITY;
        maxSeen = Double.NEGATIVE_INFINITY;
        fourBar.setUnclampedTargetForTesting(kLow - 0.5);
        assertTrue(runUntil(() -> pos() < kLow + 0.003 && isStopped(), 6.0) >= 0, "never came to rest at the low limit");
        double minAtLimit = minSeen;
        step(1.5);
        System.out.printf("[FourBar] %s pushed past low limit: min %.5f at rest, %.5f after 1.5 s more (soft %.5f, hard %.5f)%n",
            load, minAtLimit, minSeen, kLow, FourBarConstants.DownStopRot);
        assertTrue(minAtLimit >= kLow - kLimitSlackRot, "went below the low soft limit: " + minAtLimit);

        fourBar.setUnclampedTargetForTesting(kHigh + 0.5);
        assertTrue(runUntil(() -> pos() > kHigh - 0.003 && isStopped(), 6.0) >= 0, "never came to rest at the high limit");
        double maxAtLimit = maxSeen;
        step(1.5);
        System.out.printf("[FourBar] %s pushed past high limit: max %.5f at rest, %.5f after 1.5 s more (soft %.5f, hard %.5f)%n",
            load, maxAtLimit, maxSeen, kHigh, FourBarConstants.UpStopRot);
        assertTrue(maxAtLimit <= kHigh + kLimitSlackRot, "went above the high soft limit: " + maxAtLimit);

        // Also via the clamped API.
        fourBar.setTarget(Rotations.of(kLow - 1));
        assertEquals(kLow, fourBar.getTarget().in(Rotations), 1e-9);
        fourBar.setTarget(Rotations.of(kHigh + 1));
        assertEquals(kHigh, fourBar.getTarget().in(Rotations), 1e-9);
    }

    @ParameterizedTest @EnumSource(Load.class)
    void doesNotOvershoot(Load load) {
        load(load);
        double[] setpoints = {
            FourBarConstants.IntakeRot, FourBarConstants.StowRot, 0.0,
            FourBarConstants.IntakeRot, FourBarConstants.StowRot};
        double from = pos();
        for (double sp : setpoints) {
            double dir = Math.signum(sp - from);
            minSeen = Double.POSITIVE_INFINITY;
            maxSeen = Double.NEGATIVE_INFINITY;
            goToAndSettle(sp);
            step(0.5); // keep watching after the command ends
            double overshoot = dir > 0 ? maxSeen - sp : sp - minSeen;
            System.out.printf("[FourBar] %s %.3f -> %.3f overshoot %.5f rot%n", load, from, sp, overshoot);
            assertTrue(overshoot <= kMaxOvershootRot,
                "overshoot " + overshoot + " rot going " + from + " -> " + sp);
            from = sp;
        }
    }

    @ParameterizedTest @EnumSource(Load.class)
    void holdsHorizontalAgainstGravity(Load load) {
        load(load);
        goToAndSettle(0.0);
        step(2.0);
        System.out.printf("[FourBar] %s hold at horizontal: pos %.5f%n", load, pos());
        assertEquals(0.0, pos(), 0.002, "drifted off horizontal");
        assertEquals(0.0, fourBar.getSimAngleRot(), 0.002, "sim link is not at horizontal");
        assertTrue(Math.abs(fourBar.getVelocity().in(edu.wpi.first.units.Units.RotationsPerSecond)) < 0.01);
        // Gravity at horizontal needs real holding current, and the opposed follower must carry its share
        // (equal magnitude, opposite sign in each motor's own frame), with plenty of headroom to the limit.
        double lead = fourBar.getLeaderStatorCurrent().in(edu.wpi.first.units.Units.Amps);
        double foll = fourBar.getFollowerStatorCurrent().in(edu.wpi.first.units.Units.Amps);
        System.out.printf("[FourBar] %s hold at horizontal: leader %.2f A, follower %.2f A (limit %.0f A)%n",
            load, lead, foll, FourBarConstants.StatorCurrentLimitAmps);
        assertTrue(Math.abs(lead) > 0.5, "leader not holding against gravity");
        assertEquals(Math.abs(lead), Math.abs(foll), 0.25 * Math.abs(lead) + 0.2, "follower not sharing the load");
        assertTrue(Math.abs(lead) < kMaxHoldCurrentFraction * FourBarConstants.StatorCurrentLimitAmps
            && Math.abs(foll) < kMaxHoldCurrentFraction * FourBarConstants.StatorCurrentLimitAmps,
            "holding current too close to the limit");
    }

    @ParameterizedTest @EnumSource(Load.class)
    void rehomeResetsPositionToUpStop(Load load) {
        load(load);
        // Link physically sits at intake but the encoder zero has been lost (reads somewhere else).
        goToAndSettle(FourBarConstants.IntakeRot);
        fourBar.corruptPositionForTesting(FourBarConstants.StowRot - 0.1);
        step(0.1);
        assertEquals(FourBarConstants.StowRot - 0.1, pos(), 0.01, "test setup: encoder should now be wrong");
        Command c = fourBar.rehome();
        c.schedule();
        double t = runUntil(() -> !c.isScheduled(), FourBarConstants.HomingTimeoutSeconds + 1);
        System.out.printf("[FourBar] %s re-home finished in %.2f s%n", load, t);
        assertTrue(t >= 0, "re-home never finished");
        assertTrue(t < FourBarConstants.HomingTimeoutSeconds - 0.1, "re-home ended by timeout, not stall detection");
        assertEquals(FourBarConstants.StowRot, fourBar.getTarget().in(Rotations), 1e-9);
        assertTrue(runUntil(fourBar::atSetpoint, 4.0) >= 0, "did not settle at stow after re-home: pos " + pos() + " sim "
            + fourBar.getSimAngleRot() + " vel " + fourBar.getVelocity() + " target " + fourBar.getTarget());
        assertEquals(FourBarConstants.StowRot, fourBar.getSimAngleRot(), 0.002, "link not back at stow");
        assertEquals(FourBarConstants.StowRot, pos(), 0.002, "position not reset");
        goToAndSettle(FourBarConstants.IntakeRot); // and it still works afterwards
    }

    @org.junit.jupiter.api.Test
    void floorPoseRotatesAboutThePivot() {
        var pivot = new edu.wpi.first.math.geometry.Translation3d(
            FourBarConstants.PivotXMeters, FourBarConstants.PivotYMeters, FourBarConstants.PivotZMeters);

        // At the CAD pose angle the model is not moved at all.
        var atCad = FourBarSubsystem.getFloorPose(FourBarConstants.CadPoseAngleRot);
        assertEquals(0.0, atCad.getTranslation().getNorm(), 1e-9);
        assertEquals(0.0, atCad.getRotation().getAngle(), 1e-9);

        // Lowering the link by a quarter rotation pitches the floor nose-down 90 degrees about the front hinge, which
        // lifts the back so balls roll toward the intake. The hinge point itself must not move.
        var lowered = FourBarSubsystem.getFloorPose(FourBarConstants.CadPoseAngleRot - 0.25);
        assertEquals(Math.PI / 2, lowered.getRotation().getY(), 1e-9);
        var movedPivot = pivot.rotateBy(lowered.getRotation()).plus(lowered.getTranslation());
        assertEquals(0.0, movedPivot.getDistance(pivot), 1e-9);
    }
}
