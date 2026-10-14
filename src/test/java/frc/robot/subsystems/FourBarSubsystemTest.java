package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningMode;
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
        TuningMode.safeMode.set(false);
        Tunables.resetAllToDefaults(); // tuned values live in NetworkTables; don't leak them into the next test
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

    private static double nt(String key) {
        return NetworkTableInstance.getDefault().getEntry("/Tuning/" + key).getDouble(Double.NaN);
    }

    private static void setNt(String key, double value) {
        NetworkTableInstance.getDefault().getEntry("/Tuning/" + key).setDouble(value);
    }

    private void tick() {
        Tunables.pollAll(); // apply anything edited live, like Robot.robotPeriodic does
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
    void bootSeedsDownStop(Load load) {
        load(load);
        assertEquals(FourBarConstants.DownStopRot, pos(), 1e-3); // boots against the down stop, floor flat
    }

    @ParameterizedTest @EnumSource(Load.class)
    void reachesEverySetpointInReasonableTime(Load load) {
        load(load);
        double[] setpoints = {
            FourBarConstants.RaisedRot, FourBarConstants.FlatRot,
            FourBarConstants.RaisedRot, FourBarConstants.FlatRot, 0.0, FourBarConstants.RaisedRot};
        for (double sp : setpoints) {
            goToAndSettle(sp);
        }
    }

    @ParameterizedTest @EnumSource(Load.class)
    void neverGoesPastSoftLimits(Load load) {
        load(load);
        goToAndSettle(FourBarConstants.RaisedRot);
        goToAndSettle(FourBarConstants.FlatRot);

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
            FourBarConstants.RaisedRot, FourBarConstants.FlatRot, 0.0,
            FourBarConstants.RaisedRot, FourBarConstants.FlatRot};
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

    /** The hopper-load estimate relies on the flat-floor holding current tracking the load, so check the calibration constants. */
    @ParameterizedTest @EnumSource(Load.class)
    void holdingCurrentWithTheFloorFlatTracksTheLoad(Load load) {
        load(load);
        goToAndSettle(FourBarConstants.RaisedRot);
        goToAndSettle(FourBarConstants.FlatRot);
        step(1.5);
        double amps = fourBar.getHoldCurrentAmps();
        double expected = load == Load.EMPTY ? FourBarConstants.EmptyHoldCurrentAmps : FourBarConstants.FullHoldCurrentAmps;
        System.out.printf("[FourBar] %s hold current with the floor flat: %.2f A (calibration constant %.2f A)%n", load, amps, expected);
        assertEquals(expected, amps, 0.25 * expected + 0.2, "flat-floor holding current does not match the load calibration");
    }

    @ParameterizedTest @EnumSource(Load.class)
    void rehomeResetsPositionToTheDownStop(Load load) {
        load(load);
        // Link physically sits raised but the encoder zero has been lost (reads somewhere else).
        goToAndSettle(FourBarConstants.RaisedRot);
        fourBar.corruptPositionForTesting(FourBarConstants.FlatRot + 0.1);
        step(0.1);
        assertEquals(FourBarConstants.FlatRot + 0.1, pos(), 0.01, "test setup: encoder should now be wrong");
        Command c = fourBar.rehome();
        c.schedule();
        double t = runUntil(() -> !c.isScheduled(), FourBarConstants.HomingTimeoutSeconds + 1);
        System.out.printf("[FourBar] %s re-home finished in %.2f s%n", load, t);
        assertTrue(t >= 0, "re-home never finished");
        assertTrue(t < FourBarConstants.HomingTimeoutSeconds - 0.1, "re-home ended by timeout, not stall detection");
        assertEquals(FourBarConstants.FlatRot, fourBar.getTarget().in(Rotations), 1e-9);
        assertTrue(runUntil(fourBar::atSetpoint, 4.0) >= 0, "did not settle flat after re-home: pos " + pos() + " sim "
            + fourBar.getSimAngleRot() + " vel " + fourBar.getVelocity() + " target " + fourBar.getTarget());
        assertEquals(FourBarConstants.FlatRot, fourBar.getSimAngleRot(), 0.002, "link not back down");
        assertEquals(FourBarConstants.FlatRot, pos(), 0.002, "position not reset");
        goToAndSettle(FourBarConstants.RaisedRot); // and it still works afterwards
    }

    @org.junit.jupiter.api.Test
    void floorLinkageGeometryIsConsistent() {
        // The short link's ground pivot must sit one short-link length from the floor's front pivot in the flat pose.
        double aFlat = Math.toRadians(Rotations.of(FourBarConstants.DownStopRot).in(edu.wpi.first.units.Units.Degrees));
        double hubX = FourBarConstants.FloorBackPivotX + FourBarConstants.LinkLengthMeters * Math.cos(aFlat);
        double hubZ = FourBarConstants.FloorBackPivotZ - FourBarConstants.LinkLengthMeters * Math.sin(aFlat);
        double sx = hubX + FourBarConstants.ShortLinkGroundDX;
        double sz = hubZ + FourBarConstants.ShortLinkGroundDZ;
        double shortLink = Math.hypot(FourBarConstants.FloorFrontPivotX - sx, FourBarConstants.FloorFrontPivotZ - sz);
        assertEquals(FourBarConstants.ShortLinkLengthMeters, shortLink, 0.005);
    }

    @org.junit.jupiter.api.Test
    void floorIsDrawnFlatAtTheDownStopAndTiltsUpAtTheUpStop() {
        var back = new edu.wpi.first.math.geometry.Translation3d(FourBarConstants.FloorBackPivotX, 0, FourBarConstants.FloorBackPivotZ);
        var front = new edu.wpi.first.math.geometry.Translation3d(FourBarConstants.FloorFrontPivotX, 0, FourBarConstants.FloorFrontPivotZ);

        // The CAD model is drawn flat, so at the down stop the floor is (almost) not moved at all. The ground pivot was read
        // off a picture, so allow a few millimetres and a fraction of a degree.
        var flat = FourBarSubsystem.getFloorPose(FourBarConstants.DownStopRot);
        assertEquals(0.0, flat.getTranslation().getNorm(), 0.003);
        assertEquals(0.0, flat.getRotation().getAngle(), Math.toRadians(0.5));

        // At the up stop the back of the floor is higher than the front: balls roll toward the intake.
        var raised = FourBarSubsystem.getFloorPose(FourBarConstants.UpStopRot);
        var backNow = back.rotateBy(raised.getRotation()).plus(raised.getTranslation());
        var frontNow = front.rotateBy(raised.getRotation()).plus(raised.getTranslation());
        assertTrue(backNow.getZ() > frontNow.getZ() + 0.05, "raised floor should slope down toward the front");
        assertTrue(backNow.getZ() > back.getZ() + 0.1, "the back of the floor should have risen");
        // The floor is rigid: the distance between its pivots does not change.
        assertEquals(back.getDistance(front), backNow.getDistance(frontNow), 1e-9);
        // About 40 degrees of tilt change (read off the side-view pictures), nose-down in WPILib's pitch convention.
        double tiltDegrees = Math.toDegrees(raised.getRotation().getY());
        assertTrue(tiltDegrees > 30 && tiltDegrees < 55, "tilt change was " + tiltDegrees);
    }

    // ------------------------------------------------------------------ live tuning

    @org.junit.jupiter.api.Test
    void editingAGainInNetworkTablesReachesTheMotor() {
        setNt("FourBar/Gains/kP", 123.0);
        step(0.2);
        assertEquals(123.0, fourBar.readKpFromMotorForTesting(), 1e-6, "the motor should have the tuned kP");
    }

    @org.junit.jupiter.api.Test
    void outOfRangeEditsAreClampedBeforeTheyReachTheMotor() {
        setNt("FourBar/Limits/StatorCurrentAmps", 5000.0);
        step(0.2);
        assertEquals(80.0, nt("FourBar/Limits/StatorCurrentAmps"), 1e-9, "the entry itself is pulled back into range");
        assertEquals(80.0, fourBar.readStatorLimitFromMotorForTesting(), 1e-6);
    }

    @org.junit.jupiter.api.Test
    void safeModeCapsCurrentAndSpeedThenRestoresThem() {
        double cruise = FourBarConstants.CruiseVelocityRotPerSec;
        step(0.2);
        assertEquals(FourBarConstants.StatorCurrentLimitAmps, fourBar.readStatorLimitFromMotorForTesting(), 1e-6);
        assertEquals(cruise, fourBar.readCruiseVelocityFromMotorForTesting(), 1e-6);

        TuningMode.safeMode.set(true);
        step(0.2);
        assertEquals(TuningMode.safeCurrentLimitAmps.get(), fourBar.readStatorLimitFromMotorForTesting(), 1e-6);
        assertEquals(cruise * TuningMode.safeSpeedScale.get(), fourBar.readCruiseVelocityFromMotorForTesting(), 1e-6);

        TuningMode.safeMode.set(false);
        step(0.2);
        assertEquals(FourBarConstants.StatorCurrentLimitAmps, fourBar.readStatorLimitFromMotorForTesting(), 1e-6);
        assertEquals(cruise, fourBar.readCruiseVelocityFromMotorForTesting(), 1e-6);
    }

    @ParameterizedTest @EnumSource(Load.class)
    void everyMoveIsMeasuredAndPublishedWithAdvice(Load load) {
        load(load);
        setNt("FourBar/Result/MovesMeasured", 0); // the count restarts with each new subsystem; clear the old value
        frc.robot.tuning.TuningTelemetry.forgetForTesting(); // telemetry only writes changes, so make sure the next count is written
        double before = nt("FourBar/Result/MovesMeasured");
        goToAndSettle(FourBarConstants.RaisedRot);
        step(0.6); // let the metrics see it hold inside the tolerance
        double after = nt("FourBar/Result/MovesMeasured");
        assertTrue(after > before, "a finished move should be counted");
        assertTrue(nt("FourBar/Result/SettleSeconds") > 0.2, "settle time should be a real number");
        assertTrue(nt("FourBar/Result/DistanceRot") > 0.1);
        System.out.printf("[FourBar] %s move: settle %.2f s, overshoot %.5f rot, peak %.1f A%n", load,
            nt("FourBar/Result/SettleSeconds"), nt("FourBar/Result/OvershootRot"), nt("FourBar/Result/PeakStatorCurrentAmps"));
        String advice = NetworkTableInstance.getDefault().getEntry("/Tuning/FourBar/Result/Advice").getString("");
        assertFalse(advice.isEmpty());
    }

    @ParameterizedTest @EnumSource(Load.class)
    void suggestedKgTracksTheLoad(Load load) {
        load(load);
        goToAndSettle(FourBarConstants.RaisedRot);
        goToAndSettle(FourBarConstants.FlatRot);
        step(5.0); // sit still long enough for the estimator to average
        double suggested = nt("FourBar/Suggested/kG");
        // Physical need at the flat angle: voltage = (per-motor holding current) * motor resistance (~0.0328 ohm).
        double expected = (load == Load.EMPTY ? FourBarConstants.EmptyHoldCurrentAmps : FourBarConstants.FullHoldCurrentAmps) * 0.0328;
        System.out.printf("[FourBar] %s suggested kG %.3f V (expected about %.3f V)%n", load, suggested, expected);
        assertFalse(Double.isNaN(suggested), "no kG suggestion was published");
        assertEquals(expected, suggested, 0.4 * expected + 0.03);
    }

    @org.junit.jupiter.api.Test
    void applyingTheSuggestedKgButtonSetsTheTunable() {
        load(Load.LOADED);
        goToAndSettle(FourBarConstants.RaisedRot);
        goToAndSettle(FourBarConstants.FlatRot);
        step(5.0);
        double suggested = nt("FourBar/Suggested/kG");
        NetworkTableInstance.getDefault().getEntry("/Tuning/FourBar/Actions/ApplySuggestedKG").setBoolean(true);
        step(0.3);
        assertEquals(suggested, nt("FourBar/Gains/kG"), 0.05, "the button should copy the suggestion into kG");
        assertFalse(NetworkTableInstance.getDefault().getEntry("/Tuning/FourBar/Actions/ApplySuggestedKG").getBoolean(true));
    }
}
