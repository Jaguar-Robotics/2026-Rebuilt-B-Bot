package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.function.BooleanSupplier;

/**
 * Runs the real Robot class in simulation, in real time, with simulated driver-station and controller input, so a test can
 * press buttons and watch what the whole robot does. One instance per test JVM (the robot's devices are process-wide).
 */
final class RobotSim {
    static final int kRightBumper = SimAwareXboxController.SIM_BUTTON_RIGHT_BUMPER;
    static final int kLeftBumper = SimAwareXboxController.SIM_BUTTON_LEFT_BUMPER;
    static final int kBack = SimAwareXboxController.SIM_BUTTON_BACK;
    static final int kA = SimAwareXboxController.SIM_BUTTON_A;
    // Sim controller axis indices (see SimAwareXboxController): triggers rest at -1 and the two are swapped in the sim GUI.
    static final int kSimLeftTrigger = SimAwareXboxController.SIM_LEFT_TRIGGER;
    static final int kSimRightTrigger = SimAwareXboxController.SIM_RIGHT_TRIGGER;

    final Robot robot;
    final RobotContainer rc;

    /** Loops in which the intake was up in the floor's way while the floor was raised (should always stay 0). */
    int interlockViolations = 0;
    String firstInterlockViolation = "";

    private static RobotSim instance;

    /**
     * The one simulated robot for this test JVM (its motors and singletons exist once per process), brought back to a clean
     * starting state: disabled, nothing pressed, everything stowed, defaults restored, robot parked at the sim start pose.
     */
    static synchronized RobotSim get() {
        return get(true);
    }

    /** @param blueAlliance only used the first time (the alliance is fixed for the test JVM) */
    static synchronized RobotSim get(boolean blueAlliance) {
        if (instance == null) instance = new RobotSim(blueAlliance);
        instance.reset();
        return instance;
    }

    private void reset() {
        interlockViolations = 0;
        firstInterlockViolation = "";
        releaseEverything();
        enableTeleop();
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().cancelAll();
        frc.robot.tuning.Tunables.resetAllToDefaults();
        frc.robot.tuning.TuningMode.safeMode.set(false);
        if (rc.getFuelSimulation() != null) {
            rc.getFuelSimulation().setHopperCountForTesting(0);
            rc.getFuelSimulation().clearFieldForTesting();
        }
        run(0.3); // lets released triggers fire their idle transitions first
        rc.superStructure.forceStowIgnoringHopper(); // back to the boot state whatever the last test left in the hopper
        rc.drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(Constants.SimConstants.StartXMeters,
            Constants.SimConstants.StartYMeters, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(Constants.SimConstants.StartHeadingDegrees)));
        runUntil(() -> rc.fourBarHandler.getCurrentState() == frc.robot.Handlers.FourBarHandler.FourBarState.FLAT, 5);
        run(0.7);
    }

    RobotSim(boolean blueAlliance) {
        if (!HAL.initialize(500, 0)) throw new IllegalStateException("HAL failed to initialize");
        DriverStationSim.setAllianceStationId(blueAlliance ? AllianceStationID.Blue1 : AllianceStationID.Red1);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setJoystickAxisCount(0, 6);
        DriverStationSim.setJoystickButtonCount(0, 27); // the Xbox Wireless Controller reports 27
        DriverStationSim.setJoystickPOVCount(0, 1);
        releaseEverything();
        robot = new Robot();
        rc = robot.getRobotContainer();
    }

    void releaseEverything() {
        for (int i = 0; i < 4; i++) DriverStationSim.setJoystickAxis(0, i, 0.0);
        DriverStationSim.setJoystickAxis(0, kSimLeftTrigger, -1.0);
        DriverStationSim.setJoystickAxis(0, kSimRightTrigger, -1.0);
        for (int b = 1; b <= 27; b++) DriverStationSim.setJoystickButton(0, b, false);
        DriverStationSim.setJoystickPOV(0, 0, -1);
    }

    void enableTeleop() {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(true);
    }

    /** Picks the auto in the dashboard chooser and starts autonomous, like the field would. */
    void startAuto(String autoName) {
        edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry("/SmartDashboard/Auto Mode/selected").setString(autoName);
        run(0.3);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        robot.autonomousInit();
    }

    void endAuto() {
        robot.autonomousExit();
        enableTeleop();
        robot.teleopInit();
    }

    void disable() {
        DriverStationSim.setEnabled(false);
    }

    void pressDpadDown() { DriverStationSim.setJoystickPOV(0, 0, 180); }
    void releaseDpad() { DriverStationSim.setJoystickPOV(0, 0, -1); }
    void pressButton(int button) { DriverStationSim.setJoystickButton(0, button, true); }
    void releaseButton(int button) { DriverStationSim.setJoystickButton(0, button, false); }
    void holdLeftTrigger(boolean held) { DriverStationSim.setJoystickAxis(0, kSimLeftTrigger, held ? 1.0 : -1.0); }
    void holdRightTrigger(boolean held) { DriverStationSim.setJoystickAxis(0, kSimRightTrigger, held ? 1.0 : -1.0); }

    /** Left stick / right stick like a driver: forward and left are positive, counter-clockwise is positive. */
    void drive(double forward, double left, double counterClockwise) {
        DriverStationSim.setJoystickAxis(0, 0, -left);
        DriverStationSim.setJoystickAxis(0, 1, -forward);
        DriverStationSim.setJoystickAxis(0, 2, -counterClockwise);
    }

    static double nt(String key) {
        return edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry(key).getDouble(Double.NaN);
    }

    static String ntString(String key) {
        return edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry(key).getString("");
    }

    double intakeAngleDeg() { return nt("/Tuning/IntakePivot/Live/PositionDeg"); }
    double floorRot() { return nt("/Tuning/FourBar/Live/PositionRot"); }

    /** One 20 ms robot loop, like TimedRobot would run. */
    void loop() {
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        robot.robotPeriodic();
        robot.simulationPeriodic();
        checkInterlock();
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.updateValues(); // what TimedRobot's loop does; makes dashboard buttons work
        Timer.delay(0.02);
    }

    /** The collision condition from the CAD: intake past the clear zone (with a little slack) while the floor is not flat. */
    private void checkInterlock() {
        double intakeDeg = frc.robot.Handlers.IntakePivotHandler.getInstance().getPositionDeg();
        double floorRot = rc.fourBarHandler.getAngleRot();
        boolean intakeInTheWay = intakeDeg > Constants.InterlockConstants.IntakeClearBelowDeg + 5.0;
        boolean floorUp = floorRot > Constants.InterlockConstants.FloorCountsAsFlatBelowRot + 0.006;
        if (intakeInTheWay && floorUp && frc.robot.Handlers.IntakePivotHandler.getInstance() != null && interlockEnabled()) {
            if (interlockViolations == 0) firstInterlockViolation = String.format("intake %.1f deg with floor at %.4f rot", intakeDeg, floorRot);
            interlockViolations++;
        }
    }

    private boolean interlockEnabled() {
        return edu.wpi.first.networktables.NetworkTableInstance.getDefault().getEntry("/Tuning/Interlock/Enabled").getBoolean(true);
    }

    /** For @AfterEach in every full-robot test: whatever the test did, the intake and the raised floor must never have been in each other's way. */
    static void assertNoViolationsNow() {
        if (instance != null) instance.assertNoInterlockViolations();
    }

    void assertNoInterlockViolations() {
        org.junit.jupiter.api.Assertions.assertEquals(0, interlockViolations,
            "the intake and the raised floor were in each other's way (" + firstInterlockViolation + ")");
    }

    void run(double seconds) {
        double end = Timer.getFPGATimestamp() + seconds;
        while (Timer.getFPGATimestamp() < end) loop();
    }

    /** Runs until the condition is true or the time is up. @return seconds taken, or -1 on timeout. */
    double runUntil(BooleanSupplier done, double timeoutSeconds) {
        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < timeoutSeconds) {
            loop();
            if (done.getAsBoolean()) return Timer.getFPGATimestamp() - start;
        }
        return -1;
    }
}
