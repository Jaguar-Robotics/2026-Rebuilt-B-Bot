// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Handlers.FourBarHandler;
import frc.robot.Handlers.SuperStructure;
import frc.robot.Handlers.SuperStructure.SuperStructureState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.tuning.TunableNumber;
import frc.robot.tuning.Tunables;
import frc.robot.tuning.TuningTelemetry;
import frc.robot.util.FuelHopper;
import frc.robot.util.FuelSim;
import java.util.Random;

/**
 * Simulation only. Puts fuel on the field (FuelSim), lets the deployed intake pick it up until the hopper is full,
 * releases it again on outtake, and feeds the hopper's weight into the 4-bar's simulated load.
 *
 * Fuel positions are published at /Fuel Simulation/Fuels (view them as "Ball" objects in AdvantageScope's 3D field).
 */
public class FuelSimulation {
    private static final double kLoopSeconds = 0.02;

    private final FuelSim fuelSim = new FuelSim();
    private final FuelHopper hopper = new FuelHopper(
        FuelConstants.HopperCapacity,
        Constants.FourBarConstants.MovingMassEmptyKg,
        Constants.FourBarConstants.MovingMassLoadedKg);
    private final CommandSwerveDrivetrain drivetrain;
    private final SuperStructure superStructure;
    private final FourBarHandler fourBarHandler;
    private final Random random = new Random();

    // live-tunable (/Tuning/FuelSim/...)
    private final TunableNumber capacity = Tunables.number("FuelSim/HopperCapacity", FuelConstants.HopperCapacity, 1, 1000, "FuelConstants.HopperCapacity", v -> applyTuning());
    private final TunableNumber robotWidth = Tunables.number("FuelSim/Robot/WidthMeters", FuelConstants.RobotWidthMeters, 0.3, 2, "FuelConstants.RobotWidthMeters", v -> applyTuning());
    private final TunableNumber robotLength = Tunables.number("FuelSim/Robot/LengthMeters", FuelConstants.RobotLengthMeters, 0.3, 2, "FuelConstants.RobotLengthMeters", v -> applyTuning());
    private final TunableNumber bumperHeight = Tunables.number("FuelSim/Robot/BumperHeightMeters", FuelConstants.BumperHeightMeters, 0.05, 0.5, "FuelConstants.BumperHeightMeters", v -> applyTuning());
    private final TunableNumber intakeMinX = Tunables.number("FuelSim/Intake/MinX", FuelConstants.IntakeMinX, -1, 2, "FuelConstants.IntakeMinX", v -> applyTuning());
    private final TunableNumber intakeMaxX = Tunables.number("FuelSim/Intake/MaxX", FuelConstants.IntakeMaxX, -1, 2, "FuelConstants.IntakeMaxX", v -> applyTuning());
    private final TunableNumber intakeMinY = Tunables.number("FuelSim/Intake/MinY", FuelConstants.IntakeMinY, -1.5, 1.5, "FuelConstants.IntakeMinY", v -> applyTuning());
    private final TunableNumber intakeMaxY = Tunables.number("FuelSim/Intake/MaxY", FuelConstants.IntakeMaxY, -1.5, 1.5, "FuelConstants.IntakeMaxY", v -> applyTuning());
    private final TunableNumber outtakeRate = Tunables.number("FuelSim/Outtake/RatePerSecond", FuelConstants.OuttakeRatePerSecond, 0.5, 200, "FuelConstants.OuttakeRatePerSecond");
    private final TunableNumber outtakeSpawnX = Tunables.number("FuelSim/Outtake/SpawnX", FuelConstants.OuttakeSpawnX, 0, 2, "FuelConstants.OuttakeSpawnX");
    private final TunableNumber outtakeHalfWidth = Tunables.number("FuelSim/Outtake/SpawnHalfWidthY", FuelConstants.OuttakeSpawnHalfWidthY, 0, 1, "FuelConstants.OuttakeSpawnHalfWidthY");
    private final TunableNumber outtakeZ = Tunables.number("FuelSim/Outtake/SpawnZ", FuelConstants.OuttakeSpawnZ, 0.1, 1, "FuelConstants.OuttakeSpawnZ");
    private final TunableNumber outtakeSpeed = Tunables.number("FuelSim/Outtake/SpeedMetersPerSecond", FuelConstants.OuttakeSpeedMetersPerSecond, 0.2, 15, "FuelConstants.OuttakeSpeedMetersPerSecond");

    private double outtakeBudget = 0.0;
    private double lastMassKg = Double.NaN;

    public FuelSimulation(CommandSwerveDrivetrain drivetrain, SuperStructure superStructure, FourBarHandler fourBarHandler) {
        this.drivetrain = drivetrain;
        this.superStructure = superStructure;
        this.fourBarHandler = fourBarHandler;

        fuelSim.spawnStartingFuel();
        fuelSim.enableAirResistance();
        fuelSim.setSubticks(3); // fewer physics steps per loop keeps the sim light on a busy machine
        hopper.setCapacity((int) Math.round(capacity.get()));
        fuelSim.registerRobot(
            robotWidth.get(),
            robotLength.get(),
            bumperHeight.get(),
            this::getPose,
            this::getFieldSpeeds);
        // Picks up fuel only while the intake is deployed and running (INTAKE state) and there is room.
        fuelSim.registerIntake(
            intakeMinX.get(),
            intakeMaxX.get(),
            intakeMinY.get(),
            intakeMaxY.get(),
            () -> superStructure.getCurrentState() == SuperStructureState.INTAKE && !hopper.isFull(),
            hopper::intake);
        fuelSim.start();
    }

    /** Pushes the tuned sizes and pickup box into the fuel simulation. */
    private void applyTuning() {
        hopper.setCapacity((int) Math.round(capacity.get()));
        fuelSim.registerRobot(robotWidth.get(), robotLength.get(), bumperHeight.get(), this::getPose, this::getFieldSpeeds);
        fuelSim.setIntakeBounds(0, intakeMinX.get(), intakeMaxX.get(), intakeMinY.get(), intakeMaxY.get());
    }

    private Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    private ChassisSpeeds getFieldSpeeds() {
        var state = drivetrain.getState();
        return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    /** Call from simulationPeriodic(). */
    public void update() {
        fuelSim.updateSim();

        if (superStructure.getCurrentState() == SuperStructureState.POOP) {
            outtakeBudget += outtakeRate.get() * kLoopSeconds;
            while (outtakeBudget >= 1.0 && hopper.outtake()) {
                outtakeBudget -= 1.0;
                spawnOuttakenFuel();
            }
            if (hopper.isEmpty()) outtakeBudget = 0.0;
        } else {
            outtakeBudget = 0.0;
        }

        // The load on the 4-bar follows how full the hopper is.
        double massKg = hopper.getMassKg();
        if (massKg != lastMassKg) {
            fourBarHandler.setSimMassKg(massKg);
            lastMassKg = massKg;
        }

        DogLog.log("Fuel/HopperCount", hopper.getCount());
        DogLog.log("Fuel/HopperFull", hopper.isFull());
        DogLog.log("Fuel/HopperMassKg", massKg);
        TuningTelemetry.putLive("FuelSim/Live/HopperCount", hopper.getCount());
        TuningTelemetry.putLive("FuelSim/Live/HopperFull", hopper.isFull());
    }

    private void spawnOuttakenFuel() {
        Pose2d pose = getPose();
        double y = (random.nextDouble() * 2.0 - 1.0) * outtakeHalfWidth.get();
        Translation2d offset = new Translation2d(outtakeSpawnX.get(), y).rotateBy(pose.getRotation());
        Translation2d position = pose.getTranslation().plus(offset);
        Translation2d exitVelocity = new Translation2d(outtakeSpeed.get(), 0.0).rotateBy(pose.getRotation());
        ChassisSpeeds robotSpeeds = getFieldSpeeds();
        fuelSim.spawnFuel(
            new Translation3d(position.getX(), position.getY(), outtakeZ.get()),
            new Translation3d(
                exitVelocity.getX() + robotSpeeds.vxMetersPerSecond,
                exitVelocity.getY() + robotSpeeds.vyMetersPerSecond,
                0.5));
    }

    /** Test hooks. */
    void clearFieldForTesting() {
        fuelSim.clearFuel();
    }

    void spawnFuelForTesting(double xMeters, double yMeters) {
        fuelSim.spawnFuel(new Translation3d(xMeters, yMeters, 0.075), new Translation3d());
    }

    int getFuelOnFieldForTesting() {
        return fuelSim.getFuelCount();
    }

    void setHopperCountForTesting(int count) {
        hopper.clear();
        for (int i = 0; i < count; i++) hopper.intake();
    }

    public int getHopperCount() {
        return hopper.getCount();
    }
}
