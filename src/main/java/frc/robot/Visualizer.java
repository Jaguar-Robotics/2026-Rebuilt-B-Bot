// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Collects the 3D component poses of the articulated parts so AdvantageScope can animate the robot model.
 * Each subsystem sets its own pose in periodic(); Robot logs the array once per loop.
 * Order must match the component models in advantagescope/Robot_BBot (model_0, model_1, model_2).
 */
public final class Visualizer {
    private static Pose3d floor = new Pose3d();        // model_0: pivoting hopper floor (FourBarSubsystem)
    private static Pose3d intake = new Pose3d();       // model_1: intake slapdown (IntakePivotSubsystem)
    private static Pose3d hopperSlide = new Pose3d();  // model_2: slidy hopper (TODO: not driven by anything yet)

    private Visualizer() {}

    public static void setFloor(Pose3d pose) {
        floor = pose;
    }

    public static void setIntake(Pose3d pose) {
        intake = pose;
    }

    public static void setHopperSlide(Pose3d pose) {
        hopperSlide = pose;
    }

    private static final Pose3d[] poses = new Pose3d[3]; // reused every loop instead of allocating a new array

    public static void log() {
        poses[0] = floor;
        poses[1] = intake;
        poses[2] = hopperSlide;
        DogLog.log("Visualizer/ComponentPoses", poses);
    }
}
