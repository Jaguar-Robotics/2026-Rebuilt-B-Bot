// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class JoystickDriveUtil {

  private JoystickDriveUtil() {}

  // Polar shaping: deadband and square the magnitude, preserve direction.
  // Per-axis deadband / squaring would bias diagonals toward cardinal axes.
  public static Translation2d getLinearVelocityFromJoysticks(double x, double y, double deadband) {
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    if (magnitude == 0.0) {
      return Translation2d.kZero;
    }
    Rotation2d direction = new Rotation2d(x, y);
    double shapedMagnitude = magnitude * magnitude;
    return new Pose2d(Translation2d.kZero, direction)
        .transformBy(new Transform2d(shapedMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static double getOmegaFromJoysticks(double omegaInput, double deadband) {
    double omega = MathUtil.applyDeadband(omegaInput, deadband);
    return omega * omega * Math.signum(omega);
  }

  public static double scaleAxis(double input) {
    double abs = Math.abs(input);
    if (abs == 0.0) return 0.0;
    abs = Math.min(abs, 1.0);
    double scaled = Math.pow(Math.sin((Math.PI / 2.0) * abs), 2); // no offset
    return Math.copySign(scaled, input);
}
}
