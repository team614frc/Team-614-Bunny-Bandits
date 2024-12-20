// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Measure<Mass> ROBOT_MASS = Pounds.of(125);
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS.in(Kilogram));
  public static final Measure<Time> LOOP_TIME =
      Seconds.of(0.13); // s, 20ms + 110ms sprk max velocity lag
  public static final Measure<Velocity<Distance>> MAX_SPEED = FeetPerSecond.of(14.5);

  public static final class PivotConstants {
    public static final double PIVOT_kP = 0.064;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;
    public static final double PIVOT_kS = 0;
    public static final double PIVOT_kG = 0;
    public static final double PIVOT_kV = 0;
    public static final double PIVOT_kA = 0;
    public static final int PIVOT_MOTOR = 20;
    public static final Measure<Velocity<Angle>> PIVOT_MAX_VEL = DegreesPerSecond.of(10000);
    public static final Measure<Velocity<Velocity<Angle>>> PIVOT_MAX_ACCEL =
        DegreesPerSecond.per(Second).of(10000);
    public static final Measure<Mass> PIVOT_WEIGHT = Kilogram.of(9.55);
    public static final double PIVOT_MOTOR_SPEED = 0.1;
    public static final double PIVOT_REST_SPEED = 0;
    public static final Measure<Current> MOTOR_CURRENT_LIMIT = Amp.of(40);
    public static final double GEAR_RATIO = 60;
    public static final Measure<Angle> PIVOT_MAX = Degrees.of(20);
    public static final Measure<Angle> PIVOT_MIN = Degrees.of(105);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled.
    public static final Measure<Time> WHEEL_LOCK_TIME = Seconds.of(10);
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Joystick deadband.
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR = 19;
    public static final Measure<Current> MOTOR_CURRENT_LIMIT = Amp.of(40);
    public static final double OUTTAKE_SPEED = 0.4;
    public static final double INTAKE_SPEED = -0.4;
    public static final double INTAKE_REST_SPEED = -0.1;
    public static final double OUTTAKE_REST_SPEED = 0;
  }
}
