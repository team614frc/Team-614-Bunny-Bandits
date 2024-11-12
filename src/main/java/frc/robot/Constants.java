// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutoConstants {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ManipulatorConstants {
    public static final int SHOOTER_MOTOR = 19;
    public static final int MOTOR_CURRENT_LIMIT = 40;
    public static final double SHOOT_MAX_VEL_SET = 6630.0;
    public static final double SCORE_AMP_SPEED = 2000;
    public static final double SCORE_SIMPLE = 0.8;
    public static final double SCORE_SIMPLE_RPM = 5000;
    public static final double SCORE_INPLACE_BABY_RPM = 1000;
    public static final double SCORE_INPLACE_TEEN_RPM = 1500;
    public static final double FEEDER_SHOT_RPM = 4500;
    public static final double SHOOTER_FEED = 0.2;
    public static final double AMP_SPEED = 0.2;
    public static final double SHOOTER_kFF = 0.000082;
    public static final double SHOOTER_kP = 0.00045;
    public static final double OUTTAKE_SPEED = -0.5;
    public static final double INTAKE_SPEED = 0.75;
    public static final double INTAKE_REST_SPEED = 0.00;
    public static final double SHOOTER_THRESHOLD = 150;
    public static final double PUKE_SPEED = -1;
    public static final double TRAP_SPEED = 1900;
    public static final double LOADBACK_SPEED = -0.08; // MAYBE BACK AND FORTH WAS THE MOVE
    public static final double LOADING_SPEED = 1; // (Note fixed its deformity)
    public static final double AMP_LOAD = 0.5;
    public static final double RUMBLE_TIMER = 2;
    public static final double RUMBLE_SETTING = .6;

  }
}
