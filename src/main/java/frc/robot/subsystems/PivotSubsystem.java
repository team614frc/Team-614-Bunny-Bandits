// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.PivotConstants;

/**
 * The PivotSubsystem has the motor objects for the motors of the Pivot on the robot. It also sets
 * them a value based on the input received from a command.
 *
 * @param pivotSpeed Variable represents the speed passed from a command that pivot motors should be
 *     set to
 * @returns pivotPosition through the getPosition()
 */
public class PivotSubsystem extends ProfiledPIDSubsystem {
  private CANSparkFlex pivotMotor;

  private ArmFeedforward feedforward =
      new ArmFeedforward(
          PivotConstants.PIVOT_kS,
          PivotConstants.PIVOT_kG,
          PivotConstants.PIVOT_kV,
          PivotConstants.PIVOT_kA);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    super(
        new ProfiledPIDController(
            PivotConstants.PIVOT_kP,
            PivotConstants.PIVOT_kI,
            PivotConstants.PIVOT_kD,
            new TrapezoidProfile.Constraints(
                PivotConstants.PIVOT_MAX_VEL, PivotConstants.PIVOT_MAX_ACCEL)));

    pivotMotor.setSmartCurrentLimit(PivotConstants.MOTOR_CURRENT_LIMIT);
    pivotMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    pivotMotor.getEncoder().setPosition(0);
    pivotMotor.setInverted(true);
    pivotMotor.burnFlash();

    SmartDashboard.putNumber("Pivot Height", getMeasurement());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feed = feedforward.calculate(setpoint.position, setpoint.velocity);
    pivotMotor.set(output + getController().calculate(getMeasurement() + feed));
  }

  public boolean atGoal(double goal, double threshold) {
    return Math.abs(getMeasurement() - goal) < threshold;
  }

  public void set(double speed) {
    pivotMotor.set(speed);
  }

  public Command PivotDown(double pivotSpeed, double set, PivotSubsystem pivot) {
    return Commands.runEnd(
        () -> {
          if (getEncoderinRadians() < set) {
            set(pivotSpeed);
            SmartDashboard.putNumber("Encoder Position in Command", getPivotEncoder());
          } else {
            set(PivotConstants.MOTOR_ZERO_SPEED);
          }
          set(PivotConstants.MOTOR_ZERO_SPEED);
        },
        () -> {
          set(PivotConstants.MOTOR_ZERO_SPEED);
        },
        pivot);
  }

  public Command PivotUp(double pivotSpeed, PivotSubsystem pivot) {
    return Commands.runEnd(
        () -> {
          if (Math.abs(getPivotEncoder()) < PivotConstants.PIVOT_MAX) {
            set(pivotSpeed);
            SmartDashboard.putNumber("Encoder Position in Command", getPivotEncoder());
          } else {
            set(PivotConstants.MOTOR_ZERO_SPEED);
          }
        },
        () -> {
          set(PivotConstants.MOTOR_ZERO_SPEED);
        },
        pivot);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderinRadians();
  }

  public double getPivotEncoder() {
    return pivotMotor.getEncoder().getPosition();
  }

  public double getEncoderinDegrees() {
    double val = 180.0 / 360.0; // 7168 ticks per rev, 180:1 gear ratio
    return (getPivotEncoder() / val);
  }

  public double getEncoderinRadians() {
    return (getEncoderinDegrees() * (Math.PI / 180.0));
  }
}
