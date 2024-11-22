// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
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
  private final CANSparkFlex pivotMotor =
      new CANSparkFlex(Constants.PivotConstants.PIVOT_MOTOR, MotorType.kBrushless);

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

    pivotMotor.setSmartCurrentLimit((int) (PivotConstants.MOTOR_CURRENT_LIMIT.in(Amps)));
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

  public boolean atGoal(Measure<Angle> goal, Measure<Angle> threshold) {
    return Math.abs(getMeasurement() - goal.in(Radians)) < threshold.in(Radians);
  }

  public void set(double speed) {
    pivotMotor.set(speed);
  }

  public Command PivotDown(PivotSubsystem pivot, double pivotSpeed, Measure<Angle> set) {
    return Commands.runEnd(
        () -> {
          if (getPosition().in(Degrees) < set.in(Degrees)) {
            set(pivotSpeed);
            SmartDashboard.putNumber("Encoder Position in Command", getPosition().in(Degree));
          } else {
            set(PivotConstants.MOTOR_ZERO_SPEED);
          }
        },
        () -> {
          set(PivotConstants.MOTOR_ZERO_SPEED);
        },
        pivot);
  }

  public Command PivotUp(PivotSubsystem pivot, double pivotSpeed) {
    return Commands.runEnd(
        () -> {
          if (Math.abs(getPosition().baseUnitMagnitude())
              < PivotConstants.PIVOT_MAX.baseUnitMagnitude()) {
            set(pivotSpeed);
            SmartDashboard.putNumber("Pivot Position (Degrees)", getPosition().in(Degrees));
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
    return getPosition().in(Radians);
  }

  public Measure<Angle> getPosition() {
    var position = pivotMotor.getEncoder().getPosition();
    return Degree.of(position / PivotConstants.GEAR_RATIO / 360);
  }
}
