// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkFlex intakeMotor =
      new CANSparkFlex(Constants.IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.setSmartCurrentLimit((int) (Constants.IntakeConstants.MOTOR_CURRENT_LIMIT.in(Amp)));
    intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    intakeMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  public void set(double speed) {
    intakeMotor.set(speed);
  }

  public Command intakeBucket() {
    return Commands.runEnd(
        () -> {
          set(Constants.IntakeConstants.INTAKE_SPEED);
        },
        () -> {
          set(Constants.IntakeConstants.INTAKE_REST_SPEED);
        });
  }

  public Command outtakeBucket() {
    return Commands.runEnd(
        () -> {
          set(Constants.IntakeConstants.OUTTAKE_SPEED);
        },
        () -> {
          set(Constants.IntakeConstants.OUTTAKE_REST_SPEED);
        });
  }
}
