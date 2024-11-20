// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  CANSparkFlex intakeMotor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkFlex(Constants.IntakeConstants.SHOOTER_MOTOR, MotorType.kBrushless);
    // shooterMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit((int)(Constants.IntakeConstants.MOTOR_CURRENT_LIMIT.in(Amp)));
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    intakeMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  public double getIntakeVelocity() {
    return intakeMotor.getEncoder().getVelocity();
  }

  public void set(Measure<Velocity<Angle>> speed) {
    intakeMotor.set(-speed.in(RPM));
  }

    public Command intakeBucket(IntakeSubsystem intake, Measure<Velocity<Angle>> intakeSpeed) {
    return Commands.runEnd(
        () -> {
          intake.set(intakeSpeed);
        },
        () -> {
          set(Constants.IntakeConstants.INTAKE_REST_SPEED);
        },
        intake);
  }

}