// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkFlex intakeMotor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkFlex(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(
        IntakeConstants
            .MOTOR_CURRENT_LIMIT); // makes sure the motors don't take too much amps from the
    // battery
    intakeMotor.setInverted(false); // Allows the motors to be able to spin in different directions.
    intakeMotor.setIdleMode(
        CANSparkFlex.IdleMode
            .kCoast); // Causes the motors to slow down in motion when no action is being commanded.
    intakeMotor.burnFlash(); // saves changes made to a devices configuration
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeed() {
    return intakeMotor.get();
  }

  public void setIntake(double intakeSpeed) {
    intakeMotor.set(-intakeSpeed); // Allows different motors to spin opposite ways.
  }
}
